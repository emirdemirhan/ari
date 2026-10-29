#include <stdint.h>
#include <math.h>
#include <string.h>
#include "flightcontrol/controller.h"
#include "flightcontrol/stest_wrapper.h"
#include "flightcontrol/pid.h"

/*
 * Uçuş kontrol adımı: kestirim (Kalman + ESKF), kaskad PID ve motor mikseri.
 * Donanımdan bağımsızdır; firmware (ari_tasks.c, task_control) ve PC tarafındaki
 * SIL kütüphanesi (sil/sil_lib.c) aynı kodu çalıştırır.
 * Her çağrı bir 2 ms adımdır (500 Hz).
 */


/* Kontrol döngüsünü besleyen durum kestirimcisi.
   0: Lineer Kalman filtresi, 1: Error-state Kalman filtresi (ESKF, eskf.h).
   İki filtre de her döngüde çalışır; bu seçim sadece PID'lere hangisinin gittiğini belirler. */
#define USE_ESKF 1

/* Seçilen kestirimciden yükseklik ve dikey hızı alır */
static inline void estimator_getaltitude(float *height, float *v_speed)
{
#if USE_ESKF
    eskf_getaltitude(height, v_speed);
#else
    kalman_getaltitude(height, v_speed);
#endif
}

/* Seçilen kestirimciden euler açılarını alır */
static inline void estimator_getangles(float *roll, float *pitch, float *yaw)
{
#if USE_ESKF
    eskf_getangles(roll, pitch, yaw);
#else
    kalman_getangles(roll, pitch, yaw);
#endif
}


/* ---- Yükseklik kontrolü sabitleri ---- */
#define HOVER_BASE        40.0f   // Hover için tahmini baz itme (%); kalan farkı hız döngüsünün integratörü bulur
#define GROUND_REF_M      0.05f   // Yükseklik referansı bunun altındaysa "yerde bekle"
#define GROUND_HEIGHT_M   0.15f   // Menzil sensörü bunun altını okuyorsa araç yerde sayılır
#define IDLE_THRUST       5.0f    // Yerde beklerken tüm motorların komutu (%)
#define MIN_TILT_COS      0.7f    // Eğim telafisi en fazla 1 / 0.7 kat (yaklaşık 45 derece)
#define V_FILTER_ALPHA    0.1f    // Dikey hız alçak geçiren filtresi (500 Hz'de ~8 Hz kesim)
#define ALT_I_VMAX        0.3f    // m/s: araç dikeyde bundan yavaş hareket ederken hız döngüsü integratörü çalışır
// Yükseklik kaynağı: menzil sensörü (VL53L1X, ~4 m'ye kadar, hassas) geçerliyken o; menzil
// dışında veya engellenince barometre (stest_wrapper.cpp, baro_fuse). Yükseklik sınırı yoktur.
#define RANGE_LOSS_S      0.25f   // bu kadar süre geçerli menzil örneği yoksa "menzil kayıp" (barometreyle devam)
#define RANGE_RECOVER_N   5       // geri dönmek için art arda geçerli örnek sayısı (100 ms)
// Yükseklik güvenli modu: ne menzil ne barometre varsa yükseklik/hız kestirimi ivmeölçerle
// sürüklenir (manevrada saniyede ~0.3 m/s); bu kestirimle kontrol edilmez
#define BARO_LOSS_S       0.5f    // bu kadar süre geçerli barometre örneği yoksa barometre kayıp
#define FAILSAFE_DESCENT  3.0f    // %: güvenli modda hover itmesinden bu kadar az ver (yavaş alçalış)
#define FAILSAFE_TILT_MAX 0.17f   // rad (~10 derece): güvenli modda açı referansı sınırı
#define DESCENT_BASE      0.3f    // m/s: yere yakın en fazla alçalış hızı; her metre yükseklik için +1 m/s
#define MAX_TILT          0.52f   // rad (30 derece): roll ve pitch referansının toplam (vektörel) sınırı
#define ALT_FF_GAIN       0.7f    // Referans hızının ne kadarı ileri beslenir (1: gecikme yok ama rampa sonunda aşma)
#define ALT_FF_MAX        1.5f    // m/s: referans hızı ileri beslemesi sınırı (basamaklarda tek tık sıçramayı sınırlar)

/* ---- Açılış kalibrasyonu ve ivmeölçer güveni ---- */
#define CAL_SAMPLES        1000    // 2 s: araç yerde ve hareketsizken jiroskop/ivmeölçer ortalaması
#define CAL_MAX_RATE       0.15f   // rad/s: bundan hızlı dönüş varsa araç hareketli sayılır (bias +-3 dps'yi geçer)
#define CAL_MAX_ACC_DEV    0.5f    // m/s^2: | |a| - g | bundan büyükse araç hareketli sayılır
#define ACCEL_LEVEL_CAL    1       // 1: açılışta zeminin düz olduğu varsayılır ve ivmeölçer sıfırı da ölçülür
                                   //    (gerçek araçta bunun yerine bir kez düz zeminde ölçülüp saklanmalı)
#define GRAVITY            9.81f
// İvmeölçer gürültüsü çarpanı (büyük = ivmeölçere daha az güven; düzeltme zaman sabiti ~ sqrt(çarpan)):
//  - yerde ve sabit uçuşta ivmeölçer yerçekimini iyi gösterir; jiroskop kaymasını da o düzeltir
//  - pilot manevra yaparken ve sonrasında araç yatay ivmelenir, ivmeölçer "düz" der ve
//    kestirimi yanıltır; bu sırada düzeltme yavaşlatılır
#define ACCEL_NOISE_SCALE_GROUND    1.0f
#define ACCEL_NOISE_SCALE_STEADY    1.0f
#define ACCEL_NOISE_SCALE_MANEUVER  16.0f
#define MANEUVER_STICK              0.05f   // |roll| veya |pitch| çubuğu bunun üstündeyse manevra
#define MANEUVER_HOLD_S             6.0f    // çubuk bırakıldıktan / kalkıştan sonra araç yavaşlarken de düşük güven

/* Kontrolcü durumu (eskiden task_control yığınındaydı) */
static uint8_t loop_tick = 0;
static uint8_t landed = 1;        // Yerde bekleme durumu: referans yükseltilince kalkış
static pid_coeffs alt_pos, alt_spd, roll_ang, roll_rate, pitch_ang, pitch_rate, yaw_rate_pid;
static float height = 0, v_speed = 0;
static float target_v = 0;
static float alt_ref_prev = 0.0f;   // Önceki yükseklik referansı (referans hızı ileri beslemesi için)
static float range_loss_s = 0.0f;   // Son geçerli menzil örneğinden beri geçen süre
static uint8_t range_valid_count = 0;
static uint8_t range_lost = 0;      // Menzil kaybı: yükseklik barometreden
static float baro_loss_s = 0.0f;    // Son geçerli barometre örneğinden beri geçen süre
static uint8_t height_failsafe = 0; // Menzil ve barometre yok: hover itmesi tutulur
static float filtered_v_speed = 0.0f;
static float range_hist[3] = {0.0f, 0.0f, 0.0f};   // Son 3 menzil örneği (50 Hz), medyan filtresi için
static uint8_t calibrated = 0;
static uint16_t cal_count = 0;
static float cal_sum_g[3], cal_sum_a[3];
static float gyro_off[3], accel_off[3];               // Kalibrasyonla bulunan sıfır hataları
static float maneuver_timer = 0.0f;                   // Son manevradan beri kalan düşük güven süresi (s)
static float acc_sum[3];                              // ESKF güncellemesi için ivmeölçer toplamı
static uint16_t acc_count = 0;


/* Araç hareketsizse örneği biriktirir; CAL_SAMPLES örnek birikince sıfır hatalarını hesaplar.
   Hareket algılanırsa baştan başlar. */
static void calibration_step(float gx, float gy, float gz, float ax, float ay, float az)
{
    float rate = sqrtf(gx * gx + gy * gy + gz * gz);
    float acc = sqrtf(ax * ax + ay * ay + az * az);
    if (rate > CAL_MAX_RATE || fabsf(acc - GRAVITY) > CAL_MAX_ACC_DEV)
    {
        cal_count = 0;
        cal_sum_g[0] = cal_sum_g[1] = cal_sum_g[2] = 0.0f;
        cal_sum_a[0] = cal_sum_a[1] = cal_sum_a[2] = 0.0f;
        return;
    }
    cal_sum_g[0] += gx;  cal_sum_g[1] += gy;  cal_sum_g[2] += gz;
    cal_sum_a[0] += ax;  cal_sum_a[1] += ay;  cal_sum_a[2] += az;
    if (++cal_count < CAL_SAMPLES) return;

    for (int i = 0; i < 3; i++)
    {
        gyro_off[i] = cal_sum_g[i] / CAL_SAMPLES;
        accel_off[i] = ACCEL_LEVEL_CAL ? cal_sum_a[i] / CAL_SAMPLES : 0.0f;
    }
    if (ACCEL_LEVEL_CAL) accel_off[2] -= GRAVITY;
    calibrated = 1;
}


/* Üç değerin medyanı */
static inline float median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return (a > b) ? a : b;
}


void controller_init(void)
{
    loop_tick = 0;
    landed = 1;
    height = 0.0f;
    v_speed = 0.0f;
    target_v = 0.0f;
    alt_ref_prev = 0.0f;
    range_loss_s = 0.0f;
    range_valid_count = 0;
    range_lost = 0;
    baro_loss_s = 0.0f;
    height_failsafe = 0;
    filtered_v_speed = 0.0f;
    range_hist[0] = range_hist[1] = range_hist[2] = 0.0f;
    calibrated = 0;
    cal_count = 0;
    maneuver_timer = 0.0f;
    acc_sum[0] = acc_sum[1] = acc_sum[2] = 0.0f;
    acc_count = 0;
    for (int i = 0; i < 3; i++)
    {
        cal_sum_g[i] = cal_sum_a[i] = 0.0f;
        gyro_off[i] = accel_off[i] = 0.0f;
    }

    // ============================================================
    //  ALTITUDE: Cascade PID (Pozisyon → Hız → Motor)
    // ============================================================
    // Dış döngü (50 Hz): yükseklik hatası -> dikey hız isteği. Sadece P: menzil sensörü
    // gürültüsünün türevini almamak için D yok; hız sönümlemesini iç döngü yapar.
    alt_pos.Kp = 2.0f;
    alt_pos.Ki = 0.0f;
    alt_pos.Kd = 0.0f;
    alt_pos.y_prev = 0.0f;  alt_pos.e_sum = 0.0f;
    alt_pos.max_i = 0.0f;
    alt_pos.out_max = 2.0f;   // En fazla 2 m/s tırmanış
    alt_pos.out_min = -1.5f;  // En fazla 1.5 m/s alçalış (yere sert inmemek için)

    // İç döngü (500 Hz): dikey hız hatası -> itme düzeltmesi (%). Filtrelenmiş hız kullanılır.
    // I terimi +-30 %'e kadar çıkabilir: batarya voltajı, kütle ve ESC eğrisinden gelen hover
    // hatasını HOVER_BASE'e dokunmadan kapatır (thrust ~ komut^2 olan ESC'de hover ~%64).
    alt_spd.Kp = 15.0f;
    alt_spd.Ki = 5.0f;
    alt_spd.Kd = 0.0f;
    alt_spd.y_prev = 0.0f;  alt_spd.e_sum = 0.0f;
    alt_spd.max_i = 6.0f;     // Ki * max_i = 30 %
    alt_spd.out_max = 30.0f;  // Yükseklik motorlara max %30 etki etsin
    alt_spd.out_min = -30.0f;

    // ============================================================
    //  ROLL: Cascade PID (Açı → Açısal Hız → Motor PWM Farkı)
    // ============================================================
    roll_ang.Kp = 4.0f;
    roll_ang.Ki = 0.5f;
    roll_ang.Kd = 0.1f;
    roll_ang.y_prev = 0.0f;  roll_ang.e_sum = 0.0f;
    roll_ang.max_i = 1.0f;
    roll_ang.out_max = 2.0f;   
    roll_ang.out_min = -2.0f;

    roll_rate.Kp = 4.0f;       // Motor % farkı üretir; bir motorun ani verim kaybını hızlı karşılar
    roll_rate.Ki = 6.0f;       // I: +-10 %, bir motorun verim kaybı gibi kalıcı torku dengeler
    roll_rate.Kd = 0.05f;
    roll_rate.y_prev = 0.0f;  roll_rate.e_sum = 0.0f;
    roll_rate.max_i = 1.667f;
    roll_rate.out_max = 20.0f; // DİKKAT: Servo limiti (-0.3) iptal. Maksimum %20 asimetrik güç verebilir.
    roll_rate.out_min = -20.0f;

    // ============================================================
    //  PITCH: Cascade PID (Açı → Açısal Hız → Motor PWM Farkı)
    // ============================================================
    pitch_ang.Kp = 4.0f;
    pitch_ang.Ki = 0.5f;
    pitch_ang.Kd = 0.1f;
    pitch_ang.y_prev = 0.0f;  pitch_ang.e_sum = 0.0f;
    pitch_ang.max_i = 1.0f;
    pitch_ang.out_max = 2.0f;
    pitch_ang.out_min = -2.0f;

    pitch_rate.Kp = 4.0f;
    pitch_rate.Ki = 6.0f;      // I: +-10 %
    pitch_rate.Kd = 0.05f;
    pitch_rate.y_prev = 0.0f;  pitch_rate.e_sum = 0.0f;
    pitch_rate.max_i = 1.667f;
    pitch_rate.out_max = 20.0f;// Limit motor %20'sine çekildi
    pitch_rate.out_min = -20.0f;

    // ============================================================
    //  YAW: Tek PID (Açısal hız → Motor PWM Farkı)
    // ============================================================
    // Pervane sürükleme torku itmenin yalnızca ~%1.6'sı kadar: yaw için daha fazla fark gerekir.
    // Mikser doygunluk yönetimi açısal kontrolü korur, bu yüzden sınır %25'e çıkarıldı.
    yaw_rate_pid.Kp = 15.0f;
    yaw_rate_pid.Ki = 5.0f;
    yaw_rate_pid.Kd = 0.0f;
    yaw_rate_pid.y_prev = 0.0f;  yaw_rate_pid.e_sum = 0.0f;
    yaw_rate_pid.max_i = 2.0f;    // Ki * max_i = 10 %
    yaw_rate_pid.out_max = 25.0f;
    yaw_rate_pid.out_min = -25.0f;
}


void controller_step(const HIL_Sim_Data_t *in, HIL_Fc_Data_t *out)
{
    const HIL_Sim_Data_t sim_data = *in;

    // ========== AÇILIŞ KALİBRASYONU ==========
    // Araç yerde hareketsizken 2 s boyunca motorlar kapalı; kestirimciler ancak sıfır hataları
    // bulunduktan sonra, düz ve bias'sız bir başlangıçla çalışmaya başlar. Kalkış bekletilir.
    if (!calibrated)
    {
        calibration_step(sim_data.gyro[0], sim_data.gyro[1], sim_data.gyro[2],
                         sim_data.accel[0], sim_data.accel[1], sim_data.accel[2]);
        memset(out, 0, sizeof(*out));
        out->seq = sim_data.seq;
        out->flags = (USE_ESKF ? HIL_FC_FLAG_USE_ESKF : 0) | HIL_FC_FLAG_CALIBRATING;
        return;
    }

    float gyro[3], acc[3];
    for (int i = 0; i < 3; i++)
    {
        gyro[i] = sim_data.gyro[i] - gyro_off[i];
        acc[i] = sim_data.accel[i] - accel_off[i];
    }

    // İvmeölçere güven: yerde ve sabit uçuşta yüksek, manevrada ve hemen sonrasında düşük
    if (fabsf(sim_data.rc_roll) > MANEUVER_STICK || fabsf(sim_data.rc_pitch) > MANEUVER_STICK)
        maneuver_timer = MANEUVER_HOLD_S;
    else if (maneuver_timer > 0.0f)
        maneuver_timer -= 0.002f;
    float accel_scale = landed ? ACCEL_NOISE_SCALE_GROUND
                      : (maneuver_timer > 0.0f ? ACCEL_NOISE_SCALE_MANEUVER : ACCEL_NOISE_SCALE_STEADY);
    estimators_set_accel_noise_scale(accel_scale);

    // ========== KALMAN PREDICT (500 Hz) ==========
    kalman_predict_altitude(0, 0, acc[2]);
    kalman_predict_angles(gyro[0], gyro[1], gyro[2]);

    // ========== ESKF PREDICT (500 Hz) ==========
    // Yerde: yerçekimi + sıfır hız ölçümü; uçuşta: pervane sürüklemesi modeli (eskf.h)
    eskf_set_landed(landed);
    // Önce açılar: yükseklik tahmini ivmeyi ESKF açılarıyla dünya eksenine çevirir.
    eskf_predict_angles(gyro[0], gyro[1], gyro[2], acc[2]);
    eskf_predict_altitude(acc[0], acc[1], acc[2]);

    // ESKF güncellemesi 50 Hz: ivmeölçer bu 10 adım boyunca ortalanır (titreşimi bastırır)
    acc_sum[0] += acc[0];
    acc_sum[1] += acc[1];
    acc_sum[2] += acc[2];
    acc_count++;

    // Yükseklik referansı (m). Üst sınır yok: menzil sensörünün üstünde barometre kullanılır
    float alt_ref = sim_data.rc_thrust;
    if (alt_ref < 0.0f) alt_ref = 0.0f;

    // Menzil kaybı takibi (her adım): geçerli örnek yoksa süre birikir
    uint8_t range_valid = (sim_data.flags & HIL_SIM_FLAG_RANGE_VALID) != 0;
    if (range_valid)
    {
        range_loss_s = 0.0f;
    }
    else
    {
        range_loss_s += 0.002f;
        range_valid_count = 0;
        if (range_loss_s > RANGE_LOSS_S) range_lost = 1;
    }

    // Barometre takibi: sensör çalışıyorsa (geçerli örnek) süre sıfırlanır
    uint8_t baro_valid = (sim_data.flags & HIL_SIM_FLAG_BARO_VALID) != 0;
    if (baro_valid) baro_loss_s = 0.0f;
    else baro_loss_s += 0.002f;
    uint8_t baro_lost = baro_loss_s > BARO_LOSS_S;
    height_failsafe = range_lost && baro_lost;

    // ========== KALMAN + ESKF UPDATE (50 Hz) ==========
    if (loop_tick >= 10)
    {
        kalman_update_angles(acc[0], acc[1], acc[2]);
        eskf_update_angles(acc_sum[0] / acc_count, acc_sum[1] / acc_count, acc_sum[2] / acc_count);
        acc_sum[0] = acc_sum[1] = acc_sum[2] = 0.0f;
        acc_count = 0;

        // Menzil sensörü yansıma alamazsa (menzil dışı, engel, parlak/koyu yüzey) "geçersiz"
        // işaretler ve maksimum menzili raporlar; bu değer kullanılırsa araç kendini yüksekte
        // sanıp yere çakılır. Geçersizken menzil güncellemesi yapılmaz, yüksekliği barometre ölçer.
        if (range_valid)
        {
            // Kayıptan dönüşte medyan geçmişi eski (kayıp öncesi) örneklerle dolu: sıfırla
            if (range_lost && range_valid_count == 0)
                range_hist[0] = range_hist[1] = range_hist[2] = sim_data.altitude_z;

            // Tek örneklik aykırı değerleri (outlier) 3'lü medyanla at;
            // gerçek bir basamak (masa kenarı) yalnızca bir örnek (20 ms) gecikir
            range_hist[0] = range_hist[1];
            range_hist[1] = range_hist[2];
            range_hist[2] = sim_data.altitude_z;
            float range_m = median3(range_hist[0], range_hist[1], range_hist[2]);

            kalman_update_altitude(range_m);
            eskf_update_altitude(range_m);
        }

        // Barometre: menzil geçerliyken sadece sıfırını öğrenir, değilse yüksekliği ölçer
        if (baro_valid)
        {
            kalman_update_baro(sim_data.baro_alt, range_valid);
            eskf_update_baro(sim_data.baro_alt, range_valid);
        }

        if (range_valid)
        {
            // Kestirim birkaç güncellemeyle oturduktan sonra normal kontrole dön
            if (range_valid_count < 255) range_valid_count++;
            if (range_lost && range_valid_count >= RANGE_RECOVER_N) range_lost = 0;
        }
        
        estimator_getaltitude(&height, &v_speed);
        target_v = calculate_pid(&alt_pos, alt_ref, height, 0.02f);

        // İleri besleme: referans rampa yapıyorsa (kumanda düğmesi basılı) o hızı doğrudan iste;
        // yalnızca P ile araç rampanın (hız / Kp) kadar gerisinde kalır
        float ref_rate = (alt_ref - alt_ref_prev) / 0.02f;
        alt_ref_prev = alt_ref;
        if (ref_rate > ALT_FF_MAX) ref_rate = ALT_FF_MAX;
        if (ref_rate < -ALT_FF_MAX) ref_rate = -ALT_FF_MAX;
        target_v += ALT_FF_GAIN * ref_rate;
        if (target_v > alt_pos.out_max) target_v = alt_pos.out_max;
        if (target_v < alt_pos.out_min) target_v = alt_pos.out_min;

        // Yere yaklaşırken alçalış hızı sınırlanır: durmak için itme payı ve mesafe kalsın
        float descent_max = DESCENT_BASE + (height > 0.0f ? height : 0.0f);
        if (target_v < -descent_max) target_v = -descent_max;
        loop_tick = 0;
    }

    // ========== ROLL/PITCH AÇILARINI AL ==========
    float roll, pitch, yaw;
    estimator_getangles(&roll, &pitch, &yaw);

    // ========== ALTITUDE İÇ DÖNGÜ (500 Hz) ==========
    estimator_getaltitude(&height, &v_speed);

    // Kalman hızı her 50 Hz güncellemesinde basamak yapar; filtrelenmemiş hali itmede sıçrama üretir
    filtered_v_speed = (V_FILTER_ALPHA * v_speed) + ((1.0f - V_FILTER_ALPHA) * filtered_v_speed);

    // Yerde bekleme: referans ~0 ve araç yerdeyse rölanti. Havada referans 0'a inerse araç
    // önce yere iner, yere değince rölantiye geçer. Referans yükselince kalkış başlar.
    if (alt_ref < GROUND_REF_M && height < GROUND_HEIGHT_M && !height_failsafe) landed = 1;
    else if (alt_ref >= GROUND_REF_M)
    {
        // Kalkış da bir manevradır: rüzgârda araç kalkar kalkmaz sürüklenir ve ivmeölçeri yanıltır
        if (landed) maneuver_timer = MANEUVER_HOLD_S;
        landed = 0;
    }

    // Eğim telafisi: gövde eğikken itmenin dikey bileşeni cos(roll) * cos(pitch) kadar kalır
    float tilt_cos = cosf(roll) * cosf(pitch);
    if (tilt_cos < MIN_TILT_COS) tilt_cos = MIN_TILT_COS;

    float total_thrust;
    if (landed)
    {
        total_thrust = IDLE_THRUST;
        target_v = 0.0f;
        alt_pos.e_sum = 0.0f;
        alt_spd.e_sum = 0.0f;
        alt_spd.y_prev = filtered_v_speed;
    }
    else if (height_failsafe)
    {
        // Yükseklik güvenli modu (menzil ve barometre yok): yükseklik ve dikey hız kestirimi
        // kullanılmaz. Hız döngüsünün öğrendiği hover itmesi (I terimi, donduruldu) biraz eksik
        // verilir, araç yavaşça alçalır; bir sensör geri gelince normal kontrole dönülür.
        float hold = HOVER_BASE + alt_spd.Ki * alt_spd.e_sum - FAILSAFE_DESCENT;
        total_thrust = hold / tilt_cos;
        target_v = 0.0f;
        alt_spd.y_prev = filtered_v_speed;

        if (total_thrust > 90.0f) total_thrust = 90.0f;
        if (total_thrust < 10.0f) total_thrust = 10.0f;
    }
    else
    {
        // Koşullu integrasyon: araç tırmanırken/alçalırken I terimi birikmez, yoksa hedefe varınca
        // aşma (overshoot) yapar. I yalnızca hover itmesini bulmak içindir; bu yüzden araç dikeyde
        // neredeyse dururken çalışır. (Hız HATASINA bakmak yanlıştır: hover itmesi yanlışsa araç
        // hedefin altında durur, hız hatası büyük kalır ve integratör hiç çalışmaz.)
        float e_sum_before = alt_spd.e_sum;
        float base_thrust = calculate_pid(&alt_spd, target_v, filtered_v_speed, 0.002f);
        if (fabsf(filtered_v_speed) > ALT_I_VMAX) alt_spd.e_sum = e_sum_before;

        total_thrust = (HOVER_BASE + base_thrust) / tilt_cos;

        // Havada motorların durmasını engellemek için min %10, max %90 limit
        if (total_thrust > 90.0f) total_thrust = 90.0f;
        if (total_thrust < 10.0f) total_thrust = 10.0f;
    }

    // ========== ROLL CASCADE PID (500 Hz) ==========
    // Açı referansları: toplam eğim (iki eksen birlikte) sınırlanır. Tam roll + tam pitch
    // eksen başına 30 derece ama toplamda 41 derece eder; o eğimde hover için %33 fazla itme gerekir.
    float roll_ref = sim_data.rc_roll * 0.52f;
    float pitch_ref = sim_data.rc_pitch * 0.52f;
    float tilt_limit = height_failsafe ? FAILSAFE_TILT_MAX : MAX_TILT;
    float tilt_ref = sqrtf(roll_ref * roll_ref + pitch_ref * pitch_ref);
    if (tilt_ref > tilt_limit)
    {
        roll_ref *= tilt_limit / tilt_ref;
        pitch_ref *= tilt_limit / tilt_ref;
    }
    float roll_rate_ref = calculate_pid(&roll_ang, roll_ref, roll, 0.002f);
    float roll_out = calculate_pid(&roll_rate, roll_rate_ref, gyro[0], 0.002f);

    // ========== PITCH CASCADE PID (500 Hz) ==========
    float pitch_rate_ref = calculate_pid(&pitch_ang, pitch_ref, pitch, 0.002f);
    float pitch_out = calculate_pid(&pitch_rate, pitch_rate_ref, gyro[1], 0.002f);

    // ========== YAW PID (500 Hz) ==========
    float yaw_out = calculate_pid(&yaw_rate_pid, sim_data.rc_yaw_rate, gyro[2], 0.002f);

    // Yerdeyken açı/hız döngüleri motorlara karışmaz ve integratörleri birikmez
    if (landed)
    {
        roll_out = pitch_out = yaw_out = 0.0f;
        roll_ang.e_sum = roll_rate.e_sum = 0.0f;
        pitch_ang.e_sum = pitch_rate.e_sum = 0.0f;
        yaw_rate_pid.e_sum = 0.0f;
    }

    // ============================================================
    //  QUADCOPTER MOTOR MİKSERİ (X KONFİGÜRASYONU)
    // ============================================================
    
    // CCW pervaneler gövdeye CW (negatif z) tepki torku uygular.
    // Pozitif yaw (yukarıdan bakınca CCW) için CW motorlar (M3, M4) hızlanır.
    float m[4];
    m[0] = total_thrust - pitch_out - roll_out - yaw_out;   // M1: Ön-Sağ (CCW)
    m[1] = total_thrust + pitch_out + roll_out - yaw_out;   // M2: Arka-Sol (CCW)
    m[2] = total_thrust - pitch_out + roll_out + yaw_out;   // M3: Ön-Sol (CW)
    m[3] = total_thrust + pitch_out - roll_out + yaw_out;   // M4: Arka-Sağ (CW)

    // Doygunluk yönetimi: bir motor sınırı aşarsa dört motor birlikte kaydırılır. Motorlar
    // arasındaki fark (açısal kontrol) korunur, toplam itme bir an için değişir.
    float m_max = m[0], m_min = m[0];
    for (int i = 1; i < 4; i++)
    {
        if (m[i] > m_max) m_max = m[i];
        if (m[i] < m_min) m_min = m[i];
    }
    float shift = 0.0f;
    if (m_max > 100.0f) shift = 100.0f - m_max;
    else if (m_min < 0.0f) shift = -m_min;

    // Saturation (Motorlar %0'ın altına düşemez, %100'ü geçemez)
    for (int i = 0; i < 4; i++)
    {
        m[i] += shift;
        if (m[i] > 100.0f) m[i] = 100.0f;
        if (m[i] < 0.0f) m[i] = 0.0f;
    }

    // ========== ÇIKIŞ VE TELEMETRİ ==========
    out->seq = sim_data.seq;
    out->motor[0] = m[0];
    out->motor[1] = m[1];
    out->motor[2] = m[2];
    out->motor[3] = m[3];

    // Paketli (packed) yapı alanlarının adresi alınmaz, önce yerel değişkenlere okunur
    float est[12];
    kalman_getangles(&est[0], &est[1], &est[2]);
    eskf_getangles(&est[3], &est[4], &est[5]);
    kalman_getaltitude(&est[6], &est[7]);
    eskf_getaltitude(&est[8], &est[9]);
    eskf_getairvelocity(&est[10], &est[11]);

    out->kf_rpy[0] = est[0];
    out->kf_rpy[1] = est[1];
    out->kf_rpy[2] = est[2];
    out->eskf_rpy[0] = est[3];
    out->eskf_rpy[1] = est[4];
    out->eskf_rpy[2] = est[5];
    out->kf_alt = est[6];
    out->kf_vz = est[7];
    out->eskf_alt = est[8];
    out->eskf_vz = est[9];
    out->eskf_vel[0] = est[10];
    out->eskf_vel[1] = est[11];

    out->target_vz = target_v;
    out->total_thrust = total_thrust;
    out->roll_rate_ref = roll_rate_ref;
    out->pitch_rate_ref = pitch_rate_ref;

    out->exec_cycles = 0;
    out->flags = (USE_ESKF ? HIL_FC_FLAG_USE_ESKF : 0) | (landed ? HIL_FC_FLAG_LANDED : 0)
               | (range_lost ? HIL_FC_FLAG_RANGE_LOST : 0)
               | (height_failsafe ? HIL_FC_FLAG_HEIGHT_FAILSAFE : 0);

    loop_tick++;
}
