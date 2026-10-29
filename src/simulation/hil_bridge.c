#include <string.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include "simulation/hil_bridge.h"


/* Çerçeve çözücü ve son gönderilen cevap (aynı seq tekrar gelirse yeniden gönderilir) */
static hil_decoder_t rx_decoder;
static uint8_t last_reply[sizeof(HIL_Fc_Data_t) + HIL_FRAME_OVERHEAD];
static uint16_t last_reply_len = 0;
static uint32_t last_reply_seq = 0;

/* Alım sayaçları (hil_print_rx_stats) */
static volatile uint32_t rx_bytes, rx_fe, rx_ne, rx_ore, rx_drop, rx_frames;

/* Alım halka tamponu: USART1 kesmesi doldurur, kontrol görevi boşaltır.
   Eskiden görev USART'ı kendisi yokluyordu; LED görevi veya SysTick onu bir baytlık süreden
   (2 Mbps'te 5 us) uzun kesince bayt taşmayla (ORE) kayboluyor, çerçeve yeniden gönderiliyordu.
   Ayrıca ORE=1, RXNE=0 kalırsa (RM0402) yalnız RXNE'yi bekleyen döngü sonsuza kadar takılıyordu.
   Kesme önceliği 1, configMAX_SYSCALL_INTERRUPT_PRIORITY (5) değerinden yüksek: FreeRTOS kritik
   bölümleri onu maskeleyemez, bu yüzden kesme hiçbir FreeRTOS fonksiyonu çağırmamalı. */
#define RX_BUF_SIZE 1024u                       /* 2'nin kuvveti olmalı */
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint32_t rx_head, rx_tail;      /* head: kesme yazar, tail: görev yazar */
#define HIL_USART_IRQ_PRIORITY (1 << 4)


void usart1_isr(void)
{
    // SR sonra DR okumak RXNE, ORE, NE ve FE bayraklarının hepsini temizler. RXNEIE, ORE için de
    // kesme üretir; RXNE=0 iken ORE gelirse okunan DR eski bayttır ve atılır.
    uint32_t sr = USART_SR(USART1);
    if ((sr & (USART_SR_RXNE | USART_SR_ORE)) == 0) return;

    uint8_t byte = (uint8_t)USART_DR(USART1);
    if (sr & USART_SR_ORE) rx_ore++;
    if (sr & USART_SR_FE)  rx_fe++;
    if (sr & USART_SR_NE)  rx_ne++;
    if ((sr & USART_SR_RXNE) == 0) return;

    rx_bytes++;
    uint32_t head = rx_head;
    if (head - rx_tail >= RX_BUF_SIZE) {         // tampon dolu: bayt atılır
        rx_drop++;
        return;
    }
    rx_buf[head & (RX_BUF_SIZE - 1)] = byte;
    rx_head = head + 1;
}


// Bir bayt gelene kadar bekler (meşgul bekleme; aynı öncelikteki görevler zaman dilimiyle çalışır)
static uint8_t hil_read_byte(void)
{
    uint32_t tail = rx_tail;
    while (rx_head == tail) { }
    uint8_t byte = rx_buf[tail & (RX_BUF_SIZE - 1)];
    rx_tail = tail + 1;
    return byte;
}


static void hil_send_u32(uint32_t v);

static void hil_send_bytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        usart_send_blocking(USART1, data[i]);
    }
}


// Yeni bir simülasyon paketi gelene kadar bekler, 1 döndürür.
// CRC hatalı çerçeveler atlanır; aynı seq ile tekrar gelen paket kontrolcüyü
// tekrar çalıştırmaz, önbellekteki cevap yeniden gönderilir.
uint8_t hil_receive_full_packet(HIL_Sim_Data_t *data_out) {
    while (1) {
        uint8_t byte_in = hil_read_byte();

        if (!hil_decoder_push(&rx_decoder, byte_in)) continue;
        rx_frames++;

        // Simülatör yeni bir koşuya temiz durumla başlamak için kartı yeniden başlatır
        if (rx_decoder.type == HIL_TYPE_RESET) scb_reset_system();

        if (rx_decoder.type != HIL_TYPE_SIM_DATA || rx_decoder.len != sizeof(HIL_Sim_Data_t)) continue;

        memcpy(data_out, rx_decoder.payload, sizeof(HIL_Sim_Data_t));

        if (last_reply_len != 0 && data_out->seq == last_reply_seq) {
            hil_send_bytes(last_reply, last_reply_len);
            continue;
        }

        return 1;
    }
}

void hil_send_fc_data(const HIL_Fc_Data_t *data) {
    last_reply_len = hil_encode_frame(HIL_TYPE_FC_DATA, data, sizeof(HIL_Fc_Data_t),
                                      last_reply, sizeof(last_reply));
    last_reply_seq = data->seq;

    hil_send_bytes(last_reply, last_reply_len);
}


static void hil_send_u32(uint32_t v)
{
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    do {
        buf[--i] = (char)('0' + v % 10);
        v /= 10;
    } while (v && i > 0);
    hil_send(&buf[i]);
}

void hil_print_rx_stats(void)
{
    hil_send("RX bytes=");  hil_send_u32(rx_bytes);
    hil_send(" fe=");       hil_send_u32(rx_fe);
    hil_send(" ne=");       hil_send_u32(rx_ne);
    hil_send(" ore=");      hil_send_u32(rx_ore);
    hil_send(" drop=");     hil_send_u32(rx_drop);
    hil_send(" crc=");      hil_send_u32(rx_decoder.crc_errors);
    hil_send(" frames=");   hil_send_u32(rx_frames);
    hil_send("\r\n");
}


/* --- YARDIMCI GÖNDERME FONKSİYONU --- */
void hil_send(const char *str) {
    while (*str) {
        usart_send_blocking(USART1, *str++);
    }
}


void start_usb_hil_system(void)
{
    /* USART1 FOR SIMULATION */
    rcc_periph_clock_enable(RCC_USART1);
    /* USART ENABLE FOR SIMULATION MODE ONLY !!!!!!!!!!!! */
    /* 2. Pinleri Alternate Function (AF) moduna al */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO9);  // 2 Mbps TX için hızlı kenar

    /* 3. PA9 ve PA10'u AF7 (USART1) olarak ayarla */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

    /* 4. USART Ayarları (2000000 Baud, 8 Data Bit, No Parity, 1 Stop Bit) */
    usart_set_baudrate(USART1, HIL_BAUD); // varsayılan 2 Mbps
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* 5. Alım kesmesi (halka tampon, bkz. usart1_isr) ve USART'ı başlat */
    usart_enable_rx_interrupt(USART1);
    nvic_set_priority(NVIC_USART1_IRQ, HIL_USART_IRQ_PRIORITY);
    nvic_enable_irq(NVIC_USART1_IRQ);
    usart_enable(USART1);

    hil_decoder_reset(&rx_decoder);

    // Açılış mesajı ve son reset sebebi: IWDG = kart takıldı ve watchdog sıfırladı,
    // BOR/POR = besleme düştü veya yeni enerji verildi, PIN = reset düğmesi, SFT = simülatörün reset çerçevesi
    hil_send("Hello from STM32 (reset:");
    uint32_t csr = RCC_CSR;
    if (csr & RCC_CSR_IWDGRSTF) hil_send(" IWDG");
    if (csr & RCC_CSR_WWDGRSTF) hil_send(" WWDG");
    if (csr & RCC_CSR_PORRSTF)  hil_send(" POR");
    if (csr & RCC_CSR_BORRSTF)  hil_send(" BOR");
    if (csr & RCC_CSR_PINRSTF)  hil_send(" PIN");
    if (csr & RCC_CSR_SFTRSTF)  hil_send(" SFT");
    if (csr & RCC_CSR_LPWRRSTF) hil_send(" LPWR");
    // Paket boyutları: simülatör kendi boyutlarıyla karşılaştırır, eski firmware'i yakalar
    hil_send(") proto sim=");
    hil_send_u32(sizeof(HIL_Sim_Data_t));
    hil_send(" fc=");
    hil_send_u32(sizeof(HIL_Fc_Data_t));
    hil_send("\r\n");
    RCC_CSR |= RCC_CSR_RMVF;   // bayrakları temizle, bir sonraki reset kendi sebebini göstersin

}
