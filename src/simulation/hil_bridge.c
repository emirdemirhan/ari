#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "simulation/hil_bridge.h"


// Paket başlıkları
#define HIL_HEADER_1 0xAA
#define HIL_HEADER_2 0xBB
#define HIL_FOOTER   0x55


// Paketi başarıyla okursa 1, bozuk çıkarsa 0 döndürür
uint8_t hil_receive_full_packet(HIL_Sim_Data_t *data_out) {
    uint8_t byte_in;
    uint8_t checksum = 0;
    uint8_t *ptr = (uint8_t *)data_out;
    const uint16_t payload_size = sizeof(HIL_Sim_Data_t); // Şu anki boyutu: 56 Byte

    while (1) {
        // --- 1. ÇİFT HEADER ARAMASI (Senkronizasyon) ---
        byte_in = usart_recv_blocking(USART1);
        if (byte_in != HIL_HEADER_1) continue;
        
        byte_in = usart_recv_blocking(USART1);
        if (byte_in != HIL_HEADER_2) continue; // İkinci header tutmazsa başa dön

        // --- 2. PAYLOAD (ASIL VERİ) OKUMA VE CHECKSUM ---
        checksum = 0;
        for (uint16_t i = 0; i < payload_size; i++) {
            ptr[i] = usart_recv_blocking(USART1);
            checksum ^= ptr[i]; // Tüm verilerin XOR toplamını al
        }

        // --- 3. KONTROL BAYTLARINI OKU ---
        uint8_t rx_checksum = usart_recv_blocking(USART1);
        uint8_t footer = usart_recv_blocking(USART1);

        // --- 4. PAKET DOĞRULAMA ---
        if (footer == HIL_FOOTER && rx_checksum == checksum) {
            return 1; // Mükemmel Paket!
        }
        
        // Eğer checksum hatalıysa veri yolda bozulmuştur.
        // Fonksiyon 0 döndürerek bu paketi çöpe atmamızı söyler.
        return 0; 
    }
}

void hil_send_motor_cmds(float main_pwm, float tail_pwm) {
    HIL_Motor_Data_t motors;
    motors.main_motor_pwm = main_pwm;
    motors.tail_motor_pwm = tail_pwm;

    uint8_t *ptr = (uint8_t *)&motors;
    uint8_t checksum = 0;

    // 1. Çift Header Gönder
    usart_send_blocking(USART1, HIL_HEADER_1);
    usart_send_blocking(USART1, HIL_HEADER_2);

    // 2. Payload Gönder ve Checksum Hesapla
    for (int i = 0; i < sizeof(HIL_Motor_Data_t); i++) {
        usart_send_blocking(USART1, ptr[i]);
        checksum ^= ptr[i];
    }

    // 3. Checksum ve Footer Gönder
    usart_send_blocking(USART1, checksum);
    usart_send_blocking(USART1, HIL_FOOTER);
}


/* --- YARDIMCI GÖNDERME FONKSİYONU --- */
void hil_send(const char *str) {
    while (*str) {
        usart_send_blocking(USART1, *str++);
    }
}

void hil_receive(char *str)
{
    for (int i = 0; i < RECEIVE_BYTES; i++)
    {
        str[i] = usart_recv_blocking(USART1);
    }
}

void hil_send_float(float val) {
    uint8_t *p = (uint8_t *)&val;
    for (int i = 0; i < 4; i++) {
        usart_send_blocking(USART1, p[i]);
    }
}




void start_usb_hil_system(void)
{
    /* USART1 FOR SIMULATION */
    rcc_periph_clock_enable(RCC_USART1);
    /* USART ENABLE FOR SIMULATION MODE ONLY !!!!!!!!!!!! */
    /* 2. Pinleri Alternate Function (AF) moduna al */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    
    /* 3. PA9 ve PA10'u AF7 (USART1) olarak ayarla */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

    /* 4. USART Ayarları (2000000 Baud, 8 Data Bit, No Parity, 1 Stop Bit) */
    usart_set_baudrate(USART1, 2000000); // 2 Mbps
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* 5. USART'ı Başlat */
    usart_enable(USART1);

    const char *test = "Hello from STM32\r\n";
    hil_send(test); 

}