#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "simulation/hil_bridge.h"



/* --- YARDIMCI GÖNDERME FONKSİYONU --- */
void usart_send(const char *str) {
    while (*str) {
        usart_send_blocking(USART1, *str++);
    }
}

void usart_receive(char *str)
{
    for (int i = 0; i < RECEIVE_BYTES; i++)
    {
        str[i] = usart_recv_blocking(USART1);
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

    /* 4. USART Ayarları (115200 Baud, 8 Data Bit, No Parity, 1 Stop Bit) */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* 5. USART'ı Başlat */
    usart_enable(USART1);

}