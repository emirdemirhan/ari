#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include "peripherals.h"
#include "drivers/delay.h"

void setup_perihperals(void)
{
    /* LED blink configuration */
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);  // LED blink config
    
    /* SPI configuration */
    gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_SCK | SPI_MOSI | SPI_MISO);     // SPI configuration
    gpio_set_af(GPIOA, GPIO_AF5, SPI_SCK | SPI_MOSI | SPI_MISO);    // Set alternate function to SPI pins
    
    /* NRF CSN CE configuration */
    gpio_mode_setup(SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NRF_CSN | NRF_CE);

    /* Init SPI */
    spi_init_master(NRF_SPI,
        SPI_CR1_BAUDRATE_FPCLK_DIV_16,   // 84MHz/16 = ~5.25 MHz 
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, // Idle Low
        SPI_CR1_CPHA_CLK_TRANSITION_1,   // Mode 0
        SPI_CR1_DFF_8BIT,                // 8-bit data
        SPI_CR1_MSBFIRST);               // Big endian

    spi_enable(NRF_SPI);

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
    gpio_set(GPIOB, GPIO12); // CS High (Pasif)

    // 3. SPI Pin Ayarları (SCK=PB13, MISO=PB14, MOSI=PB15)
    // Bu pinler de AF5 (Alternate Function 5) modunda çalışır.
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13 | GPIO14 | GPIO15);
    gpio_set_af(GPIOB, GPIO_AF5, GPIO13 | GPIO14 | GPIO15);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO13 | GPIO15);


    // 4. SPI2 Ayarları (Master Mode)
    // Dikkat: SPI2'nin base clock hızı (APB1) SPI1'den (APB2) farklı olabilir.
    // F412'de APB1 genelde 50MHz, APB2 100MHz'dir.
    // DIV_8 veya DIV_16 güvenlidir (3-6 MHz civarı).
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_256, 
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, 
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);
    

    // SPI Modülünü Aktif Et
    spi_enable(SPI2);

    /* DWT delay init */
    delay_init();

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
