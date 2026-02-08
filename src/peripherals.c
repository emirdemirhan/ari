#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include "peripherals.h"

void setup_perihperals(void)
{
    /* LED blink configuration */
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);  // LED blink config
    
    /* SPI configuration */
    gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_SCK | SPI_MOSI | SPI_MISO);     // SPI configuration
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);    // Set alternate function to SPI pins
    
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

}
