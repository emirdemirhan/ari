#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <libopencm3/stm32/gpio.h>

// Define constants

#define LED_PORT GPIOB
#define LED_PIN  GPIO2
#define SPI_PORT GPIOA
#define SPI_SCK  GPIO5
#define SPI_MISO  GPIO6
#define SPI_MOSI  GPIO7
#define NRF_CSN  GPIO4
#define NRF_CE  GPIO3
#define NRF_SPI SPI1_BASE



// Define functions

/* 
 @brief Configure and setup perihperals devices and pins 
 */
void setup_perihperals(void);


#endif