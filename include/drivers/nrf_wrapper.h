#ifndef NRF_WRAPPER_H
#define NRF_WRAPPER_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

 /* --- Hardware Configuration (Adjust for your Board) --- */
 #define NRF_SPI             SPI1
 #define NRF_PORT_CSN        GPIOA       /**< Chip Select Not Port */
 #define NRF_PIN_CSN         GPIO4       /**< Chip Select Not Pin */
 #define NRF_PORT_CE         GPIOA       /**< Chip Enable Port */
 #define NRF_PIN_CE          GPIO3       /**< Chip Enable Pin */


/**
  * @brief Sends/Receives one byte via SPI.
  * @param data Byte to send.
  * @return Received byte.
  */
 static inline uint8_t spi_transfer(uint8_t data) {
    spi_send(NRF_SPI, data);
    return (uint8_t)spi_read(NRF_SPI);
}


 /* --- Private Helper Functions --- */
 
 /**
  * @brief Selects the slave (CSN Low).
  */
 static inline void cs_select(void) {
    gpio_clear(NRF_PORT_CSN, NRF_PIN_CSN);
}

/**
 * @brief Deselects the slave (CSN High).
 */
static inline void cs_deselect(void) {
    gpio_set(NRF_PORT_CSN, NRF_PIN_CSN);
}

/**
 * @brief Enables the chip (CE High).
 */
static inline void ce_enable(void) {
    gpio_set(NRF_PORT_CE, NRF_PIN_CE);
}

/**
 * @brief Disables the chip (CE Low).
 */
static inline void ce_disable(void) {
    gpio_clear(NRF_PORT_CE, NRF_PIN_CE);
}


#endif