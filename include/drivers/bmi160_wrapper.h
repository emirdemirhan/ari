#ifndef BMI160_WRAPPER_H
#define BMI160_WRAPPER_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include "drivers/delay.h"

// --- PIN DEFINITIONS (Adjust for your schematic) ---
#define BMI_SPI     SPI2
#define BMI_CS_PORT GPIOB
#define BMI_CS_PIN  GPIO12

/* * STATIC INLINE FUNCTIONS
 * These functions are embedded directly into the source code during compilation.
 * This eliminates function call overhead (Branching), providing maximum performance.
 */

// Drive CS (Chip Select) Pin Low to start transaction
static inline void bmi160_cs_low(void) {
    gpio_clear(BMI_CS_PORT, BMI_CS_PIN);
}

// Drive CS (Chip Select) Pin High to end transaction
static inline void bmi160_cs_high(void) {
    gpio_set(BMI_CS_PORT, BMI_CS_PIN);
}

// Send and Receive 1 Byte via SPI
static inline uint8_t bmi160_spi_xfer(uint8_t data) {
    spi_send(BMI_SPI, data);
    while ((SPI_SR(BMI_SPI) & SPI_SR_RXNE) == 0);
    uint8_t valret = spi_read(BMI_SPI); 
    return valret;
}

// Simple Delay Function (Microseconds)
// Note: This is a rough blocking delay based on loop cycles.
static inline void bmi160_delay_us(uint32_t us) {
    // Rough calculation for 100MHz clock
    delay_us(us);
}

// --- Low-Level Driver Functions ---

// Write to a register
static inline void bmi160_ll_write(uint8_t reg, uint8_t data) {
    bmi160_cs_low();
    bmi160_spi_xfer(reg & 0x7F); // Mask for Write (Bit 0 = 0)
    bmi160_spi_xfer(data);
    bmi160_cs_high();
}

// Read from a register
static inline uint8_t bmi160_ll_read(uint8_t reg) {
    uint8_t val;
    bmi160_cs_low();
    bmi160_spi_xfer(reg | 0x80); // Mask for Read (Bit 0 = 1)
    val = bmi160_spi_xfer(0xFF); // Send dummy byte to clock out data
    
    bmi160_cs_high();
    return val;
}

// Burst Read (Multiple bytes)
static inline void bmi160_ll_read_burst(uint8_t reg, uint8_t *data, uint16_t len) {
    bmi160_cs_low();
    bmi160_spi_xfer(reg | 0x80); // Mask for Read
    for (uint16_t i = 0; i < len; i++) {
        data[i] = bmi160_spi_xfer(0xFF); // Send dummy bytes
    }
    bmi160_cs_high();
}

#endif // BMI160_WRAPPER_H