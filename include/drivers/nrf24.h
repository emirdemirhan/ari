/**
 * @file nrf24.h
 * @brief Driver for nRF24L01+ 2.4GHz Transceiver Module
 * @version 1.0
 */

 #ifndef NRF24_H
 #define NRF24_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "ari_error.h"
 

 
 /* --- nRF24L01+ Commands --- */
 #define NRF_CMD_R_REGISTER    0x00      /**< Read command */
 #define NRF_CMD_W_REGISTER    0x20      /**< Write command */
 #define NRF_CMD_R_RX_PAYLOAD  0x61      /**< Read RX payload */
 #define NRF_CMD_W_TX_PAYLOAD  0xA0      /**< Write TX payload */
 #define NRF_CMD_FLUSH_TX      0xE1      /**< Flush TX FIFO */
 #define NRF_CMD_FLUSH_RX      0xE2      /**< Flush RX FIFO */
 #define NRF_CMD_REUSE_TX_PL   0xE3      /**< Reuse TX Payload */
 #define NRF_CMD_NOP           0xFF      /**< No Operation */
 
 /* --- Register Map --- */
 #define NRF_REG_CONFIG        0x00      /**< Configuration Register */
 #define NRF_REG_EN_AA         0x01      /**< Enable 'Auto Acknowledgment' */
 #define NRF_REG_EN_RXADDR     0x02      /**< Enabled RX Addresses */
 #define NRF_REG_SETUP_AW      0x03      /**< Setup of Address Widths */
 #define NRF_REG_SETUP_RETR    0x04      /**< Setup of Automatic Retransmission */
 #define NRF_REG_RF_CH         0x05      /**< RF Channel */
 #define NRF_REG_RF_SETUP      0x06      /**< RF Setup Register */
 #define NRF_REG_STATUS        0x07      /**< Status Register */
 #define NRF_REG_OBSERVE_TX    0x08      /**< Transmit observe register */
 #define NRF_REG_RPD           0x09      /**< Received Power Detector */
 #define NRF_REG_RX_ADDR_P0    0x0A      /**< Receive address data pipe 0 */
 #define NRF_REG_TX_ADDR       0x10      /**< Transmit address */
 #define NRF_REG_RX_PW_P0      0x11      /**< Number of bytes in RX payload in data pipe 0 */
 #define NRF_REG_FIFO_STATUS   0x17      /**< FIFO Status Register */
 
 /* --- Constants --- */
 #define NRF_PAYLOAD_SIZE      32        /**< Fixed payload size (bytes) */
 #define NRF_ADDR_WIDTH        5         /**< Address width (bytes) */
 
 /* --- Function Prototypes --- */
 
 /**
  * @brief Initializes the SPI and GPIOs for nRF24L01.
  * @note  Call this once at system startup.
  */
 void nrf24_init(void);
 
 /**
  * @brief Configures the module settings (Channel, Payload size, etc.).
  * @param channel RF Channel (0-125). formula: Freq = 2400 + channel [MHz].
  * @param payload_size Size of the packet in bytes (max 32).
  */
 void nrf24_config(uint8_t channel, uint8_t payload_size);
 
 /**
  * @brief Sets the transmit address.
  * @param addr Pointer to the 5-byte address array.
  */
 void nrf24_set_tx_address(uint8_t *addr);
 
 /**
  * @brief Sets the receive address for Pipe 0.
  * @param addr Pointer to the 5-byte address array.
  */
 void nrf24_set_rx_address(uint8_t *addr);
 
 /**
  * @brief Puts the module into TX (Transmit) Mode.
  */
 void nrf24_mode_tx(void);
 
 /**
  * @brief Puts the module into RX (Receive) Mode.
  */
 void nrf24_mode_rx(void);
 
 /**
  * @brief Checks if there is data available in the RX FIFO.
  * @return true if data is ready, false otherwise.
  */
 bool nrf24_data_available(void);
 
 /**
  * @brief Reads a payload from the RX FIFO.
  * @param data Buffer to store the received data.
  * @param len Number of bytes to read (must match configured payload size).
  */
 void nrf24_read(uint8_t *data, uint8_t len);
 
 /**
  * @brief Sends a payload.
  * @param data Buffer containing data to send.
  * @param len Number of bytes to send.
  */
 void nrf24_write(uint8_t *data, uint8_t len);

 /**
 * @brief Verifies the hardware connection (SPI Ping Test).
 * * Writes a specific value to a read/write register (RF_CH) and reads it back.
 * If the read value matches the written value, it confirms that SPI wiring is correct.
 * * @return nrfsetup_success  SPI connection is OK.
 * @return nrfsetup_fail Connection failed (Check wiring or power).
 */
nrf_err_t nrf24_test_connection(void);
 
 #endif /* NRF24_H */