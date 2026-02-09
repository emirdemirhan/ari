/**
 * @file nrf24.c
 * @brief Implementation of nRF24L01+ Driver
 */

 #include "drivers/nrf24.h"
 #include "drivers/delay.h"
 #include "drivers/nrf_wrapper.h"
 
 
 /**
  * @brief Writes a value to a register.
  * @param reg Register address (5 bits).
  * @param value Value to write.
  */
 static void nrf_write_reg(uint8_t reg, uint8_t value) {
     cs_select();
     spi_transfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
     spi_transfer(value);
     cs_deselect();
 }
 
 /**
  * @brief Reads a value from a register.
  * @param reg Register address.
  * @return Register value.
  */
 static uint8_t nrf_read_reg(uint8_t reg) {
     uint8_t value;
     cs_select();
     spi_transfer(NRF_CMD_R_REGISTER | (reg & 0x1F));
     value = spi_transfer(0xFF); // Send dummy byte to read
     cs_deselect();
     return value;
 }
 
 /**
  * @brief Writes a buffer to a specific register/command.
  * @param cmd Command byte (e.g., Write TX Payload).
  * @param data Pointer to data buffer.
  * @param len Length of data.
  */
 static void nrf_write_buf(uint8_t cmd, uint8_t *data, uint8_t len) {
     cs_select();
     spi_transfer(cmd);
     for (uint8_t i = 0; i < len; i++) {
         spi_transfer(data[i]);
     }
     cs_deselect();
 }
 
 /**
  * @brief Reads a buffer from a specific register/command.
  * @param cmd Command byte.
  * @param data Pointer to buffer to store data.
  * @param len Length of data.
  */
 static void nrf_read_buf(uint8_t cmd, uint8_t *data, uint8_t len) {
     cs_select();
     spi_transfer(cmd);
     for (uint8_t i = 0; i < len; i++) {
         data[i] = spi_transfer(0xFF); // Dummy byte
     }
     cs_deselect();
 }
 
 /* --- Public API Implementation --- */
 
 void nrf24_init(void) {

    cs_deselect(); // Idle state High
 

    ce_disable(); // Idle state Low
     
    /* Wait for chip to stabilize (Power on reset) */
    delay_us(10);
 }
 
 void nrf24_config(uint8_t channel, uint8_t payload_size) {
     ce_disable();
 
     /* Set Address Width to 5 bytes */
     nrf_write_reg(NRF_REG_SETUP_AW, 0x03);
 
     /* Set RF Channel */
     nrf_write_reg(NRF_REG_RF_CH, channel);
 
     /* Set Payload Size for Pipe 0 */
     nrf_write_reg(NRF_REG_RX_PW_P0, payload_size);
 
     /* RF Setup: 1Mbps, 0dBm (Max Power) */
     /* 0x06 = 1Mbps, 0dBm. Use 0x26 for 250kbps */
     nrf_write_reg(NRF_REG_RF_SETUP, 0x06);
 
     /* Auto Acknowledgment: DISABLED (Crucial for Drone latency) */
     nrf_write_reg(NRF_REG_EN_AA, 0x00);
 
     /* Retransmission: DISABLED */
     nrf_write_reg(NRF_REG_SETUP_RETR, 0x00);
 
     /* Enable RX Pipe 0 */
     nrf_write_reg(NRF_REG_EN_RXADDR, 0x01);
 }
 
 void nrf24_set_tx_address(uint8_t *addr) {
     nrf_write_buf(NRF_CMD_W_REGISTER | NRF_REG_TX_ADDR, addr, 5);
 }
 
 void nrf24_set_rx_address(uint8_t *addr) {
     nrf_write_buf(NRF_CMD_W_REGISTER | NRF_REG_RX_ADDR_P0, addr, 5);
 }
 
 void nrf24_mode_tx(void) {
     ce_disable();
     
     uint8_t config = nrf_read_reg(NRF_REG_CONFIG);
     config &= ~(1 << 0); // Clear PRIM_RX bit (Set to TX)
     config |= (1 << 1);  // Set PWR_UP bit
     nrf_write_reg(NRF_REG_CONFIG, config);
 
     /* Flush FIFOs to start fresh */
     cs_select(); spi_transfer(NRF_CMD_FLUSH_TX); cs_deselect();
     
     // CE is handled during write operation
 }
 
 void nrf24_mode_rx(void) {
     ce_disable();
     
     uint8_t config = nrf_read_reg(NRF_REG_CONFIG);
     config |= (1 << 0);  // Set PRIM_RX bit (Set to RX)
     config |= (1 << 1);  // Set PWR_UP bit
     nrf_write_reg(NRF_REG_CONFIG, config);
 
     /* Flush FIFOs */
     cs_select(); spi_transfer(NRF_CMD_FLUSH_RX); cs_deselect();
     
     /* Clear Status Register (Clear interrupts) */
     nrf_write_reg(NRF_REG_STATUS, 0x70); 
 
     ce_enable(); // Start listening
     
     /* 130us startup delay */
     delay_us(130);
 }
 
 bool nrf24_data_available(void) {
     uint8_t status = nrf_read_reg(NRF_REG_STATUS);
     
     /* Bit 6 (RX_DR) indicates Data Ready */
     if (status & (1 << 6)) {
         /* Clear the interrupt flag by writing 1 to it */
         nrf_write_reg(NRF_REG_STATUS, (1 << 6));
         return true;
     }
     return false;
 }
 
 void nrf24_read(uint8_t *data, uint8_t len) {
     nrf_read_buf(NRF_CMD_R_RX_PAYLOAD, data, len);
     
     /* Clear status again to be safe */
     nrf_write_reg(NRF_REG_STATUS, (1 << 6));
 }
 
 void nrf24_write(uint8_t *data, uint8_t len) {
     /* To transmit, we load the FIFO and pulse CE for >10us */
     nrf_write_buf(NRF_CMD_W_TX_PAYLOAD, data, len);
     
     ce_enable();
     delay_us(10); // Short delay > 10us
     ce_disable();
 }

 nrf_err_t nrf24_test_connection(void) {
    uint8_t test_val = 0x35; // Hex 0x35 = Decimal 53 (random number)
    
    /* 1. Write it to register */
    nrf_write_reg(NRF_REG_RF_CH, test_val);
    
    /* 2. Read it back */
    uint8_t read_val = nrf_read_reg(NRF_REG_RF_CH);
    
    /* 3. Compare */
    if (read_val == test_val) {
        return nrfsetup_success; // Success
    }
    
    return nrfsetup_fail; // Fail
}