#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

enum{
    TWI_SLAVE_RX_PENDING = 0,
    TWI_SLAVE_TX_PENDING,
    TWI_SLAVE_RX_DONE,
    TWI_SLAVE_TX_DONE,
    TWI_SLAVE_READY,
    TWI_SLAVE_TRANSFER_ERROR
};

volatile uint8_t twi_rx_status;
volatile uint8_t twi_tx_status;
volatile uint8_t buffer_address;
volatile uint8_t txbuffer[0xFF];
volatile uint8_t rxbuffer[0xFF];

void I2C_init(uint8_t address);
void I2C_stop(void);

#endif // I2C_SLAVE_H
