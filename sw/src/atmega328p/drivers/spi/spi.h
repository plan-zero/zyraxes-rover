
#ifndef SPI_H
#define SPI_H

#include <avr/io.h>
#include "stdint.h"

#define SCK  PB5
#define MISO PB4
#define MOSI PB3
#define SS   PB2
#define SPI_DDR  DDRB
#define SPI_PORT PORTB

void spi_tx_8(uint8_t data);
void spi_tx_16(uint16_t data);
uint8_t spi_rx_8(uint8_t data);
uint16_t spi_rx_16(uint16_t data);

void spi_master_init(void);

#endif /*SPI_H*/