#ifndef SPI_H
#define SPI_H

#include "asf.h"

#define SPI_CHIP_PCS_5 0x5
#define SPI_CHIP_PCS_4 0x4

#define SPI_CHIP_PCS_3 0x3
#define SPI_CHIP_PCS_2 0x2

#define SPI_CHIP_PCS_1 0x1
#define SPI_CHIP_PCS_0  0x00

void spi_master_initialize(void);
uint8_t spi_sync_transfer(uint8_t in_data, uint8_t cs, uint8_t last);

#endif /*SPI_H*/