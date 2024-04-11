#ifndef SPI_H
#define SPI_H

#include "asf.h"

#define SPI_CHIP_PCS_15 0xF
#define SPI_CHIP_PCS_14 0xE

#define SPI_CHIP_PCS_13 0xD
#define SPI_CHIP_PCS_12 0xC

#define SPI_CHIP_PCS_11 0xB
#define SPI_CHIP_PCS_10 0xA

#define SPI_CHIP_PCS_9 0x9
#define SPI_CHIP_PCS_8 0x8

#define SPI_CHIP_PCS_7 0x7
#define SPI_CHIP_PCS_6 0x6

#define SPI_CHIP_PCS_5 0x5
#define SPI_CHIP_PCS_4 0x4

#define SPI_CHIP_PCS_3 0x3
#define SPI_CHIP_PCS_2 0x2

#define SPI_CHIP_PCS_1 0x1
#define SPI_CHIP_PCS_0  0x00

void spi_master_initialize(void);
uint8_t spi_sync_transfer(uint8_t in_data, uint8_t cs, uint8_t last);

#endif /*SPI_H*/