#ifndef SPI_H
#define SPI_H

#include "asf.h"

#define SPI_CHIP_PCS_5 0x4
#define SPI_CHIP_PCS_4 0x5

#define SPI_CHIP_PCS_3 0x2
#define SPI_CHIP_PCS_2 0x3

#define SPI_CHIP_PCS_1 0x0//0x0D//spi_get_pcs(SPI_CHIP_SEL_1)
#define SPI_CHIP_PCS_0  0x01//0x0E//spi_get_pcs(SPI_CHIP_SEL_0)

void spi_master_initialize(void);
uint8_t spi_sync_transfer(uint8_t in_data, uint8_t cs, uint8_t last);

#endif /*SPI_H*/