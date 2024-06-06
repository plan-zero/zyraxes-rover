// Copyright (C) 2023 Plan-Zero
// 
// This file is part of automatic-farm.
// 
// automatic-farm is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// automatic-farm is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with automatic-farm.  If not, see <http://www.gnu.org/licenses/>.


#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#define SPI_DATA_SIZE 5
#define SPI_DATA_CHECKSUM_POS 3

#define SLAVE_ACK 0xAA

typedef enum {
    SlaveSYNC = 0xAA,
    SlaveSTEP = 0x77,
    SlaveSETUP = 0x78,
    SlaveERROR = 0x55
}SlaveACK;

typedef enum {
    MasterSYNC = 0x3,
    MasterSETUP = 0x2,
    MasterSTEP = 0x1,
    MasterCount
}MasterCMD;

typedef struct  {
	unsigned char transferComplete : 1; //!< True when transfer completed.
    unsigned char doChecksum : 1;
	unsigned char writeCollision : 1;   //!< True if put attempted during transfer.
	unsigned char cs_assert : 1; //!< True if in slaveMode and CS is asserted by master.
    unsigned char master_cmd : 2;
}usidriverStatus_t;

extern volatile unsigned  char spi_received_data[SPI_DATA_SIZE];
extern volatile usidriverStatus_t spiX_status;

void spiX_initslave(char spi_mode);
char spiX_put(unsigned char val);

#endif /*SPI_DRIVER_H*/