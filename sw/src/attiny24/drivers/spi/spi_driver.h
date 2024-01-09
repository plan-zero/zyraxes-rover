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


void spiX_initmaster(char spi_mode);
void spiX_initslave(char spi_mode);
char spiX_put(unsigned char val);
unsigned char spiX_get();
void spiX_wait();

#endif /*SPI_DRIVER_H*/