# Copyright (C) 2023 Plan-Zero
# 
# This file is part of automatic-farm.
# 
# automatic-farm is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# automatic-farm is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with automatic-farm.  If not, see <http://www.gnu.org/licenses/>.

# Template for a AVR project
cmake_minimum_required(VERSION 2.8)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/../../../cmake/generic-avr-gcc.cmake")


project(firmware_attiny24)

set(SPI_HEADER_INC
    ${CMAKE_SOURCE_DIR}/../drivers/spi
)
include_directories(${SPI_HEADER_INC})

set(FIRMWARE_SRC
    ${CMAKE_SOURCE_DIR}/../drivers/spi/spi_driver.c
    ${CMAKE_SOURCE_DIR}/firmware.c
    ${CMAKE_SOURCE_DIR}/sin_lut_microstep_16.c
)
avr_add_executable(firmware ${FIRMWARE_SRC})

