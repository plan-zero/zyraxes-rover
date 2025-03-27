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





project(firmware_atmega328p)

set(FIRMWARE_M328P_INC
    ${CMAKE_SOURCE_DIR}/../drivers/uart
    ${CMAKE_SOURCE_DIR}/../drivers/interrupts
    ${CMAKE_SOURCE_DIR}/../drivers/twi_slave
    ${CMAKE_SOURCE_DIR}/../drivers/spi
)
include_directories(${FIRMWARE_M328P_INC})



# Basic stepper motor code (for wiring checking)

set(FIRMWARE_M328P_SRC
    ${CMAKE_SOURCE_DIR}/firmware_m328p.c
    ${CMAKE_SOURCE_DIR}/firmware_hw.c
    ${CMAKE_SOURCE_DIR}/firmware_e2p.c
    ${CMAKE_SOURCE_DIR}/magnetic_sensor.c
    ${CMAKE_SOURCE_DIR}/../drivers/uart/uart.c
    ${CMAKE_SOURCE_DIR}/../drivers/twi_slave/twi.c
    ${CMAKE_SOURCE_DIR}/../drivers/spi/spi.c
)

#eeprom cmake setup

set(EEP_MEMORY_SECTION .eeprom)
set(EEPROM_START_ADDR "0")

set(E2P_DATA_VAR_NAME e2p_default_data CACHE INTERNAL "")
set(AVR_LFLAGS "-Wl,--relax,--gc-sections -Wl,--undefined=ptrs -Wl,--undefined=${E2P_DATA_VAR_NAME}")

#TWI ADDRESS 
set(TWI_CHIP_ADDRESS ${EEPROM_START_ADDR})
set(TWI_CHIP_ADDR_LENGTH  "1")

#FIRMWARE VERSION, this is 3 bytes as V<major>.<minor>.<patch>
MATH(EXPR FIRMWARE_VERSION_ADDRESS "${TWI_CHIP_ADDRESS} + ${TWI_CHIP_ADDR_LENGTH}")
set(FIRMWARE_VERSION_LENGTH "3")

MATH(EXPR E2P_SIZE "${FIRMWARE_VERSION_ADDRESS} + ${FIRMWARE_VERSION_LENGTH}")

set(PROGRAM_EEPROM  True)

if(E2P_CHIP_TWI_ADDRESS)
    add_definitions(-DSLAVE_TWI_CHIP_ADDRESS=0x${E2P_CHIP_TWI_ADDRESS})
else ()
    message("No Chip TWI address provided, set default: 0x70")
    add_definitions(-DSLAVE_TWI_CHIP_ADDRESS=0x70)
endif(E2P_CHIP_TWI_ADDRESS)

configure_file(firmware_e2p.h.in ${CMAKE_CURRENT_SOURCE_DIR}/firmware_e2p.h @ONLY)

avr_add_executable(firmware_m328p ${FIRMWARE_M328P_SRC})


