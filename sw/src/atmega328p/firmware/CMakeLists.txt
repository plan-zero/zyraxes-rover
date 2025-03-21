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
    ${CMAKE_SOURCE_DIR}/magnetic_sensor.c
    ${CMAKE_SOURCE_DIR}/../drivers/uart/uart.c
    ${CMAKE_SOURCE_DIR}/../drivers/twi_slave/twi.c
    ${CMAKE_SOURCE_DIR}/../drivers/spi/spi.c
)


avr_add_executable(firmware_m328p ${FIRMWARE_M328P_SRC})


