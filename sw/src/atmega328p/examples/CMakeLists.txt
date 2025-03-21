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


project(examples_atmega328p)

set(DEMO_HEADER_INC
    ${CMAKE_SOURCE_DIR}/../drivers/uart
    ${CMAKE_SOURCE_DIR}/../drivers/interrupts
    ${CMAKE_SOURCE_DIR}/../drivers/twi_slave
    ${CMAKE_SOURCE_DIR}/../drivers/spi
)
include_directories(${DEMO_HEADER_INC})



# Basic stepper motor code (for wiring checking)

set(MOTOR_DEMO_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/stepper_demo.c
)

set(TWI_DEMO_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/twi_example.c
    ${CMAKE_SOURCE_DIR}/../drivers/uart/uart.c
    ${CMAKE_SOURCE_DIR}/../drivers/twi_slave/twi.c
)

SET(SPI_EXAMPLE_SOURCE_FILES 
    ${CMAKE_SOURCE_DIR}/spi_example.c
    ${CMAKE_SOURCE_DIR}/../drivers/uart/uart.c
    ${CMAKE_SOURCE_DIR}/../drivers/spi/spi.c
)


avr_add_executable(stepper_demo ${MOTOR_DEMO_SOURCE_FILES})
avr_add_executable(twi_demo ${TWI_DEMO_SOURCE_FILES})
avr_add_executable(spi_demo ${SPI_EXAMPLE_SOURCE_FILES})


