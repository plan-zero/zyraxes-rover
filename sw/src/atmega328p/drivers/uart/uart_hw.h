// Copyright (C) 2020 Coding Night Romania
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


#ifndef _UART_HW_H
#define _UART_HW_H

#include <avr/io.h>
#include "stdint.h"

#define UART_BUSY_STATUS            0
#define UART_2X_ENABLE              0

#define UART_HW_SET_LBAUD(x)        UBRR0L = x
#define UART_HW_SET_HBAUD(x)        UBRR0H = x
#define UART_HW_SET_PARITY_EVEN()   UCSR0C |= (0x1) << UPM01
#define UART_HW_SET_PARITY_ODD()    UCSR0C |= (0x1) << UPM01 | (0x1) << UPM00
#define UART_HW_SET_DATA(x)         UDR0 = x
#define UART_HW_GET_DATA()          UDR0
#define UART_HW_IS_BUSY()           ((UCSR0A >> UDRE0) & 0x1)


#define UART_HW_RXCIE()              UCSR0B |= (0x1 << RXCIE0)
#define UART_HW_TXCIE()              UCSR0B |= (0x1 << TXCIE0)

#if (UART_2X_ENABLE == 1)
    #define UART_HW_ENABLE_2X()          UCSR0A  = (0x1 << U2X0)
#elif (UART_2X_ENABLE == 0)
    #define UART_HW_ENABLE_2X()          __asm__("nop");
#endif
#define UART_HW_IS_2X()              (UCSR0A >> U2X0) & 0x1

#define UART_HW_SETUP_CODE() \
	UCSR0B  |= (0x1 << TXEN0) | (0x1 << RXEN0); \
	UCSR0C  |= (0x1 << UCSZ00) | (0x1 << UCSZ01)


#define _UART_1MHZ 0
#define _UART_2MHZ 1
#define _UART_4MHZ 2
#define _UART_8MHZ 3
#define _UART_16MHZ 4
#define _UART_9216MHZ 5
#define _UART_110592MHZ 6
#define _UART_16384MHZ 7
#define _UART_COUNT_MHZ 9

#define _UART_9600BAUD  0
#define _UART_19200BAUD 1
#define _UART_57600BAUD 2
#define _UART_115200BAUD 3
#define _UART_250000BAUD 4
#define _UART_576000BAUD 5
#define _UART_921600BAUD 6
#define _UART_COUNT_BAUD 7

#define UART_ERROR_STATE()      ((UCSR0A & 0x1C) >> 2)
#define UART_ERROR_CLEAR()      UCSR0A &= ~0x1C

#if (UART_2X_ENABLE == 1)
const int16_t uart_values[_UART_COUNT_MHZ][_UART_COUNT_BAUD] = 
{
// 9600	  19200  57600  115200  250000	576000 921600 (U2X = 0)
	{12,  6,     1,     0,      -1,     -1,    -1},
	{25,  12,    3,     1,      0,      -1,    -1},
	{51,  25,    8,     3,      1,      -1,    -1},
	{103, 51,    16,    8,      3,      -1,    -1},
	{207, 103,   34,    16,     7,      -1,    -1},
	{119, 59,    19,    9,      4,       1,     0},  //9.216MHZ
	{143, 71,    23,    11,     4       -1,    -1},  //11,059,200MHZ
	{212, 106,  35,     17,      7,      3,     1}

};
#elif (UART_2X_ENABLE == 0)
const int16_t uart_values[_UART_COUNT_MHZ][_UART_COUNT_BAUD] = 
{
// 9600	  19200  57600  115200  250000	576000 (U2X = 0)
	{12,  6,     1,     0,      -1,     -1,    -1}, //1MHZ
	{25,  12,    3,     1,      0,      -1,    -1}, //2MHZ
	{51,  25,    8,     3,      1,      -1,    -1}, //4MHZ
	{103, 51,    16,    8,      3,      -1,    -1}, //8MHZ
	{207, 103,   34,    16,     7,      -1,    -1}, //16MHZ
	{59,  29,    9,     4,       1      -1,    -1},  //9.216MHZ
	{71,  35,   11,     5,       2,     -1,    -1}, //11.059.200MHZ
	{106, 52,   17,     8,      3,       1,     0}   //16384000 MHZ
};
#endif

#endif