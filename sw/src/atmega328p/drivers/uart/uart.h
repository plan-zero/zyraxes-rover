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



#ifndef _UART_H
#define _UART_H

#include "stdint.h"

#define UART_RX_MAX 40
#define UART_RX_ERR 255

#define UART_PARITY_NONE 0
#define UART_PARITY_EVEN 1
#define UART_PARITY_ODD  2

#define UART_1MHZ 0
#define UART_2MHZ 1
#define UART_4MHZ 2
#define UART_8MHZ 3
#define UART_16MHZ 4
#define UART_9216MHZ 5
#define UART_110592MHZ 6
#define UART_16384MHZ 7
#define UART_COUNT_MHZ 8

#define UART_9600BAUD  0
#define UART_19200BAUD 1
#define UART_57600BAUD 2
#define UART_115200BAUD 3
#define UART_250000BAUD 4
#define UART_576000BAUD 5
#define UART_921600BAUD 6
#define UART_COUNT_BAUD 7

typedef enum {
	UART_INVALID_ARG,
	UART_BAUD_NOT_SUPPORTED,
	UART_CONFIG_OK	
}uart_err;

#define uartPrintHex(x) (x > 9 ? uart_sendByte('A' + (x - 10 ) ) : uart_sendByte('0' + x))
#define uartNewLine() /*
		*/ uart_sendByte(10); /*
		*/ uart_sendByte(13)

extern uart_err uart_init(uint8_t baud, uint8_t cpu_freq, uint8_t uart_parity);
extern void uart_sendByte(uint8_t byte);
extern uint8_t uart_sendByteNotBlocking(uint8_t byte);
extern uint8_t uart_rx_flush(uint8_t *buffer, uint8_t *rx_error);
extern void uart_printRegister(unsigned char reg);
extern void uart_printString(char *string, char crlf);
extern volatile uint8_t uart_rx_err_state;


#endif//_UART_H
