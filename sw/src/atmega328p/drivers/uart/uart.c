/**
 * Copyright (C) 2020 Coding Night Romania
 * 
 * This file is part of automatic-farm.
 * 
 * automatic-farm is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * automatic-farm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with automatic-farm.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "uart.h"
#include "uart_hw.h"
#include "interrupt_hw.h"
#include <stdlib.h>



#define UART_IDLE 		0x0
#define UART_SEND 		0x1
#define UART_RECEIVE 	0x2
#define UART_OK			0x4
#define UART_NOK		0x5
#define UART_RX_ERROR   0x6



uint8_t uart_tx_state;
uint8_t uart_rx_state;
uint8_t uart_rx_buffer[UART_RX_MAX];
uint8_t uart_rx_index;

volatile uint8_t uart_rx_err_state;

INTERRUPT_ROUTINE(IRQ_USART_TXC_HANDLER)
{
	uart_tx_state = UART_IDLE;
}

INTERRUPT_ROUTINE(IRQ_USART_RXC_HANDLER)
{

	if ( UART_ERROR_STATE() != 0)
	{
		uart_rx_err_state = UART_ERROR_STATE();
		uart_rx_state = UART_RX_ERROR;
		UART_ERROR_CLEAR();
	}
	else 
	{
		uart_rx_state = UART_RECEIVE;
		uart_rx_buffer[uart_rx_index++] = UART_HW_GET_DATA();

		if(uart_rx_index >= UART_RX_MAX)
		{
			uart_rx_index = 0;
		}
	}

}

uint8_t uart_rx_flush(uint8_t *buffer, uint8_t *rx_error)
{
	cli();
	uint8_t uart_available = 0;
	if(uart_rx_state == UART_RECEIVE)
	{
		for(uint8_t idx = 0; idx < uart_rx_index; idx++)
		{
			*(buffer + idx) = *(uart_rx_buffer + idx);

		}
		uart_available = uart_rx_index;
		uart_rx_index = 0;
		uart_rx_state = UART_IDLE;
	}
	else if(uart_rx_state == UART_RX_ERROR)
	{
		//reset the uart buffer
		uart_rx_index = 0;
		uart_rx_state = UART_IDLE;
		(*rx_error) = uart_rx_err_state;
		uart_rx_err_state = 0;
		uart_available = UART_RX_ERR;
	}
	sei();

	return uart_available;
}



uart_err uart_init(uint8_t baud, uint8_t cpu_freq, uint8_t uart_parity) // 1Mhz baud, 8 data, 1 stop, none parity
{
	//check if the input arguments are valid
	if( (baud >= UART_COUNT_BAUD) | (cpu_freq >= UART_COUNT_MHZ) )
		return UART_INVALID_ARG;

	//set the internal UART state
	uart_rx_index = 0;
	uart_tx_state = UART_IDLE;
	uart_rx_state = UART_IDLE;

	//set the HW registers
    UART_HW_SETUP_CODE();
	//enable interrupts for RX & TC operation complete
	UART_HW_RXCIE();
	UART_HW_TXCIE();
	UART_HW_ENABLE_2X();

	//set the baudrate according to the CPU speed
	if( uart_values[cpu_freq][baud] != -1)
		UART_HW_SET_LBAUD((uint8_t)uart_values[cpu_freq][baud]);
	else
		return UART_BAUD_NOT_SUPPORTED;

	//set the parity
	if(uart_parity == UART_PARITY_EVEN)
		UART_HW_SET_PARITY_EVEN();
	else if (uart_parity == UART_PARITY_ODD)
		UART_HW_SET_PARITY_ODD();

	
	return UART_CONFIG_OK;
}

void uart_sendByte(uint8_t byte)
{
	//set data and wait until it is done
	UART_HW_SET_DATA(byte);
	while( UART_BUSY_STATUS == UART_HW_IS_BUSY() ) {};
}



uint8_t uart_sendByteNotBlocking(uint8_t byte)
{
	uint8_t retValue = UART_NOK;
	if( uart_tx_state == UART_IDLE)
	{
		retValue = UART_OK;
		UART_HW_SET_DATA(byte);

	}
	return retValue;
}

void uart_printString(char *string, char crlf){
	char *idx = &string[0];

	while(idx != NULL && *idx != '\0'){
		uart_sendByte(*idx);
		idx++;
	}
	if(crlf) {
		uartNewLine();
	}
}

void uart_printRegister(unsigned char reg){
	//uart_sendByte('0');
	//uart_sendByte('x');
	unsigned char n1 = reg >> 4;
	unsigned char n2 = reg & 0x0F;
	uartPrintHex(n1);
	uartPrintHex(n2);
	//uart_sendByte(0x20);
	
}