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



#ifndef _INTERRUPT_HW_H
#define _INTERRUPT_HW_H

#include <avr/io.h>
#include <avr/interrupt.h>


//generat interrupt definitions
#define INTERRUPT_ROUTINE(x) ISR(x)

#define INT_ENABLE_BOOTLOADER_INTVECT() \
    MCUCR |= (1 << IVCE); \
	MCUCR = 0x2

#define INT_GLOBAL_EN()   sei()
#define INT_GLOBAL_DIS()  cli()

//EXT INT0 used for nrf24 RX/TX notification pin
#define IRQ_INT0_EN			    INT0
#define IRQ_INT0_EDGE		    ISC01 //rising edge
#define IRQ_INT0_HANDLER		INT0_vect

#define INT_ENABLE_INTERRUPT0_HW() \
    EIMSK |= _BV(IRQ_INT0_EN); \
	EICRA |= _BV(IRQ_INT0_EDGE)

//USART_TXC and USART_RXC used by uart module
#define IRQ_USART_TXC_HANDLER       USART_TX_vect
#define IRQ_USART_RXC_HANDLER       USART_RX_vect

//TIMERS

#define IRQ_TIMER0_OVF  TIMER0_OVF_vect
#define IRQ_TIMER1_OVF  TIMER1_OVF_vect
#define IRQ_TIMER2_OVF  TIMER2_OVF_vect

#define IRQ_TIMER0A_CMP     TIMER0_COMPA_vect
#define IRQ_TIMER0B_CMP     TIMER0_COMPB_vect
#define IRQ_TIMER1A_CMP     TIMER1_COMPA_vect
#define IRQ_TIMER1B_CMP     TIMER1_COMPB_vect
#define IRQ_TIMER2A_CMP     TIMER2_COMPA_vect
#define IRQ_TIMER2B_CMP     TIMER2_COMPB_vect

#define IRQ_TIMER1_CAPT     TIMER1_CAPT_vect

#if defined(IRQ_TIMER0A_CMP)
    #define EN_TIMER0_CMPA_IRQ 1
#endif






#endif