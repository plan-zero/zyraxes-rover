/**
 * Copyright (C) 2024 Coding Night Romania
 * 
 * This file is part of firmware_tiny24.
 * 
 * firmware_tiny24 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * firmware_tiny24 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with firmware_tiny24.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>

#define F_CPU 8000000UL
#include <util/delay.h>



#define IN1 PA0
#define IN2 PA1
#define IN3 PA3
#define IN4 PA2

#define INPORT PORTA
#define REF1_PORT PORTA
#define REF2_PORT PORTB

#define REF1 PA7
#define REF2 PB2

#define POS1 _BV(IN1) | _BV(IN4)
#define POS2 _BV(IN1)
#define POS3 _BV(IN1) | _BV(IN2)
#define POS4 _BV(IN2)
#define POS5 _BV(IN2) | _BV(IN3)
#define POS6 _BV(IN3)
#define POS7 _BV(IN3) | _BV(IN4)
#define POS8 _BV(IN4)

//Wiring
//in1 = PA0
//in2 = PA1
//in3 = PA3
//in4 = PA2

//VREF1 PA7
//VREF2 PB2


int main(){

    DDRA |= _BV(IN1);
    DDRA |= _BV(IN2);
    DDRA |= _BV(IN3);
    DDRA |= _BV(IN4);

    DDRA |= _BV(REF1);
    DDRB |= _BV(REF2);


  //  comment out if PWM is used
  //  TCCR0A = _BV(COM0B1) | _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
  //  TCCR0B = _BV(CS00);
  // OCR0A = 120;
  //  OCR0B = 120;

    REF1_PORT |= _BV(REF1);
    REF2_PORT |= _BV(REF2);



    for(int i=0; i < 200; i++)
    {

        INPORT &= ~0x0F;
        INPORT |= POS1;
        _delay_us(750);

        INPORT &= ~0x0F;
        INPORT |= POS2;
        _delay_us(750);

        INPORT &= ~0x0F;
        INPORT |= POS3;
        _delay_us(750);

        INPORT &= ~0x0F;
        INPORT |= POS4;
        _delay_us(750);

        INPORT &= ~0x0F;
        INPORT |= POS5;
        _delay_us(750);

        INPORT &= ~0x0F;
        INPORT |= POS6;
       _delay_us(750);

        INPORT &= ~0x0F;
        INPORT |= POS7;
        _delay_us(750);

        INPORT &= ~0x0F;
        INPORT |= POS8;
        _delay_us(750);    

     _delay_ms(25);   
    }


    while(1)
    {


    }

    return 0;
}