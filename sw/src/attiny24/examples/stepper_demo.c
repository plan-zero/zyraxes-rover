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
#include <math.h>


#include "sin_lut_microstep_16.h"

#define IN1 PA0
#define IN2 PA1
#define IN3 PA2
#define IN4 PA3

#define INPORT PORTA

#define EFFORT 40

// NEMA17 no gearbox
//measured 12.625 us 
//if this is 64 microstep it requires at least 22.635 us to have time to update ONE step
//ideally this should be 25us, hence ~1.5 ms for a full step (1489us measured,  64 microsteps)
// measured step: 1.5 ms
// full rotation: 320 ms
// Vin = 13.6
// rpm: 187.5
// measured speed with NO gearbox and wheel diameter 120: 1.175 m/s

// NEMA17 with gearbox 5 2/11
//measure step: 1.81 ms 
// full rotation: 384 ms
// full rotation with 5.18 ratio: 1980ms (1,98s)
//Vin = 13.6
// rpm: 30.3
// measured speed with gearbox and wheel diameter 120: 0.18 m/s

static inline void output()
{
  int8_t v_coil_A = 0, v_coil_B = 0, angle_A = 0, angle_B = 0;
  static uint8_t pos_a = 0, pos_b = POS_OFFSET;


  angle_A = (int8_t)pgm_read_byte(&sin_lut[pos_a % SIN_LUT_LEN]); //(int8_t)sin_lut[pos_b % SIN_LUT_LEN];
  pos_a++;
  angle_B = (int8_t)pgm_read_byte(&sin_lut[pos_b % SIN_LUT_LEN]);
  pos_b++;

  v_coil_A = ( angle_A* EFFORT )/128;
  v_coil_B = ( angle_B* EFFORT )/128;

  OCR0B = abs(v_coil_A);
  OCR0A = abs(v_coil_B);

  //INPORT &= 0xF0;
  if (v_coil_A >= 0)  {
    INPORT |= _BV(IN2);
    INPORT &= ~_BV(IN1);
  }
  else {
    INPORT &= ~_BV(IN2);
    INPORT |= _BV(IN1);
  }

  if (v_coil_B >= 0) {
    INPORT |= _BV(IN4);
    INPORT &= ~_BV(IN3);
  }
  else {
    INPORT &= ~_BV(IN4);
    INPORT |= _BV(IN3);
  }
}


int main(){

    DDRA |= _BV(IN1);
    DDRA |= _BV(IN2);
    DDRA |= _BV(IN3);
    DDRA |= _BV(IN4);

    DDRB |= 1 << PB0;
    PORTB |= 1 << PB0;


	//configure PWM
    TCCR0A |= _BV(COM0B1) | _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B |=  _BV(CS00);
	  DDRB |= _BV(PB2);
	  DDRA |= _BV(PA7);
    OCR0A = 0;
    OCR0B = 0;

    PORTB &= ~(1 << PB0);

    for(uint32_t i=0; i < 6400 * 2; i++)
    {
      output();
      _delay_us(75);
    }

    PORTB |= 1 << PB0;
    //stop motors
    //INPORT &= 0xF0;

    while(1)
    {


    }

    return 0;
}