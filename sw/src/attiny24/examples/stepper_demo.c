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

#include <avr/pgmspace.h>


#define IN1 PA0
#define IN2 PA1
#define IN3 PA3
#define IN4 PA2

#define INPORT PORTA
#define REF1_PORT PORTA
#define REF2_PORT PORTB

#define REF1 PA7
#define REF2 PB2



// v_coil_A = (effort * sin(angle)) / 1024
// v_coil_B = (effort * sin(angle + 90deg)) / 1024

//theta 1.8 (step): 0, 90, 180, 270

//value calculated for 33% effort from MAXVref 
//
//#define EFFORT 115

#define MAX_SIN 128
//const int16_t vref_lut[MAX_SIN] = {
//	0,   //0, 0,
//	37,  //1024, 90
//	0,   //0, 180
//	-37  //-1024, 270
//};

const int8_t vref_lut[MAX_SIN] PROGMEM  = {
 1,  3,  5,  7, 
 8,  10, 12, 14,
 15, 17, 19, 20,
 22, 23, 24, 26,
 27, 28, 29, 30,
 31, 32, 33, 34,
 34, 35, 35, 36,
 36, 36, 36, 37,
 36, 36, 36, 36,
 35, 35, 34, 34,
 33, 32, 31, 30,
 29, 28, 27, 26,
 24, 23, 22, 20,
 19, 17, 15, 14,
 12, 10, 9,  7, 
 5,  3,  1,  0, 
-1, -3, -5, -7, 
-8, -10,-12,-14,
-15,-17,-19,-20,
-22,-23,-24,-26,
-27,-28,-29,-30,
-31,-32,-33,-34,
-34,-35,-35,-36,
-36,-36,-36,-37,
-36,-36,-36,-36,
-35,-35,-34,-34,
-33,-32,-31,-30,
-29,-28,-27,-26,
-24,-23,-22,-20,
-19,-17,-15,-14,
-12,-10,-9, -7, 
-5, -3, -1
};


#define get_idx(x) (x % MAX_SIN)


static inline uint8_t _abs(int8_t val)
{
  if(val < 0)
    return (uint8_t)(val * (int8_t)(-1));

  return (uint8_t)val;
}



void output()
{
  int16_t v_coil_A = 0, v_coil_B = 0;
  static uint8_t pos_a = 0, pos_b = 32;

  uint8_t angle = 0;

  v_coil_A = (int8_t)pgm_read_byte(&vref_lut[pos_a % MAX_SIN]);
  pos_a++;
  v_coil_B = (int8_t)pgm_read_byte(&vref_lut[pos_b % MAX_SIN]);
  pos_b++;

  OCR0B = _abs(v_coil_A);
  OCR0A = _abs(v_coil_B);

  INPORT &= 0xF0;
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


	//configure PWM
    TCCR0A |= _BV(COM0B1) | _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B |= _BV(CS01) | _BV(CS00);
	  DDRB |= _BV(PB2);
	  DDRA |= _BV(PA7);
    OCR0A = 0;
    OCR0B = 0;


    for(int i=0; i < 6400; i++)
    {
      output();
      _delay_us(100);
    }

    //stop motors
    INPORT &= 0xF0;

    while(1)
    {


    }

    return 0;
}