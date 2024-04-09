/* Test application for the SPI-via-USI-driver. */

#include "spi_driver.h"
#include "sin_lut_microstep_16.h"
#include <avr/interrupt.h>

#define F_CPU 8000000UL
#include <util/delay.h>


#define SPIMODE 0	// Sample on leading _rising_ edge, setup on trailing _falling_ edge.
//#define SPIMODE 1	// Sample on leading _falling_ edge, setup on trailing _rising_ edge.


#define SLAVE_SYNC 		0xAA
#define SLAVE_ERROR		0x55
#define SLAVE_STEP		0x77
#define SLAVE_RESERVED	0x00

#define MASTER_MASK			0xC0
#define MASTER_SYNC			0xC0
#define MASTER_DOSTEP_REV	0x40
#define MASTER_DOSTEP		0x80


#define IN1	PA0
#define IN2 PA1
#define IN3 PA2
#define IN4 PA3

#define INPORT PORTA

#define EFFORT 37



static inline void step(int8_t dir)
{
  int16_t v_coil_A = 0, v_coil_B = 0, angle_A = 0, angle_B = 0;
  static uint16_t pos_a = 0, pos_b = POS_OFFSET;


  angle_A = (int8_t)pgm_read_byte(&sin_lut[pos_a % SIN_LUT_LEN]);
  pos_a += dir;
  angle_B = (int8_t)pgm_read_byte(&sin_lut[pos_b % SIN_LUT_LEN]);
  pos_b += dir;

  v_coil_A = ( angle_A* EFFORT )/128;
  v_coil_B = ( angle_B* EFFORT )/128;

  OCR0B = abs(v_coil_A);
  OCR0A = abs(v_coil_B);

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


int main()
{

	uint8_t in_data = 0;
	uint8_t new_data = 0;
	spiX_initslave(SPIMODE);	// Init SPI driver as slave.
	sei();		// Must do this to make driver work.

	spiX_put(SLAVE_SYNC);

	//configure PWM
    TCCR0A |= _BV(COM0B1) | _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B |= _BV(CS00);
	DDRB |= _BV(PB2);
	DDRA |= _BV(PA7);
    OCR0A = 0;
    OCR0B = 0;

	//Configure IN1-4 as output
	DDRA |= _BV(IN1) | _BV(IN2) | _BV(IN3) | _BV(IN4);
	DDRB |= 1 << PB0;
	PORTB |= 1 << PB0;
	
	do {

		if( (in_data & MASTER_MASK) == MASTER_DOSTEP)
		{
			step(1);
			spiX_put(SLAVE_STEP);
			in_data = 0;
			PORTB |= 1 << PB0;
		}
		else if( (in_data & MASTER_MASK) == MASTER_DOSTEP_REV)
		{
			step(-1);
			spiX_put(SLAVE_STEP);
			in_data = 0;
			PORTB |= 1 << PB0;
		}
		else if( (in_data & MASTER_MASK) == MASTER_SYNC)
		{
			//TODO: send some status here
			spiX_put(SLAVE_SYNC);
			in_data = 0;
			PORTB |= 1 << PB0;
		}



		if(spiX_start_transfer())
		{
			spiX_wait();		// wait for transmission to finish
			in_data = spiX_get();	// and finally put result on PORTB.
			PORTB &= ~(1 << PB0);
		}
		
		
		
		
	} while(1);			// Loop forever...
}
