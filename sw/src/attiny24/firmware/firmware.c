/* Test application for the SPI-via-USI-driver. */

#include "spi_driver.h"
#include "sin_lut_microstep_64.h"
#include <avr/interrupt.h>

#define F_CPU 8000000UL
#include <util/delay.h>


#define SPIMODE 0	// Sample on leading _rising_ edge, setup on trailing _falling_ edge.
//#define SPIMODE 1	// Sample on leading _falling_ edge, setup on trailing _rising_ edge.


#define SLAVE_SYNC 		0xAA
#define MASTER_SYNC		0x10
#define MASTER_DIAG		0x20
#define MASTER_DATA		0x30
#define SLAVE_ACK		0x81
#define SLAVE_SET       0x82
#define SLAVE_ERR		0x71

#define STATE_SLAVE_INIT  		0
#define STATE_SLAVE_SEND_SYNC 	1
#define STATE_SLAVE_GET_CMD 	2
#define STATE_SLAVE_GET_DATA    3
#define STATE_SLAVE_ERROR		4

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

union steps_data{
	uint32_t do_steps;
	uint8_t data[4]; 
};

int main()
{
	unsigned char slave_state = STATE_SLAVE_INIT;
	unsigned char in_data;
	unsigned char byte_count;
	unsigned char step_config = 0;
	union steps_data motor_steps;

	motor_steps.do_steps = 0;

	spiX_initslave(SPIMODE);	// Init SPI driver as slave.
	sei();		// Must do this to make driver work.

	byte_count = 0;
	in_data = 0;
	step_config = 0;
	spiX_put(SLAVE_SYNC);
	slave_state = STATE_SLAVE_SEND_SYNC;

	//configure PWM
    TCCR0A |= _BV(COM0B1) | _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B |= _BV(CS00);
	DDRB |= _BV(PB2);
	DDRA |= _BV(PA7);
    OCR0A = 0;
    OCR0B = 0;

	//Configure IN1-4 as output
	DDRA |= _BV(IN1) | _BV(IN2) | _BV(IN3) | _BV(IN4);

	do {
		
		switch(slave_state) {

		case STATE_SLAVE_SEND_SYNC:
			if(in_data == MASTER_SYNC)
			{
				spiX_put(SLAVE_SYNC);
				slave_state = STATE_SLAVE_GET_CMD;
			}
			else
			{
				slave_state = STATE_SLAVE_INIT;
			}
		break;
		case STATE_SLAVE_GET_CMD:
			//mask the low nibble, that will contain GPIO data
			if( (in_data & 0xF0) == MASTER_DATA)
			{
				//get ready to recieve data, send ACK byte
				step_config = in_data & 0x0F;
				spiX_put(SLAVE_ACK);
				slave_state = STATE_SLAVE_GET_DATA;

			}
			else if(in_data == MASTER_DIAG)
			{
				spiX_put(SLAVE_ACK);
				slave_state = STATE_SLAVE_INIT;
			}
			else
			{
				slave_state = STATE_SLAVE_INIT;
			}
		break;
		case STATE_SLAVE_GET_DATA:
			if(byte_count >= 3)
				slave_state = STATE_SLAVE_INIT;
	
			motor_steps.data[byte_count] = in_data;
			byte_count += 1;
			spiX_put(SLAVE_SET);


		break;

		default:
		break;
		}

		//re-init slave in case of any errors
		if(STATE_SLAVE_INIT == slave_state)
		{
			for(uint32_t i = 0; i < motor_steps.do_steps; i++)
			{
				if(step_config)
					step(-1);
				else
					step(1);
				_delay_us(10);
			}
			if(step_config & 0x2) //stop motor
				INPORT &= 0xF0;
			//reset vars
			in_data = 0;
			step_config = 0;
			spiX_put(SLAVE_SYNC);
			slave_state = STATE_SLAVE_SEND_SYNC;
			motor_steps.do_steps = 0;
			byte_count = 0;
			step_config = 0;
		}

		while(!spiX_start_transfer())//wait transfer and do idle stuff
		{

		}
		
		spiX_wait();		// wait for transmission to finish
		in_data = spiX_get();	// and finally put result on PORTB.
		
	} while(1);			// Loop forever...
}
