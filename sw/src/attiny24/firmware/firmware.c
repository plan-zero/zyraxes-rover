/* Test application for the SPI-via-USI-driver. */

#include "spi_driver.h"
#include <avr/interrupt.h>


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
#define IN3 PA3
#define IN4 PA2

const unsigned char gpio_port_lut[] = 
{
	0x0, //0 0 0 0
	0x1, //0 0 0 1
	0x2, //0 0 1 0
	0x3, //0 0 1 1
	0x8, //0 1 0 0
	0x9, //0 1 0 1
	0xA, //0 1 1 0
	0xB, //0 1 1 1
	0x4, //1 0 0 0
	0x5, //1 0 0 1
	0x6, //1 0 1 0
	0x7, //1 0 1 1
	0xC, //1 1 0 0
	0xD, //1 1 0 1
	0xE, //1 1 1 0
	0xF, //1 1 1 1
};
unsigned char pwm_data[2] = {0};
unsigned char gpio_data = 0;

int main()
{
	unsigned char slave_state = STATE_SLAVE_INIT;
	unsigned char in_data = 0;
	unsigned char byte_count = 0;
	unsigned char update_regs = 0;

	spiX_initslave(SPIMODE);	// Init SPI driver as slave.
	sei();		// Must do this to make driver work.

	in_data = 0;
	byte_count = 0;
	pwm_data[0] = 0;
	pwm_data[1] = 0;
	gpio_data = 0;
	spiX_put(SLAVE_SYNC);
	slave_state = STATE_SLAVE_SEND_SYNC;

	//configure PWM
    TCCR0A |= _BV(COM0B1) | _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B |= _BV(CS00) | _BV(CS01);
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
				gpio_data = in_data & 0x0F;
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
			if(byte_count >= 1)
				slave_state = STATE_SLAVE_INIT;
		    update_regs = 1;
			pwm_data[byte_count] = in_data;
			byte_count += 1;
			spiX_put(SLAVE_SET);


		break;

		default:
		break;
		}

		//re-init slave in case of any errors
		if(STATE_SLAVE_INIT == slave_state)
		{
			in_data = 0;
			byte_count = 0;
			if(update_regs)
			{
				OCR0A = pwm_data[0];
				OCR0B = pwm_data[1];

				PORTA &= 0xF0;
				PORTA |= gpio_port_lut[gpio_data];
			}
			update_regs = 0;
			pwm_data[0] = 0;
			pwm_data[1] = 0;
			gpio_data = 0;
			spiX_put(SLAVE_SYNC);
			slave_state = STATE_SLAVE_SEND_SYNC;
		}

		while(!spiX_start_transfer())//wait transfer and do idle stuff
		{

		}
		
		spiX_wait();		// wait for transmission to finish
		in_data = spiX_get();	// and finally put result on PORTB.
		
	} while(1);			// Loop forever...
}
