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


unsigned char master_data[2] = {0};

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
	master_data[0] = 0;
	master_data[1] = 0;
	spiX_put(SLAVE_SYNC);
	slave_state = STATE_SLAVE_SEND_SYNC;

	//configure PWM
    TCCR0A |= _BV(COM0B1) | _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B |= _BV(CS00) | _BV(CS01);
	DDRB |= _BV(PB2);
	DDRA |= _BV(PA7);
    OCR0A = 0;
    OCR0B = 0;

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
			if(in_data == MASTER_DATA)
			{
				//get ready to recieve data, send ACK byte
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
			master_data[byte_count] = in_data;
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
				OCR0A = master_data[0];
				OCR0B = master_data[1];
			}
			master_data[0] = 0;
			master_data[1] = 0;
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
