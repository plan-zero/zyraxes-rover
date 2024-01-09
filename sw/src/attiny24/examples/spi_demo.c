/* Test application for the SPI-via-USI-driver. */

#include "spi_driver.h"
#include <avr/interrupt.h>


#define SPIMODE 0	// Sample on leading _rising_ edge, setup on trailing _falling_ edge.
//#define SPIMODE 1	// Sample on leading _falling_ edge, setup on trailing _rising_ edge.



void main()
{
	unsigned char val = 0x01;		// Temp value to send.
	unsigned char in_data = 0;
//	spiX_initmaster(SPIMODE);	// Init SPI driver as master.
	spiX_initslave(SPIMODE);	// Init SPI driver as slave.
	sei();		// Must do this to make driver work.
	do {
		spiX_put( val );	// Send temp value to SPI and increment,
		spiX_wait();		// wait for transmission to finish
		in_data = spiX_get();	// and finally put result on PORTB.

		val++;
	} while(1);			// Loop forever...
}
