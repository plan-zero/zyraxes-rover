// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               spi_via_usi_driver.c
* \li Compiler:           IAR EWAAVR 3.10c
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All devices with Universal Serial Interface (USI)
*                         capabilities can be used.
*                         The example is written for ATmega169.
*
* \li AppNote:            AVR319 - Using the USI module for SPI communication.
*
* \li Description:        Example on how to use the USI module for communicating
*                         with SPI compatible devices. The functions and variables
*                         prefixed 'spiX_' can be renamed to be able to use several
*                         spi drivers (using different interfaces) with similar names.
*                         Some basic SPI knowledge is assumed.
*
*                         $Revision: 1.4 $
*                         $Date: Monday, September 13, 2004 12:08:54 UTC $
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "spi_driver.h"


/* USI port and pin definitions.
 */
#define USI_OUT_REG	PORTA	//!< USI port output register.
#define USI_IN_REG	PINA	//!< USI port input register.
#define USI_DIR_REG	DDRA	//!< USI port direction register.
#define USI_CLOCK_PIN	PA4	//!< USI clock I/O pin.
#define USI_DATAIN_PIN	PA6	//!< USI data input pin.
#define USI_DATAOUT_PIN	PA5	//!< USI data output pin.

#define USI_CS_DIR    DDRB
#define USI_CS_PORT   PORTB
#define USI_CS_IN_REG PINB
#ifdef CS_RESET
#define USI_CS_PIN  PB3
#define USI_PCINT PCINT11
#else
#define USI_CS_PIN  PB1
#define USI_PCINT PCINT9
#endif



/*  Speed configuration:
 *  Bits per second = CPUSPEED / PRESCALER / (COMPAREVALUE+1) / 2.
 *  Maximum = CPUSPEED / 64.
 */
#define TC0_PRESCALER_VALUE 1	//!< Must be 1, 8, 64, 256 or 1024.
#define TC0_COMPARE_VALUE   31	//!< Must be 0 to 255. Minimum 31 with prescaler CLK/1.




/*  Prescaler value converted to bit settings.
 */
#if TC0_PRESCALER_VALUE == 1
	#define TC0_PS_SETTING (1<<CS00)
#elif TC0_PRESCALER_VALUE == 8
	#define TC0_PS_SETTING (1<<CS01)
#elif TC0_PRESCALER_VALUE == 64
	#define TC0_PS_SETTING (1<<CS01)|(1<<CS00)
#elif TC0_PRESCALER_VALUE == 256
	#define TC0_PS_SETTING (1<<CS02)
#elif TC0_PRESCALER_VALUE == 1024
	#define TC0_PS_SETTING (1<<CS02)|(1<<CS00)
#else
	#error Invalid T/C0 prescaler setting.
#endif



/*! \brief  Data input register buffer.
 *
 *  Incoming bytes are stored in this byte until the next transfer is complete.
 *  This byte can be used the same way as the SPI data register in the native
 *  SPI module, which means that the byte must be read before the next transfer
 *  completes and overwrites the current value.
 */
volatile unsigned char spi_received_data[SPI_DATA_SIZE];


volatile unsigned char spi_ack_data[SPI_DATA_SIZE];



/*! \brief  Driver status bit structure.
 *
 *  This struct contains status flags for the driver.
 *  The flags have the same meaning as the corresponding status flags
 *  for the native SPI module. The flags should not be changed by the user.
 *  The driver takes care of updating the flags when required.
 */


volatile usidriverStatus_t spiX_status; //!< The driver status bits.
volatile unsigned char data_counter = 0;


ISR(PCINT1_vect)
{
	//read back the pin value to see if that is low
	if( (USI_CS_IN_REG & (1<<USI_CS_PIN)) == 0){
		spiX_status.cs_assert = 1;
		//enable output
		USI_DIR_REG |= (1<<USI_DATAOUT_PIN);
		//enable interrupt
		USICR |= (1<<USIOIE);
		//disable timer interrupt
		TIMSK1 &= ~(1 << OCIE1A);
		//reinit data counter to zero
		data_counter = 0;
	}
	else{
		spiX_status.cs_assert = 0;
		//disable output
		USI_DIR_REG &= ~(1<<USI_DATAOUT_PIN);
		//disable interrupt
		USICR &= ~(1<<USIOIE);
	}

}


/*! \brief  USI Timer Overflow Interrupt handler.
 *
 *  This handler disables the compare match interrupt if in master mode.
 *  When the USI counter overflows, a byte has been transferred, and we
 *  have to stop the timer tick.
 *  For all modes the USIDR contents are stored and flags are updated.
 */

ISR (USI_OVF_vect)
{

	// exchange data between slave-master
	spi_received_data[data_counter] = USIDR;
	USIDR = spi_ack_data[data_counter];
	data_counter++;

	if(data_counter == SPI_DATA_CHECKSUM_POS)
	{
		spiX_status.doChecksum = 1;
	}

	if(data_counter >= SPI_DATA_SIZE)
	{
		data_counter = 0;
		spiX_status.transferComplete = 1;
	}
	

	// Update flags and clear USI counter
	USISR = (1<<USIOIF);
}


/*! \brief  Initialize USI as SPI slave.
 *
 *  This function sets up all pin directions and module configurations.
 *  Use this function initially or when changing from master to slave mode.
 *  Note that the stored USIDR value is cleared.
 *
 *  \param spi_mode  Required SPI mode, must be 0 or 1.
 */
void spiX_initslave( char spi_mode )
{
	// Configure port directions.
	//keep output disabled to allow other slaves comunicate
	USI_DIR_REG &= ~(1<<USI_DATAOUT_PIN);                      // Outputs.
	USI_DIR_REG &= ~(1<<USI_DATAIN_PIN) | (1<<USI_CLOCK_PIN); // Inputs.
	USI_OUT_REG |= (1<<USI_DATAIN_PIN) | (1<<USI_CLOCK_PIN);  // Pull-ups.

	// Configure CS as slave
	USI_CS_DIR &= ~(1<<USI_CS_PIN);
	USI_CS_PORT |= (1<<USI_CS_PIN);
	GIMSK |= (1<<PCIE1);
	PCMSK1 |= (1<< USI_PCINT);
	
	
	// Configure USI to 3-wire slave mode with overflow interrupt.
	USICR = (1<<USIWM0) | (1<<USICS1) | (spi_mode<<USICS0);
	

	spiX_status.transferComplete = 0;
	spiX_status.writeCollision   = 0;
	spiX_status.cs_assert = 0;
	
	spi_received_data[0] = 0;
	spi_received_data[1] = 0;
	spi_received_data[2] = 0;
	spi_received_data[3] = 0;
	data_counter = 0;
}



/*! \brief  Put one byte on bus.
 *
 *  Use this function like you would write to the SPDR register in the native SPI module.
 *  Calling this function in master mode starts a transfer, while in slave mode, a
 *  byte will be prepared for the next transfer initiated by the master device.
 *  If a transfer is in progress, this function will set the write collision flag
 *  and return without altering the data registers.
 *
 *  \returns  0 if a write collision occurred, 1 otherwise.
 */
char spiX_put( unsigned char val )
{
	// Check if transmission in progress,
	// i.e. USI counter unequal to zero.
	if( (USISR & 0x0F) != 0 ) {
		// Indicate write collision and return.
		spiX_status.writeCollision = 1;
		return;
	}
	
	// Reinit flags.
	spiX_status.transferComplete = 0;
	spiX_status.writeCollision = 0;

	// Put data in USI data register.
	spi_ack_data[0] = SLAVE_ACK;
	spi_ack_data[1] = 0;
	spi_ack_data[2] = 0;
	spi_ack_data[3] = val;
	
	return 0;
}


// end of file
