#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#include "twi.h"


void I2C_init(uint8_t address){
	// load address into TWI address register
	TWAR = address << 1;
	// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
	//enable pull-up resitors
	DDRC &= ~(1 << PINC4);
	DDRC &= ~(1 << PINC5);
	PORTC |= (1 << PINC4);
	PORTC |= (1 << PINC5);
}

void I2C_stop(void){
	// clear acknowledge and enable bits
	TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
}

ISR(TWI_vect){
	
	// temporary stores the received data
	uint8_t data;
	
	// own address has been acknowledged
	if( (TWSR & 0xF8) == TW_SR_SLA_ACK ){  
		buffer_address = 0xFF;
		// clear TWI interrupt flag, prepare to receive next byte and acknowledge
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
	}
	else if( (TWSR & 0xF8) == TW_SR_DATA_ACK ){ // data has been received in slave receiver mode
		
		// save the received byte inside data 
		data = TWDR;
		// check wether an address has already been transmitted or not
		if(buffer_address == 0xFF){
			
			buffer_address = data; 
			
			// clear TWI interrupt flag, prepare to receive next byte and acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
		}
		else{ // if a databyte has already been received
			twi_rx_status = TWI_SLAVE_RX_PENDING;
			// store the data at the current address
			rxbuffer[buffer_address] = data;
			
			// increment the buffer address
			buffer_address++;
			
			// if there is still enough space inside the buffer
			if(buffer_address < 0xFF){
				// clear TWI interrupt flag, prepare to receive next byte and acknowledge
				TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
			}
			else{
				// clear TWI interrupt flag, prepare to receive last byte and don't acknowledge
				TWCR |= (1<<TWIE) | (1<<TWINT) | (0<<TWEA) | (1<<TWEN); 
			}
		}
	}
	else if( ((TWSR & 0xF8) == TW_ST_DATA_ACK) || ((TWSR & 0xF8) == TW_ST_SLA_ACK) ){ // device has been addressed to be a transmitter
		twi_tx_status = TWI_SLAVE_TX_PENDING;
		// copy data from TWDR to the temporary memory
		data = TWDR;
		
		// if no buffer read address has been sent yet
		if( buffer_address == 0xFF ){
			buffer_address = data;
		}
		
		// copy the specified buffer address into the TWDR register for transmission
		TWDR = txbuffer[buffer_address];
		// increment buffer read address
		buffer_address++;
		
		// if there is another buffer address that can be sent
		if(buffer_address < 0xFF){
			// clear TWI interrupt flag, prepare to send next byte and receive acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
		}
		else{
			// clear TWI interrupt flag, prepare to send last byte and receive not acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (0<<TWEA) | (1<<TWEN); 
		}
		
	}
	else if((TWSR & 0xF8) == TW_ST_DATA_NACK)
	{
		twi_tx_status = TWI_SLAVE_TX_DONE;
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
	} 
	else if((TWSR & 0xF8) ==TW_SR_STOP)
	{
		if(twi_rx_status == TWI_SLAVE_RX_PENDING)
			twi_rx_status = TWI_SLAVE_RX_DONE;
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
	}
	else{
		// if none of the above apply prepare TWI to be addressed again
		//probably the transfer was not succesfull and app should discard the data
		twi_rx_status = TWI_SLAVE_TRANSFER_ERROR;
		twi_tx_status = TWI_SLAVE_TRANSFER_ERROR;
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
	} 
}
