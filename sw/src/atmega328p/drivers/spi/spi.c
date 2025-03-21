

#include "spi.h"
#define F_CPU 16384000UL
#include <util/delay.h>
#include <avr/interrupt.h>

#define SCK  PB5
#define MISO PB4
#define MOSI PB3
#define SS   PB2
#define SPI_DDR  DDRB
#define SPI_PORT PORTB


void spi_master_init(void)
{	

	SPCR = (1<<SPE)|(1<<MSTR);

    SPI_DDR |= (1 << MOSI) | (1 << SCK) | (1 << SS);
    SPI_DDR &= ~(1 << MISO);
    SPI_PORT |= (1 << MISO);

    SPI_PORT |= (1 << SS);
	

	SPCR |= (1 << SPR0);
    SPSR |= (1 << SPI2X);
    SPCR |= (1 << CPHA);
}


void spi_tx_8(uint8_t data)
{
    SPI_PORT &= ~(1 << SS); // Select slave (active low)
    _delay_us(1);
 
	SPDR = data;
    
	while(!(SPSR & (1<<SPIF)));

    _delay_us(1);
    SPI_PORT |= (1 << SS); // Deselect slave
 
}

uint8_t spi_rx_8(uint8_t data)
{
    SPI_PORT &= ~(1 << SS); // Select slave (active low)
    _delay_us(1);
	SPDR = data;
    
	while(!(SPSR & (1<<SPIF)));

    SPI_PORT |= (1 << SS); // Deselect slave
    _delay_us(1);
	return(SPDR);

}

void spi_tx_16(uint16_t data)
{
    uint8_t b1 = 0, b2 = 0;
    /* data format MSB B1 B2 LSB*/
    b1 = (uint8_t)((data & 0xFF00) >> 8);
    b2 = (uint8_t)(data & 0x00FF);
    SPI_PORT &= ~(1 << SS); // Select slave (active low)
    _delay_us(1);

    /*send first byte*/
	SPDR = b1;
	while(!(SPSR & (1<<SPIF)));

    /*second byte*/
	SPDR = b1;
	while(!(SPSR & (1<<SPIF)));

    _delay_us(1);
    SPI_PORT |= (1 << SS); // Deselect slave
 
}

uint16_t spi_rx_16(uint16_t data)
{
    uint8_t b1 = 0, b2 = 0;
    uint8_t out1 = 0, out2 = 0;
    /* data format MSB B1 B2 LSB*/
    b1 = (uint8_t)((data & 0xFF00) >> 8);
    b2 = (uint8_t)(data & 0x00FF);
    SPI_PORT &= ~(1 << SS); // Select slave (active low)
    _delay_us(1);

    /*send first byte*/
	SPDR = b1;
	while(!(SPSR & (1<<SPIF)));
    out1 = SPDR;

    /*second byte*/
	SPDR = b1;
	while(!(SPSR & (1<<SPIF)));
    out2 = SPDR;

    _delay_us(1);
    SPI_PORT |= (1 << SS); // Deselect slave
    
	return( ((uint16_t)out1 << 8) | (uint16_t)out2 );

}