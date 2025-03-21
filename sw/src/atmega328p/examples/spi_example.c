#include "spi.h"
#include "uart.h"
#include <avr/io.h>
#define F_CPU 16384000UL
#include <util/delay.h>
#include <avr/interrupt.h>


void magnetic_sensor_diag()
{

	long angleTemp = 0;
	uint16_t data = 0;

	uart_printString("motor_diag: Checking AS5047 diagnostic and error registers ... ", 1);

	spi_tx_16(0xFFFC);
	_delay_ms(1);
	data = spi_rx_16(0xC000);//((b1 << 8) | b2);

	uart_printString("motor_diag: Check DIAAGC register ... ", 1);

	angleTemp = (data & 0xFFFF);
	//uart_printString("%ld BIN n\r", (angleTemp | 0B1110000000000000000 ));

  	if (angleTemp & (1 << 14))    uart_printString("  Error occurred  ", 1);

  	if (angleTemp & (1 << 11))    uart_printString("  MAGH - magnetic field strength too high, set if AGC = 0x00. This indicates the non-linearity error may be increased", 1);

  	if (angleTemp & (1 << 10))    uart_printString("  MAGL - magnetic field strength too low, set if AGC = 0xFF. This indicates the output noise of the measured angle may be increased", 1);

  	if (angleTemp & (1 << 9))     uart_printString("  COF - CORDIC overflow. This indicates the measured angle is not reliable", 1);

  	if (angleTemp & (1 << 8))     uart_printString("  LF - offset compensation completed. At power-up, an internal offset compensation procedure is started, and this bit is set when the procedure is completed", 1);

  	if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 11)) | (angleTemp & (1 << 10)) | (angleTemp & (1 << 9))))  uart_printString("Looks good!", 1);
  
  	_delay_ms(1);
	spi_tx_16(0x4001);
  	_delay_ms(1);
	data = spi_rx_16(0xC000);//((b1 << 8) | b2);

    uart_printString("motor_diag: Check ERRFL register ...", 1);


  	angleTemp = (data & 0xFFFF);

  	if (angleTemp & (1 << 14)) {
    	uart_printString("  Error occurred ", 1);
  	}
  	if (angleTemp & (1 << 2)) {
    	uart_printString("  parity error ", 1);
  	}
  	if (angleTemp & (1 << 1)) {
    	uart_printString("  invalid register ", 1);
  	}
  	if (angleTemp & (1 << 0)) {
    	uart_printString("  framing error ", 1);
  	}
  	if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 2)) | (angleTemp & (1 << 1)) | (angleTemp & (1 << 0))))  uart_printString("Looks good!", 1);


    _delay_ms(1);


}

static inline int _motor_read_raw()
{
    uint16_t angleTemp;
    uint8_t b1 = 0, b2 = 0;

	//SPI_PORT &= ~(1 << SS); // Select slave (active low)
   // _delay_us(1);
   // b1 = spi_rx(0xFF);
   // b2 = spi_rx(0xFF);
	//_delay_us(1);
   // SPI_PORT |= (1 << SS); // Deselect slave
   	
    //angleTemp = (((b1 << 8) | b2) );
	angleTemp = spi_rx_16(0xFFFF);

    if(angleTemp & (1 << 14))
		uart_printString("Angle error! ", 1);

    angleTemp &= 0B0011111111111111;


    return angleTemp; //>> MAGNETIC_REDUCE_RESOLUTION;
}


int main()
{

    uart_init(UART_115200BAUD, UART_16384MHZ, UART_PARITY_NONE);
    _delay_ms(1000);
    uart_printString("SPI magnetic sensor atmega328p example",1);

    spi_master_init();
   

    magnetic_sensor_diag();

    uart_printString("Done, Ready!", 1);
	char str[5];
	uint16_t angle = 0;

    while(1)
    {
        _delay_ms(1000);
		angle = _motor_read_raw();
		itoa(angle,str,10);
		uart_printString(str,1);
    }
    return 0;
}