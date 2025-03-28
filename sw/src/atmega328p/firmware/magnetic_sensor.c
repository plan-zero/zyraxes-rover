#include "spi.h"
#include "magnetic_sensor.h"
#include "firmware_hw.h"
#include <util/delay.h>
#include "stdint.h"

uint16_t magnetic_sensor_read()
{
    uint16_t angleTemp;

	angleTemp = spi_rx_16(0xFFFF);

    if(angleTemp & (1 << 14))
		uart_printString("Angle error! ", 1);

    angleTemp &= 0B0011111111111111;


    return angleTemp; //>> MAGNETIC_REDUCE_RESOLUTION;
}

uint32_t magnetic_sensor_diag()
{

	long angleTemp = 0;
	uint16_t data = 0;
	uint32_t ret;

	uart_printString("motor_diag: Checking AS5047 diagnostic and error registers ... ", 1);

	spi_tx_16(0xFFFC);
	_delay_ms(1);
	data = spi_rx_16(0xC000);
	ret = (uint32_t)data & 0x0000FFFF;

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
	ret |= (uint32_t)data << 16;

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

	return ret;
}