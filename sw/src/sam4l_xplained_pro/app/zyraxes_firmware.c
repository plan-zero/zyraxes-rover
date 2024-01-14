/**
 * \file
 *
 * \brief Serial Peripheral Interface (SPI) example for SAM.
 *
 * Copyright (c) 2011-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage SPI Example
 *
 * \par Purpose
 *
 * This example uses Serial Peripheral Interface (SPI) of one EK board in
 * slave mode to communicate with another EK board's SPI in master mode.
 *
 * \par Requirements
 *
 * This package can be used with two SAM evaluation kits boards.
 * Please connect the SPI pins from one board to another.
 * \copydoc spi_example_pin_defs
 *
 * \par Descriptions
 *
 * This example shows control of the SPI, and how to configure and
 * transfer data with SPI. By default, example runs in SPI slave mode,
 * waiting SPI slave & UART inputs.
 *
 * The code can be roughly broken down as follows:
 * <ul>
 * <li> 't' will start SPI transfer test.
 * <ol>
 * <li>Configure SPI as master, and set up SPI clock.
 * <li>Send 4-byte CMD_TEST to indicate the start of test.
 * <li>Send several 64-byte blocks, and after transmitting the next block, the
 * content of the last block is returned and checked.
 * <li>Send CMD_STATUS command and wait for the status reports from slave.
 * <li>Send CMD_END command to indicate the end of test.
 * </ol>
 * <li>Setup SPI clock for master.
 * </ul>
 *
 * \par Usage
 *
 * -# Compile the application.
 * -# Connect the UART port of the evaluation board to the computer and open
 * it in a terminal.
 *    - Settings: 115200 bauds, 8 bits, 1 stop bit, no parity, no flow control.
 * -# Download the program into the evaluation board and run it.
 * -# Upon startup, the application will output the following line on the
 *    terminal:
 *    \code
	-- Spi Example  --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
 * -# The following traces detail operations on the SPI example, displaying
 *    success or error messages depending on the results of the commands.
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_spi_example.h"
#include "delay.h"
#if (SAMG55)
#include "flexcom.h"
#include <ioport.h>
#endif

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/* Chip select. */
#define SPI_CHIP_SEL 0
#define SPI_CHIP_PCS spi_get_pcs(SPI_CHIP_SEL)

/* Clock polarity. */
#define SPI_CLK_POLARITY 0

/* Clock phase. */
#define SPI_CLK_PHASE 1

/* Delay before SPCK. */
#define SPI_DLYBS 0x80

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x40

#define CHIPSELECT_HIGH() ioport_set_pin_level(PIN_PB12, IOPORT_PIN_LEVEL_HIGH)
#define CHIPSELECT_LOW()  ioport_set_pin_level(PIN_PB12, IOPORT_PIN_LEVEL_LOW)


/* UART baudrate. */
#define UART_BAUDRATE      115200

#define STRING_EOL    "\r"
#define STRING_HEADER "--Spi Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


/* SPI clock setting (Hz). */
static uint32_t gs_ul_spi_clock = 2000000;

/**
 * \brief Display the user menu on the terminal.
 */
static void display_menu(void)
{

	puts("\n Plan-Zero Magnetic Sensor example\n\r");
	puts("\n\rMenu :\n\r"
			"------\r");

	puts("  m: Re-initialize SPI Master\n\r"
		 "  d: Magnetic Sensor DIAG\n\r"
		 "  h: Display this menu again\n\r\r");
}


/**
 * \brief Initialize SPI as master.
 */
static void spi_master_initialize(void)
{
	puts("-I- Initialize SPI as master\r");

#if (SAMG55)
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);
#else
	/* Configure an SPI peripheral. */
	spi_enable_clock(SPI_MASTER_BASE);
#endif
	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_PCS);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL,
			(
#if (SAM4L)
			sysclk_get_pba_hz()
#else
			sysclk_get_peripheral_hz()
#endif
			/ gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS,
			SPI_DLYBCT);
	spi_enable(SPI_MASTER_BASE);
}


/**
 * \brief Perform SPI master transfer.
 *
 * \param pbuf Pointer to buffer to transfer.
 * \param size Size of the buffer.
 
static void spi_master_transfer(void *p_buf, uint32_t size)
{
	uint32_t i;
	uint8_t uc_pcs;
	static uint16_t data;

	uint8_t *p_buffer;

	p_buffer = p_buf;

	for (i = 0; i < size; i++) {
		spi_write(SPI_MASTER_BASE, p_buffer[i], 0, 0);
		
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs);
		p_buffer[i] = data;
	}
}*/

void readEncoderDiagnostics()
{
	long angleTemp = 0;
	uint16_t data = 0;
	uint8_t uc_pcs;

	
	CHIPSELECT_LOW(); //digitalWrite(chipSelectPin, LOW);

	///////////////////////////////////////////////READ DIAAGC (0x3FFC)
	puts("------------------------------------------------ \n\r");

	puts("Checking AS5047 diagnostic and error registers \n\r");
	puts("See AS5047 datasheet for details \n\r\r");


	
	spi_write(SPI_MASTER_BASE, 0xFFFC, 0, 0); //0xFFFC
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);


	CHIPSELECT_HIGH();
	delay_ms(1);
	CHIPSELECT_LOW();

	// 0xC000
	spi_write(SPI_MASTER_BASE, 0xC000, 0, 0);
	/* Wait transfer done. */
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
	spi_read(SPI_MASTER_BASE, &data, &uc_pcs);


	puts("Check DIAAGC register (0x3FFC) ...  n\r\r");

	angleTemp = (data & 0xFFFF);
	printf("%ld BIN n\r", (angleTemp | 0B1110000000000000000 ));

  	if (angleTemp & (1 << 14))    puts("  Error occurred  \n\r");

  	if (angleTemp & (1 << 11))    puts("  MAGH - magnetic field strength too high, set if AGC = 0x00. This indicates the non-linearity error may be increased \n\r");

  	if (angleTemp & (1 << 10))    puts("  MAGL - magnetic field strength too low, set if AGC = 0xFF. This indicates the output noise of the measured angle may be increased\n\r");

  	if (angleTemp & (1 << 9))     puts("  COF - CORDIC overflow. This indicates the measured angle is not reliable\n\r");

  	if (angleTemp & (1 << 8))     puts("  LF - offset compensation completed. At power-up, an internal offset compensation procedure is started, and this bit is set when the procedure is completed\n\r");

  	if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 11)) | (angleTemp & (1 << 10)) | (angleTemp & (1 << 9))))  puts("Looks good!\n\r");
  	puts("\n\r");


  	CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  	delay_ms(1);
  	CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

	//4001
	spi_write(SPI_MASTER_BASE, 0x4001, 0, 0);
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);

  	CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  	delay_ms(1);
  	CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);
	//0xC000
	spi_write(SPI_MASTER_BASE, 0xC000, 0, 0);
	/* Wait transfer done. */
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
	spi_read(SPI_MASTER_BASE, &data, &uc_pcs);


  	puts("Check ERRFL register (0x0001) ...  \n\r\r");


  	angleTemp = (data & 0xFFFF);
  	printf("%ld BIN\n\r", (angleTemp | 0B1110000000000000000 ));

  	if (angleTemp & (1 << 14)) {
    	puts("  Error occurred  \n\r");
  	}
  	if (angleTemp & (1 << 2)) {
    	puts("  parity error \n\r");
  	}
  	if (angleTemp & (1 << 1)) {
    	puts("  invalid register  \n\r");
  	}
  	if (angleTemp & (1 << 0)) {
    	puts("  framing error  \n\r");
  	}
  	if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 2)) | (angleTemp & (1 << 1)) | (angleTemp & (1 << 0))))  puts("Looks good! \n\r");

  	puts("\n\r");

  	CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);


  	delay_ms(1);

}


/**
 *  \brief Configure the Console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


/**
 * \brief Application entry point for SPI example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t uc_key;

	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Initialize the console UART. */
	configure_console();

	/* Output example information. */
	puts(STRING_HEADER);

	/* Configure SPI interrupts for slave only. */
	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 0);
	NVIC_EnableIRQ(SPI_IRQn);

	spi_master_initialize();

	//Init LED0
	ioport_set_pin_dir(LED_0_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_0_PIN, IOPORT_PIN_LEVEL_HIGH);
	//INIT CS for Encoder
	ioport_set_pin_dir(PIN_PB12,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_PB12, IOPORT_PIN_LEVEL_HIGH);

	/* Display menu. */
	display_menu();

	while (1) {
		scanf("%c", (char *)&uc_key);

		switch (uc_key) {
		case 'h':
			display_menu();
			break;

		case 'm':
			spi_master_initialize();
			break;
		case 'd':
			readEncoderDiagnostics();
			break;

		default:
			break;
		}
	}
}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
