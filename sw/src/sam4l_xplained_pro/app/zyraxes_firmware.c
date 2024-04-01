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
#include "constants.h"
#include "nvm.h"
#include "string.h"




#define ioport_set_pin_peripheral_mode(pin, mode) \
	do {\
		ioport_set_pin_mode(pin, mode);\
		ioport_disable_pin(pin);\
	} while (0)

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/* Chip select. */
#define SPI_CHIP_SEL_0 0
#define SPI_CHIP_SEL_1 1
#define SPI_CHIP_SEL_2 2
#define SPI_CHIP_SEL_3 3

#define IN1 1
#define IN2 2
#define IN3 4
#define IN4 8

#define SPI_CHIP_PCS_1 0x0D//spi_get_pcs(SPI_CHIP_SEL_1)
#define SPI_CHIP_PCS_0 0x0E//spi_get_pcs(SPI_CHIP_SEL_0)

/* Clock polarity. */
#define SPI_CLK_POLARITY 0

/* Clock phase. */
#define SPI_CLK_PHASE 1

/* Delay before SPCK. */
#define SPI_DLYBS 200

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
static uint32_t gs_ul_spi_clock = 3500000;

static inline uint8_t spi_8bit_sync_transfer(uint8_t in_data, uint8_t cs, uint8_t last);
static void display_menu(void);
static void spi_master_initialize(void);
void readAttiny24Diagnostics(void);
static inline void setAttiny24Motor(uint8_t gpio, int steps);
void readEncoder(void);
void readEncoderDiagnostics(void);
void output(float theta, int effort);
int mod(int xMod, int mMod);
void oneStep(void);
static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to);

int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}


/**
 * \brief Display the user menu on the terminal.
 */
static void display_menu(void)
{

	puts("\n Plan-Zero Magnetic Sensor example\n\r");
	puts("\n\rMenu :\n\r"
			"------\r");

	puts("  m: Re-initialize SPI Master\n\r"
		 "  a: Attiny24 DIAG\n\r"
		 "  b: Attiny24 Motor STOP\n\r"
		 "  t: Attiny Motor test \n\r"
		 "  d: Magnetic Sensor DIAG\n\r"
		 "  f: Magnetic Sensor READ\n\r"
		 "  s: stepper one step forward \n\r"
		 "  l: Test calibration NVM data storage \n\r"
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

	spi_enable_peripheral_select_decode(SPI_MASTER_BASE);
	spi_set_variable_peripheral_select(SPI_MASTER_BASE);
	
	//configure CS0 AS5047
	spi_configure_cs_behavior(SPI_MASTER_BASE, SPI_CHIP_SEL_0, SPI_CS_RISE_NO_TX);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL_0, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL_0, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL_0,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL_0,
			(
#if (SAM4L)
			sysclk_get_pba_hz()
#else
			sysclk_get_peripheral_hz()
#endif
			/ gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL_0, SPI_DLYBS,
			SPI_DLYBCT);

	//configure CS1 attiny24
	spi_configure_cs_behavior(SPI_MASTER_BASE, SPI_CHIP_SEL_1, SPI_CS_RISE_NO_TX);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL_1, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL_1, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL_1,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL_1,
			(
#if (SAM4L)
			sysclk_get_pba_hz()
#else
			sysclk_get_peripheral_hz()
#endif
			/ gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL_1, SPI_DLYBS,
			SPI_DLYBCT);

	//CS2 devices
	spi_configure_cs_behavior(SPI_MASTER_BASE, SPI_CHIP_SEL_2, SPI_CS_RISE_NO_TX);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL_2, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL_2, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL_2,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL_2,
			(
#if (SAM4L)
			sysclk_get_pba_hz()
#else
			sysclk_get_peripheral_hz()
#endif
			/ gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL_2, SPI_DLYBS,
			SPI_DLYBCT);


	//CS3 devices
	spi_configure_cs_behavior(SPI_MASTER_BASE, SPI_CHIP_SEL_3, SPI_CS_RISE_NO_TX);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL_3, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL_3, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL_3,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL_3,
			(
#if (SAM4L)
			sysclk_get_pba_hz()
#else
			sysclk_get_peripheral_hz()
#endif
			/ gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL_3, SPI_DLYBS,
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

#define SPI_TIMEOUT_READ 100000 // timeout for SPI (TBD: exagerated, do some measurments and lower this)


static inline uint8_t spi_8bit_sync_transfer(uint8_t in_data, uint8_t cs, uint8_t last)
{
	uint32_t timeout = 0;
	uint16_t out_data = 0;
	uint8_t uc_pcs;

	spi_write(SPI_MASTER_BASE, in_data, cs, last);
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0 && timeout < SPI_TIMEOUT_READ)
	{
		delay_us(10);
		timeout++;
	}
	if(timeout >= SPI_TIMEOUT_READ)
	{
		printf("Timeout error: 0x%x", cs);
		return -1;
	}
	spi_read(SPI_MASTER_BASE, &out_data, &uc_pcs);
	return out_data;
}

void readAttiny24Diagnostics()
{

	uint32_t response = 0;
	uint8_t data = 0;

	//sync with attiny24
	data = spi_8bit_sync_transfer(0x10, SPI_CHIP_PCS_1, 1);
	response |= data;
	data = spi_8bit_sync_transfer(0x20, SPI_CHIP_PCS_1, 1);
	response |= (data << 8);

	printf("ATTINY response: 0x%lx\n\r", response);
}

int stepNumber = 0;

void oneStep() {           /////////////////////////////////   oneStep    ///////////////////////////////
  

  
	for(int i = 0; i < 200; i++)
		setAttiny24Motor(0x01, 64);
  	setAttiny24Motor(0x02, 1);
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}

void output(float theta, int effort) {
   int angle_1;
   int angle_2;
   int v_coil_A;
   int v_coil_B;

   int sin_coil_A;
   int sin_coil_B;
   int phase_multiplier = 10 * spr / 4;

  //REG_PORT_OUTCLR0 = PORT_PA09; for debugging/timing

  angle_1 = mod((phase_multiplier * theta) , 3600);   //
  angle_2 = mod((phase_multiplier * theta)+900 , 3600);
  
  sin_coil_A  = sin_1[angle_1];

  sin_coil_B = sin_1[angle_2];

  v_coil_A = ((effort * sin_coil_A) / 1024);
  v_coil_B = ((effort * sin_coil_B) / 1024);
  printf("Angle A %d, B %d, Sin A %d, B %d, uMAX %d ,Vref1 %d,Vref2 %d \n\r",angle_1, angle_2, sin_coil_A, sin_coil_B, uMAX, v_coil_A, v_coil_B);

  //analogFastWrite(VREF_1, abs(v_coil_A));
  //analogFastWrite(VREF_2, abs(v_coil_B));
  uint8_t gpio_data = 0;
  if (v_coil_A >= 0)  {
	gpio_data |= IN2;
	gpio_data &= ~IN1;
    //IN_2_HIGH();  //REG_PORT_OUTSET0 = PORT_PA21;     //write IN_2 HIGH
    //IN_1_LOW();   //REG_PORT_OUTCLR0 = PORT_PA06;     //write IN_1 LOW
  }
  else  {
	gpio_data &= ~IN2;
	gpio_data |=IN1;
    //IN_2_LOW();   //REG_PORT_OUTCLR0 = PORT_PA21;     //write IN_2 LOW
    //IN_1_HIGH();  //REG_PORT_OUTSET0 = PORT_PA06;     //write IN_1 HIGH
  }

  if (v_coil_B >= 0)  {
	gpio_data |= IN4;
	gpio_data &= ~IN3;
    //IN_4_HIGH();  //REG_PORT_OUTSET0 = PORT_PA20;     //write IN_4 HIGH
    //IN_3_LOW();   //REG_PORT_OUTCLR0 = PORT_PA15;     //write IN_3 LOW
  }
  else  {
	gpio_data &= ~IN4;
	gpio_data |= IN3;
    //IN_4_LOW();     //REG_PORT_OUTCLR0 = PORT_PA20;     //write IN_4 LOW
    //IN_3_HIGH();    //REG_PORT_OUTSET0 = PORT_PA15;     //write IN_3 HIGH
  }
  v_coil_A = abs(v_coil_A);
  v_coil_A = mapResolution(v_coil_A,8, 8);
  v_coil_B = abs(v_coil_B);
  v_coil_B = mapResolution(v_coil_B,8, 8);
  //printf("uMAX %d ,Vref1 %d,Vref2 %d \n\r",uMAX, v_coil_A, v_coil_B);
  //setAttiny24Motor( (uint8_t)v_coil_A, (uint8_t)v_coil_B, gpio_data);
}

static inline void setAttiny24Motor(uint8_t gpio, int steps)
{

	uint32_t response = 0;
	uint8_t data = 0;

	//sync with attiny24
	data = spi_8bit_sync_transfer(0x10, SPI_CHIP_PCS_1, 1);
	response |= (data << 8);
	gpio &= 0x0F;
	for(int i = 0; i < steps; i++) {
		data = spi_8bit_sync_transfer(0x30 | gpio, SPI_CHIP_PCS_1, 1);
		response |= data ;
	}
	data = spi_8bit_sync_transfer(0x10, SPI_CHIP_PCS_1, 1);

	//printf("ATTINY response 2: 0x%lx\n\r", response);
}

unsigned page_count = 0;
float page[CALIBRATION_DATA_SIZE];
int page_number = 0;
nvram_data_t * page_ptr;

static void store_lookup(float lookupAngle)
{
  page[page_count++] = lookupAngle;
  if(page_count != CALIBRATION_DATA_SIZE)
    return;

  // we've filled an entire page, write it to the flash
  flash_rw_calibration(page, page_ptr);

  // reset our counters and increment our flash page
  page_ptr = (nvram_data_t *)NVRAM_PAGE_ADDRESS(++page_number);
  page_count = 0;
  memset(page, 0, sizeof(page));
}


void readEncoder()
{
  long angleTemp;
  uint8_t b1 = 0, b2 = 0;

  b1 = spi_8bit_sync_transfer(0xFF, SPI_CHIP_PCS_0, 0);
  b2 = spi_8bit_sync_transfer(0xFF, SPI_CHIP_PCS_0, 1);

  angleTemp = (((b1 << 8) | b2) & 0B0011111111111111);

  printf("Magnetic RAW: %lu \n\r", angleTemp);
}

void readEncoderDiagnostics()
{
	long angleTemp = 0;
	uint16_t data = 0;
	//uint8_t uc_pcs;
	uint8_t b1 = 0, b2 = 0;
	
	

	///////////////////////////////////////////////READ DIAAGC (0x3FFC)
	puts("------------------------------------------------ \n\r");

	puts("Checking AS5047 diagnostic and error registers \n\r");
	puts("See AS5047 datasheet for details \n\r\r");


	spi_8bit_sync_transfer(0xFF, SPI_CHIP_PCS_0, 0);
	spi_8bit_sync_transfer(0xFC, SPI_CHIP_PCS_0, 1);

	delay_ms(1);

	b1 = spi_8bit_sync_transfer(0xC0, SPI_CHIP_PCS_0, 0);
	b2 = spi_8bit_sync_transfer(0x00, SPI_CHIP_PCS_0, 1);

	data = ((b1 << 8) | b2);

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



  	delay_ms(1);


	//4001
	spi_8bit_sync_transfer(0x40, SPI_CHIP_PCS_0, 0);
	spi_8bit_sync_transfer(0x01, SPI_CHIP_PCS_0, 1);


  	delay_ms(1);

	b1 = spi_8bit_sync_transfer(0xC0, SPI_CHIP_PCS_0 ,0);
	b2 = spi_8bit_sync_transfer(0x00, SPI_CHIP_PCS_0 ,1);
	data = ((b1 << 8) | b2);

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
	//uMAX = (255/3.3)*(iMAX*10*rSense);

	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Initialize the console UART. */
	configure_console();

	/* Output example information. */
	puts(STRING_HEADER);


	spi_master_initialize();

	//Init LED0
	ioport_set_pin_dir(LED_0_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_0_PIN, IOPORT_PIN_LEVEL_HIGH);
	//INIT CS for Encoder
	ioport_set_pin_dir(PIN_PB12,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_PB12, IOPORT_PIN_LEVEL_HIGH);

	ioport_set_pin_peripheral_mode(PIN_PC01A_SPI_NPCS3,
		MUX_PC01A_SPI_NPCS3);


	//init pointer to the flash first page
	page_ptr = (nvram_data_t *)NVRAM_PAGE_ADDRESS(++page_number);

	/* Display menu. */
	display_menu();

	uint8_t motor_test_gpio = 0;
	uint8_t motor_pwm_a = 127;
	uint8_t motor_pwm_b = 0;

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
		case 'a':
			readAttiny24Diagnostics();
			break;
		case 't':
			printf("Test motor: %d %d %d \n\r",motor_pwm_a, motor_pwm_b, motor_test_gpio);
			//setAttiny24Motor(motor_pwm_a, motor_pwm_b, motor_test_gpio);
			//motor_pwm_b = motor_pwm_a;
			//motor_pwm_a ^= 0xFF;
			motor_test_gpio++;
			if(motor_test_gpio == 0x10)
				motor_test_gpio = 0;
			break;
		case 'l':
			printf("Flash stats: start_addr: %x, flash_size: %d \n\r", FLASH_ADDR, FLASH_SIZE);
			printf("Add test angle to NVM, value count: %d, page no: %d, page_addr: %x \n\r",page_count, page_number , page_ptr);
			static float angleTest = 0.5;
			angleTest += 1.0;
			store_lookup(angleTest);
			break;
		case 'b':
			//setAttiny24Motor(0,0, 0);
			break;
		case 'f':
			readEncoder();
			break;
		case 's':
			oneStep();
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
