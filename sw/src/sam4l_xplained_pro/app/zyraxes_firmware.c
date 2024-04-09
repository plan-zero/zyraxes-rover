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
#include <tc.h>
#include "math.h"



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

#define SPI_CHIP_PCS_5 0x4
#define SPI_CHIP_PCS_4 0x5

#define SPI_CHIP_PCS_3 0x2
#define SPI_CHIP_PCS_2 0x3

#define SPI_CHIP_PCS_1 0x0//0x0D//spi_get_pcs(SPI_CHIP_SEL_1)
#define SPI_CHIP_PCS_0  0x01//0x0E//spi_get_pcs(SPI_CHIP_SEL_0)

/* Clock polarity. */
#define SPI_CLK_POLARITY 0

/* Clock phase. */
#define SPI_CLK_PHASE 1

/* Delay before SPCK. */
#define SPI_DLYBS 200

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 64

#define CHIPSELECT_HIGH() ioport_set_pin_level(PIN_PB12, IOPORT_PIN_LEVEL_HIGH)
#define CHIPSELECT_LOW()  ioport_set_pin_level(PIN_PB12, IOPORT_PIN_LEVEL_LOW)


/* UART baudrate. */
#define UART_BAUDRATE      115200

#define STRING_EOL    "\r"
#define STRING_HEADER "--Spi Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


/* SPI clock setting (Hz). */
static uint32_t gs_ul_spi_clock = 3000000;

#define DEFAULT_CPR 16384
#define DEFAULT_CPR_THRESHOLD 15000
//used to reduce the resolution of the magnetic data as calibration data is huge
//anyway for our aplication it's enough to use only 11 bits, increase below number 
//to reduce more bits
#define MAGNETIC_REDUCE_RESOLUTION 2
#define CPR (DEFAULT_CPR >> MAGNETIC_REDUCE_RESOLUTION)
#define CPR_THRESHOLD (DEFAULT_CPR_THRESHOLD >> MAGNETIC_REDUCE_RESOLUTION)
#define MAGNETIC_LUT_SIZE CPR


const float __attribute__((__aligned__(512))) lookup[MAGNETIC_LUT_SIZE] = {
//Put lookup table here!
};

static inline uint8_t spi_8bit_sync_transfer(uint8_t in_data, uint8_t cs, uint8_t last);
static void display_menu(void);
static void spi_master_initialize(void);
void readAttiny24Diagnostics(uint8_t PCs);
static inline void setAttiny24Motor(uint8_t steps_config, uint32_t steps, uint8_t sync, uint8_t PCs);
int readEncoder(void);
void readEncoderDiagnostics(void);
int mod(int xMod, int mMod);
void oneStep(uint8_t PCs);
static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to);
void calibrate(uint8_t PCs);
float read_angle();
void pid_init_data();

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
		 "  t: Attiny Motor1 test \n\r"
		 "  t: Attiny Motor2 test \n\r"
		 "  d: Magnetic Sensor DIAG\n\r"
		 "  f: Magnetic Sensor READ\n\r"
		 "  s: stepper one step forward \n\r"
		 "  l: Test calibration NVM data storage \n\r"
		 "  c: Calibrate routine \n\r"
		 "  p: Print Calibration data \n\r"
		 "  w: Toggle close loop control \n\r"
		 "  h: Display this menu again\n\r\r");
}

//interrupt vars

volatile int U = 0;       //control effort (abs)
volatile float r = 0.0;   //setpoint
volatile float y = 0.0;   // measured angle
volatile float v = 0.0;  // estimated velocity  (velocity loop)
volatile float yw = 0.0;  // "wrapped" angle (not limited to 0-360)
volatile float yw_1 = 0.0;
volatile float e = 0.0;   // e = r-y (error)
volatile float p = 0.0;   // proportional effort
volatile float i = 0.0;   // integral effort


volatile float u = 0.0;     //real control effort (not abs)
volatile float u_1 = 0.0;   //value of u at previous time step, etc...
volatile float e_1 = 0.0;   //these past values can be useful for more complex controllers/filters     
volatile float u_2 = 0.0;
volatile float e_2 = 0.0;
volatile float u_3 = 0.0;
volatile float e_3 = 0.0;
volatile long counter = 0;

volatile long wrap_count = 0;  //keeps track of how many revolutions the motor has gone though (so you can command angles outside of 0-360)
volatile float y_1 = 0;

  
volatile long step_count = 0;  //For step/dir interrupt (closed loop)
int stepNumber = 0; // open loop step number (used by 's' and for cal routine)

volatile float ITerm;
volatile float DTerm;

//----Current Parameters-----

volatile float Fs = 1000.0;   //Sample frequency in Hz

volatile float pKp = 15.0;      //position mode PID values.  Depending on your motor/load/desired performance, you will need to tune these values.  You can also implement your own control scheme
volatile float pKi = 0.2;
volatile float pKd = 250.0;//1000.0;
volatile float pLPF = 30;       //break frequency in hertz

volatile float vKp = 0.001;       //velocity mode PID values.  Depending on your motor/load/desired performance, you will need to tune these values.  You can also implement your own control scheme
volatile float vKi = 0.001;
volatile float vKd = 0.0;
volatile float vLPF = 100.0;       //break frequency in hertz

volatile float pLPFa;
volatile float pLPFb;
volatile float vLPFa;
volatile float vLPFb;

volatile float PA;
volatile int angle_to_steps = 0;
volatile int steps_dir = 0;
volatile int missed_steps = 0;

char mode = 'x';

void pid_init_data() {
	pLPFa = exp(pLPF*-2*3.14159/Fs); // z = e^st pole mapping
	pLPFb = (1.0-pLPFa);
	vLPFa = exp(vLPF*-2*3.14159/Fs); // z = e^st pole mapping
	vLPFb = (1.0-vLPFa)* Fs * 0.16666667;
	PA = 1.8;
}


void TC00_Handler(void)
{
	volatile uint32_t ul_dummy;





  static int print_counter = 0;               //this is used by step response

     

    y = lookup[readEncoder()];                    //read encoder and lookup corrected angle in calibration lookup table
   
    if ((y - y_1) < -180.0) wrap_count += 1;      //Check if we've rotated more than a full revolution (have we "wrapped" around from 359 degrees to 0 or ffrom 0 to 359?)
    else if ((y - y_1) > 180.0) wrap_count -= 1;

    yw = (y + (360.0 * wrap_count));              //yw is the wrapped angle (can exceed one revolution)

	if (yw < r - 1.8) {
		missed_steps -= 1;
	}
	else if (yw > r + 1.8) {
		missed_steps += 1;
	}
    
	//output(0.1125 * (-(r - missed_steps)), (255 / 3.3) * (iLevel * 10 * rSense));
	angle_to_steps = (int)(r/0.028125) - (missed_steps * 64);
	if(angle_to_steps < 0)
		steps_dir = 1;
	else 
		steps_dir = 0;
	angle_to_steps = abs(angle_to_steps);
	setAttiny24Motor(steps_dir, angle_to_steps, 0, SPI_CHIP_PCS_1);

	/*
	//switch (mode) {
	//case 'x':         // position control                        
		e = (r - yw);
		
		ITerm += (pKi * e);                             //Integral wind up limit
		if (ITerm > 150.0) ITerm = 150.0;
		else if (ITerm < -150.0) ITerm = -150.0;          
		
		DTerm = pLPFa*DTerm -  pLPFb*pKd*(yw-yw_1);
		
		u = (pKp * e) + ITerm + DTerm;
		
		
	//	break;
	
	case 'v':         // velocity controlr
		v = vLPFa*v +  vLPFb*(yw-yw_1);     //filtered velocity called "DTerm" because it is similar to derivative action in position loop

		e = (r - v);   //error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )

		ITerm += (vKi * e);                 //Integral wind up limit
		if (ITerm > 200) ITerm = 200;
		else if (ITerm < -200) ITerm = -200;
	
		u = ((vKp * e) + ITerm - (vKd * (e-e_1)));
		
		//SerialUSB.println(e);
		break;
		
	case 't':         // torque control
		u = 1.0 * r ;
		break;
	default:
		u = 0;
		break;
	}

	y_1 = y;  //copy current value of y to previous value (y_1) for next control cycle before PA angle added


	if (u > 0)          //Depending on direction we want to apply torque, add or subtract a phase angle of PA for max effective torque.  PA should be equal to one full step angle: if the excitation angle is the same as the current position, we would not move!  
	{                 //You can experiment with "Phase Advance" by increasing PA when operating at high speeds
		y += PA;          //update phase excitation angle
		if (u > uMAX)     // limit control effort
			u = uMAX;       //saturation limits max current command
	}
	else
	{
		y -= PA;          //update phase excitation angle
		if (u < -uMAX)    // limit control effort
			u = -uMAX;      //saturation limits max current command
	}

	U = abs(u);       //
	*/

	//output(-y, round(U));    // update phase currents
	//output(-y);
	//convert y to steps knowing the driver is using 64 microsteps
	//angle_to_steps = (int)( y / 0.028125);
	//if(angle_to_steps < 0)
	//	steps_dir = 0;
	//else
	//	steps_dir = 1;
	//angle_to_steps = abs(angle_to_steps);
	//setAttiny24Motor(steps_dir, angle_to_steps, 0);
	

	//static int count = 0;
	//count++;
	//if( (count % 6500) == 0){
	//	printf("steps = %d, dir = %d, missed steps: %d\n\r", angle_to_steps, steps_dir, missed_steps);
	//	char str[8];
	//	snprintf(str, sizeof(str), "%f", y);
	//	printf("Y= %s \n\r",str);
	//	snprintf(str, sizeof(str), "%f", y_1);
	//	printf("Y_1= %s \n\r",str);
	//	snprintf(str, sizeof(str), "%f", yw_1);
	//	printf("Yw_1= %s \n\r",str);
	//}
    
    
	// e_3 = e_2;    //copy current values to previous values for next control cycle
	e_2 = e_1;    //these past values can be useful for more complex controllers/filters.  Uncomment as necessary    
	e_1 = e;
	// u_3 = u_2;
	u_2 = u_1;
	u_1 = u;
	yw_1 = yw;
	y_1 = y;


	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

    

}

/**
 *  Configure Timer Counter 0 to generate an interrupt every 200ms.
 */
static void configure_tc(void)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	uint32_t handler_freq_hz = 50;

	/* Configure TC0 */
	sysclk_enable_peripheral_clock(TC0);

	/* Configure TC for a 5Hz frequency and trigger on RC compare. */
	if (!tc_find_mck_divisor(handler_freq_hz, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk)) {
		puts("No valid divisor found!\r");
		return;
	}
	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / handler_freq_hz);

	/* Configure and enable interrupt on RC compare */
	NVIC_EnableIRQ((IRQn_Type) TC00_IRQn);
	//tc_enable_interrupt(TC0, 0, TC_IER_CPCS);

	/* Start the counter. */
	tc_start(TC0, 0);
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

void readAttiny24Diagnostics(uint8_t PCs)
{

	uint32_t response = 0;
	uint8_t data = 0;

	//sync with attiny24
	data = spi_8bit_sync_transfer(0x10, PCs, 1);
	response = data;

	printf("ATTINY response: 0x%lx\n\r", response);
}

int dir = 0;

void oneStep(uint8_t PCs) {           /////////////////////////////////   oneStep    ///////////////////////////////
	
	if (!dir) {
		stepNumber += 1;
	}
	else {
		stepNumber -= 1;
	}
	setAttiny24Motor(0x01, 64, 0, PCs);
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

static inline void setAttiny24Motor(uint8_t steps_config, uint32_t steps, uint8_t sync, uint8_t PCs)
{

	uint32_t response = 0;
	uint8_t data = 0;
	uint8_t * u8 = (uint8_t *)&steps;

	//sync with attiny24
	data = spi_8bit_sync_transfer(0x10, PCs, 1);
	response |= data;

	if(sync)
	{
		while(response != 0xaa00)
		{
			delay_us(600);
			response = 0;
			data = spi_8bit_sync_transfer(0x10, PCs, 1);
			response |= data;
		}
	}

	
	
	steps_config &= 0x0F;
	data = spi_8bit_sync_transfer(0x30 | steps_config , PCs, 1);
	response |= (data << 8);

	
	printf("Response: 0x%lx\n\r", response);

	uint32_t u32res = 0;
	for(int i = 0; i < 4; i++) {
		data = spi_8bit_sync_transfer(*(u8+i), PCs, 1);
		u32res = data << (8*i);
	}

	printf("ATTINY response 2: 0x%lx\n\r", u32res);
}

unsigned page_count = 0;
float page[CALIBRATION_DATA_SIZE];
int page_number = 0;
static const void * page_ptr;

static void store_lookup(float lookupAngle)
{
  page[page_count++] = lookupAngle;
  if(page_count != CALIBRATION_DATA_SIZE)
    return;

  // we've filled an entire page, write it to the flash
  printf("Add to NVM, page no: %d, page_addr: %x \n\r",page_number , page_ptr);
  //flash_rw_calibration(page, page_ptr);
  flashcalw_memcpy(page_ptr, page, FLASH_PAGE_SIZE, true);

  // reset our counters and increment our flash page
  page_number++;
  page_ptr += FLASH_PAGE_SIZE;
  page_count = 0;
  memset(page, 0, sizeof(page));
  
}


int readEncoder()
{
  long angleTemp;
  uint8_t b1 = 0, b2 = 0;

  b1 = spi_8bit_sync_transfer(0xFF, SPI_CHIP_PCS_0, 0);
  b2 = spi_8bit_sync_transfer(0xFF, SPI_CHIP_PCS_0, 1);

  angleTemp = (((b1 << 8) | b2) & 0B0011111111111111);

  //printf("Magnetic RAW: %lu \n\r", angleTemp);
  //reduce rezolution
  return angleTemp >> MAGNETIC_REDUCE_RESOLUTION;
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

float read_angle()
{
  const int avg = 10;            //average a few readings
  int encoderReading = 0;

  //disableTCInterrupts();        //can't use readEncoder while in closed loop

  for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
    encoderReading += readEncoder();
    delay_ms(10);
    }

  //return encoderReading * (360.0 / 16384.0) / avg;
  return lookup[encoderReading / avg];
}

void calibrate(uint8_t PCs) {   /// this is the calibration routine

  static int cpr = CPR;
  static float aps = 1.8;
  int encoderReading = 0;     //or float?  not sure if we can average for more res?
  int currentencoderReading = 0;
  int lastencoderReading = 0;
  int avg = 10;               //how many readings to average

  int iStart = 0;     //encoder zero position index
  int jStart = 0;
  int stepNo = 0;
  
  int fullStepReadings[spr];
    
  int fullStep = 0;
  int ticks = 0;
  float lookupAngle = 0.0;
  puts("Beginning calibration routine... \n\r");

  encoderReading = readEncoder();
  dir = 1;
  oneStep(PCs);
  delay_ms(500);

  if ((readEncoder() - encoderReading) < 0)   //check which way motor moves when dir = true
  {
    puts("Wired backwards \n\r");    // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...
    return;
  }

  while (stepNumber != 0) {       //go to step zero
    if (stepNumber > 0) {
      dir = 1;
    }
    else
    {
      dir = 0;
    }
    oneStep(PCs);
    delay_ms(100);
  }
  dir = 1;
  for (int x = 0; x < spr; x++) {     //step through all full step positions, recording their encoder readings

    encoderReading = 0;
    delay_ms(20);                         //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
    lastencoderReading = readEncoder();
        
    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
      currentencoderReading = readEncoder();

      if ((currentencoderReading-lastencoderReading)<(-(cpr/2))){
        currentencoderReading += cpr;
      }
      else if ((currentencoderReading-lastencoderReading)>((cpr/2))){
        currentencoderReading -= cpr;
      }
 
      encoderReading += currentencoderReading;
      delay_ms(10);
      lastencoderReading = currentencoderReading;
    }
    encoderReading = encoderReading / avg;
    if (encoderReading>cpr){
      encoderReading-= cpr;
    }
    else if (encoderReading<0){
      encoderReading+= cpr;
    }

    fullStepReadings[x] = encoderReading;
   // SerialUSB.println(fullStepReadings[x], DEC);      //print readings as a sanity check
    if (x % 20 == 0)
    {
      printf("\n\r DBG: %d \n\r", 100*x/spr);
      
    } else {
      printf(".");
    }
    
    oneStep(PCs);
  	}
      puts("\n\r");

 
  for (int i = 0; i < spr; i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];
    if (ticks < -CPR_THRESHOLD) {
      ticks += cpr;

    }
    else if (ticks > CPR_THRESHOLD) {
      ticks -= cpr;
    }
   // SerialUSB.println(ticks);

    if (ticks > 1) {                                    //note starting point with iStart,jStart
      for (int j = 0; j < ticks; j++) {
        stepNo = (mod(fullStepReadings[i] + j, cpr));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

    if (ticks < 1) {                                    //note starting point with iStart,jStart
      for (int j = -ticks; j > 0; j--) {
        stepNo = (mod(fullStepReadings[spr - 1 - i] + j, cpr));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

  }

  // The code below generates the lookup table by intepolating between
  // full steps and mapping each encoder count to a calibrated angle
  // The lookup table is too big to store in volatile memory,
  // so we must generate and store it into the flash on the fly

  // begin the write to the calibration table
  page_count = 0;
  page_ptr = (const uint8_t*) lookup;
  puts("Writing to flash 0x \n\r");
  printf("%x ", page_ptr);

  for (int i = iStart; i < (iStart + spr + 1); i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];

    if (ticks < -CPR_THRESHOLD) {           //check if current interval wraps over encoder's zero positon
      ticks += cpr;
    }
    else if (ticks > CPR_THRESHOLD) {
      ticks -= cpr;
    }
    //Here we print an interpolated angle corresponding to each encoder count (in order)
    if (ticks > 1) {              //if encoder counts were increasing during cal routine...

      if (i == iStart) { //this is an edge case
        for (int j = jStart; j < ticks; j++) {
	  store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / (float)(ticks))), 360000.0));
        }
      }

      else if (i == (iStart + spr)) { //this is an edge case
        for (int j = 0; j < jStart; j++) {
	  store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / (float)(ticks))), 360000.0));
        }
      }
      else {                        //this is the general case
        for (int j = 0; j < ticks; j++) {
	  store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / (float)(ticks))), 360000.0));
        }
      }
    }

    else if (ticks < 1) {             //similar to above... for case when encoder counts were decreasing during cal routine
      if (i == iStart) {
        for (int j = - ticks; j > (jStart); j--) {
          store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / (float)(ticks))), 360000.0));
        }
      }
      else if (i == iStart + spr) {
        for (int j = jStart; j > 0; j--) {
          store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / (float)(ticks))), 360000.0));
        }
      }
      else {
        for (int j = - ticks; j > 0; j--) {
          store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / (float)(ticks))), 360000.0));
        }
      }

    }


  }

  puts("Calibration complete! \n\r");
  puts("The calibration table has been written to non-volatile Flash memory! \n\r");

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
	ioport_set_pin_dir(PIN_PC02,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_PC02, IOPORT_PIN_LEVEL_LOW);

	ioport_set_pin_peripheral_mode(PIN_PC01A_SPI_NPCS3,
		MUX_PC01A_SPI_NPCS3);


	//init pointer to the flash first page
	page_ptr = (const uint8_t*) lookup;

	/* Display menu. */
	display_menu();

	configure_tc();
	pid_init_data();

	uint8_t enable_close_loop = 0;

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
			readAttiny24Diagnostics(SPI_CHIP_PCS_1);
			break;
		case 't':
			//for(int i = 0; i < 200; i++)
				setAttiny24Motor(0x01, 12800, 0, SPI_CHIP_PCS_1);
			break;
		case 'y':
				setAttiny24Motor(0x01, 12800, 0, SPI_CHIP_PCS_5);
			break;
		case 'l':
			printf("Flash stats: start_addr: %x, flash_size: %d \n\r", FLASH_ADDR, FLASH_SIZE);
			printf("Add test angle to NVM, value count: %d, page no: %d, page_addr: %x \n\r",page_count, page_number , page_ptr);
			static float angleTest = 0.5;
			angleTest += 1.0;
			store_lookup(angleTest);
			break;
		case 'c':
			calibrate(SPI_CHIP_PCS_1);
			

			break;
		case 'p':
			puts("Print calibration data: \n\r");
			for(int i = 0; i < MAGNETIC_LUT_SIZE; i++)
			{
				char str[8];
				snprintf(str, sizeof(str), "%f", lookup[i]);
				printf("%s, ",str);
				if( (i % 16 == 0) && (i != 0) )
            		printf("\n\r");
			}
			break;
		case 'b':
			//setAttiny24Motor(0,0, 0);
			break;
		case 'f':
			printf("Angle: ");
			float angle = read_angle();
			char str[8];
			snprintf(str, sizeof(str), "%f", angle);
			printf(" %s \n\r",str);
			int raw = readEncoder();
			printf("Raw: %d \n\r", raw);
			break;
		case 's':
			//oneStep();
			r += 1.8;
			printf("Setpoint[step] = ");
			char str2[8];
			snprintf(str, sizeof(str2), "%f", r);
			printf(" %s \n\r",str2);

			break;
		case 'w':
			enable_close_loop ^= 1;
			if(enable_close_loop)
			{
				
				printf("Enabling close loop control! \n\r");
				tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
			}
			else
			{
				printf("Disabling close loop control! \n\r");
				tc_disable_interrupt(TC0, 0, TC_IER_CPCS);
			}
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
