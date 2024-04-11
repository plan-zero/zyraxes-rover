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
#include "string.h"
#include <tc.h>

#include "zyra_spi.h"
#include "motor.h"
#include "motor_calibration.h"


/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

#define ioport_set_pin_peripheral_mode(pin, mode) \
	do {\
		ioport_set_pin_mode(pin, mode);\
		ioport_disable_pin(pin);\
	} while (0)


#define STRING_EOL    "\r"
#define STRING_HEADER "\r\n -- ZYRAXES FIRMWARE --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/* UART baudrate. */
#define UART_BAUDRATE      115200



/**
 * \brief Display the user menu on the terminal.
 */
static void display_menu(void)
{

	puts("\n Plan-Zero Zyraxes Firmware\n\r");
	puts("\n\rInteractive Menu :\n\r"
			"------\r");

	puts("  q: Re-initialize SPI Master\n\r"
		 "0-7: Select motor\n\r"
		 "  a: sync\n\r"
		 "  t: full rotation test\n\r"
		 "  d: DIAG\n\r"
		 "  f: read\n\r"
		 "  s: step\n\r"
		 "  c: calibration \n\r"
		 "  p: print calibration data \n\r"
		 "  n: speed up \n\r\r"
		 "  m: speed down \n\r\r"
		 "  y: toggle direction\n\r\r"
		 "  h: Display this menu again\n\r\r");

}


void TC00_Handler(void)
{
	int ul_dummy;
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
	ioport_set_pin_dir(PIN_PC02,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_PC02, IOPORT_PIN_LEVEL_LOW);

	ioport_set_pin_peripheral_mode(PIN_PC01A_SPI_NPCS3,
		MUX_PC01A_SPI_NPCS3);

	motor_init(MOTOR_0, SPI_CHIP_PCS_0, SPI_CHIP_PCS_1);
	motor_init(MOTOR_2, SPI_CHIP_PCS_4, SPI_CHIP_PCS_5);


	//init pointer to the flash first page
	//page_ptr = (const uint8_t*) lookup;

	/* Display menu. */
	display_menu();

	configure_tc();

	uMotorID selected_motor = MOTOR_COUNT;
	uint8_t selected_dir = MOTOR_FORWARD;
	int motor_speed = MOTOR_MICROSTEP_WAIT_US;
	int ati_cmd = 0;

	while (1) {
		scanf("%c", (char *)&uc_key);

		//m 1,8 a t d f s c p n m h
		switch (uc_key) {
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
			ati_cmd = uc_key - '0';
			printf("Select MOTOR_%d \n\r", ati_cmd);
			if(motor_get_status(ati_cmd) == STATE_MOTOR_OK)
				selected_motor = ati_cmd;
			else
				printf("MOTOR_%d offline/error! \n\r", ati_cmd);
			break;
		case 'h':
			display_menu();
			break;

		case 'q':
			spi_master_initialize();
			break;
		case 'd':
			if(selected_motor < MOTOR_COUNT)
				motor_diagnoise(selected_motor);
			break;
		case 'a':
			if(selected_motor < MOTOR_COUNT)
				motor_sync(selected_motor);
			break;
		case 't':
			printf("Do 360 rotation test on motor: %d  \n\r", selected_motor);
			if(selected_motor < MOTOR_COUNT)
				for(int i = 0; i < MOTOR_SPR * MOTOR_MICROSTEP_CONFIG; i++){
					motor_microstep(selected_motor, selected_dir);
					delay_us(motor_speed);
				}
			break;
		case 'y':
			puts("Toggle dir! \n\r");
			if(selected_dir == MOTOR_FORWARD)
			{
				puts("Motor reverse! \n\r");
				selected_dir = MOTOR_REVERSE;
			}
			else
			{
				puts("Motor forward! \n\r");
				selected_dir = MOTOR_FORWARD;
			}
			break;

		case 'c':
			if(selected_motor < MOTOR_COUNT)
				motor_calibrate(selected_motor);

			break;
		case 'p':
			if(selected_motor < MOTOR_COUNT)
				motor_printout(selected_motor);
			break;
		case 'f':
			if(selected_motor < MOTOR_COUNT)
			{
				printf("Angle: ");
				float angle = motor_read_angle(selected_motor);
				char str[8];
				snprintf(str, sizeof(str), "%f", angle);
				printf(" %s \n\r",str);
				int raw = motor_read_position(selected_motor);
				printf("Raw: %d \n\r", raw);
			}
			break;
		case 's':
			if(selected_motor < MOTOR_COUNT)
				motor_one_step(selected_motor, selected_dir);
			break;
		case 'n':
			puts("Increase speed with 10p! \n\r");
			if(motor_speed > MOTOR_MICROSTEP_WAIT_US)
				motor_speed -= 10;
			break;
		case 'm':
			puts("Decrease speed with 10p! \n\r");
			motor_speed += 10;
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
