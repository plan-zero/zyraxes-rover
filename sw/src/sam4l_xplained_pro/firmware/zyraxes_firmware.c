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
		 "  w: start motor control loop \n\r"
		 "  i: Increase RPM \n\r"
		 "  o: Decrease RPM \n\r"
		 "  b: Break all motors \n\r"
		 "  k: Dev SPI tryout \n\r"
		 "  x: Start commands\n\r"
		 "  h: Display this menu again\n\r\r");

}

volatile int do_motor_task = 0;
volatile int count = 0;

void TC00_Handler(void)
{
	int ul_dummy;
	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	do_motor_task = 1;

}

/**
 *  Configure Timer Counter 0 to generate an interrupt every 200ms.
 */
static void configure_tc(uint32_t freq)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	uint32_t handler_freq_hz = freq;

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
	uint32_t uc_key;
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

	//INIT CS for Encoder
	ioport_set_pin_dir(PIN_PC08,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_PC08, IOPORT_PIN_LEVEL_HIGH);

	ioport_set_pin_peripheral_mode(PIN_PC01A_SPI_NPCS3,
		MUX_PC01A_SPI_NPCS3);

	//wait for motors driver to start
	delay_ms(2000);
	motor_init(MOTOR_0, SPI_CHIP_PCS_0, SPI_CHIP_PCS_1, 0, 1);
	motor_init(MOTOR_1, SPI_CHIP_PCS_2, SPI_CHIP_PCS_3, 0, 1);
	motor_init(MOTOR_2, SPI_CHIP_PCS_4, SPI_CHIP_PCS_5, 0, 1);
	motor_init(MOTOR_3, SPI_CHIP_PCS_6, SPI_CHIP_PCS_7, 0, 1);
	motor_init(MOTOR_4, SPI_CHIP_PCS_8, SPI_CHIP_PCS_9, 0, 1);
	motor_init(MOTOR_5, SPI_CHIP_PCS_10, SPI_CHIP_PCS_11, 0, 1);
	motor_init(MOTOR_6, SPI_CHIP_PCS_12, SPI_CHIP_PCS_13, 0, 1);
	motor_init(MOTOR_7, SPI_CHIP_PCS_14, SPI_CHIP_PCS_15, 0, 1);
	//wait to go to zero pozition
	delay_ms(1000);
	motor_set_power(MOTOR_0, 0.8);
	motor_set_power(MOTOR_1, 0.8);
	motor_set_power(MOTOR_2, 0.8);
	motor_set_power(MOTOR_3, 0.8);
	motor_set_power(MOTOR_4, 0.8);
	motor_set_power(MOTOR_5, 0.8);
	motor_set_power(MOTOR_6, 0.8);
	motor_set_power(MOTOR_7, 0.8);


	//init pointer to the flash first page
	//page_ptr = (const uint8_t*) lookup;

	/* Display menu. */
	display_menu();

	//10ms
	configure_tc(20);

	uMotorID selected_motor = MOTOR_COUNT;
	uint8_t selected_dir = MOTOR_FORWARD;
	int motor_speed = MOTOR_MICROSTEP_WAIT_US;
	int ati_cmd = 0;
	int run_timer = 0;
	int interrupt_us = 0;
	int interrupt_hz = 0;

	uint8_t spi_data1 = 0, spi_data2 = 0, spi_data3 = 0;
	int toggle = 0;
	int process_extended = 0;

	int8_t x_val = 0;
	int8_t y_val = 0;
	int command_mode = 0;
	uint8_t js_rpm = 0;
	int js_timeout = 0;
	int left = 0;
	int right = 0;

	float angle_m0 = 0, angle_m1 = 0, angle_m2 = 0, angle_m3 = 0;
	float fangle_m0 = 0, fangle_m1 = 0, fangle_m2 = 0, fangle_m3 = 0;


	while (1) {
		//scanf("%c", (char *)&uc_key);

		//m 1,8 a t d f s c p n m h
		if(usart_is_rx_ready(CONF_UART))
		{
			usart_getchar(CONF_UART, &uc_key);
			//printf("recieved char %x \n\r", uc_key);
			switch (uc_key) {

			case 'x':
				command_mode = 1;
			break;					
			case 0x1b:
				process_extended = 1;
			break;
			case 0x44: //left
				if(process_extended)
				{
					if(left >= 0)
					{
						left--;
						right++;
						//printf("Left \n\r");
						angle_m0 = motor_read_angle(MOTOR_0);
						angle_m0 = motor_read_angle(MOTOR_2);
						angle_m0 = motor_read_angle(MOTOR_4);
						angle_m0 = motor_read_angle(MOTOR_6);

						motor_microstep(MOTOR_0, 0,  100 * MOTOR_MICROSTEP_CONFIG, 100);
						motor_microstep(MOTOR_2, 0,  100 * MOTOR_MICROSTEP_CONFIG, 100);

						motor_microstep(MOTOR_4, 1,  100 * MOTOR_MICROSTEP_CONFIG, 100);
						motor_microstep(MOTOR_6, 1,  100 * MOTOR_MICROSTEP_CONFIG, 100);

						fangle_m0 = motor_read_angle(MOTOR_0);
						fangle_m0 = motor_read_angle(MOTOR_2);
						fangle_m0 = motor_read_angle(MOTOR_4);
						fangle_m0 = motor_read_angle(MOTOR_6);

						delay_ms(10);

						if(abs(fangle_m0 - angle_m0) < 0.1)
						{
							motor_microstep(MOTOR_0, 0,  100 * MOTOR_MICROSTEP_CONFIG, 100);
							printf("1\n\r");

						}
						if(abs(fangle_m1 - angle_m1) < 0.1)
						{
							motor_microstep(MOTOR_2, 0,  100 * MOTOR_MICROSTEP_CONFIG, 100);
							printf("2\n\r");
						}
						if(abs(fangle_m2 - angle_m2) < 0.1)
						{
							motor_microstep(MOTOR_4, 1,  100 * MOTOR_MICROSTEP_CONFIG, 100);
							printf("3\n\r");
						}
						if(abs(fangle_m3 - angle_m3) < 0.1)
						{
							motor_microstep(MOTOR_6, 1,  100 * MOTOR_MICROSTEP_CONFIG, 100);
							printf("4\n\r");
						}


						delay_ms(150);
					}
					process_extended = 0;
				}
			break;

			case 0x41: //forward
				if(process_extended)
				{
					//printf("Forward \n\r");
					process_extended = 0;

					motor_microstep(MOTOR_5, 1,  MOTOR_SPR * MOTOR_MICROSTEP_CONFIG, 150);
					motor_microstep(MOTOR_1, 1,  MOTOR_SPR * MOTOR_MICROSTEP_CONFIG, 150);

					motor_microstep(MOTOR_7, 0,  MOTOR_SPR * MOTOR_MICROSTEP_CONFIG, 150);
					motor_microstep(MOTOR_3, 0,  MOTOR_SPR * MOTOR_MICROSTEP_CONFIG, 150);
					delay_ms(320);
				}
			break;

			case 0x42: //backward
				if(process_extended)
				{
					//printf("Backward \n\r");
					motor_microstep(MOTOR_5, 0,  MOTOR_SPR * MOTOR_MICROSTEP_CONFIG, 150);
					motor_microstep(MOTOR_1, 0,  MOTOR_SPR * MOTOR_MICROSTEP_CONFIG, 150);

					motor_microstep(MOTOR_7, 1,  MOTOR_SPR * MOTOR_MICROSTEP_CONFIG, 150);
					motor_microstep(MOTOR_3, 1,  MOTOR_SPR * MOTOR_MICROSTEP_CONFIG, 150);
					delay_ms(320);
					process_extended = 0;
				}
			break;

			case 0x43: //right
				if(process_extended)
				{
					if(right >= 0)
					{
						right--;
						left++;
						
						angle_m0 = motor_read_angle(MOTOR_0);
						angle_m0 = motor_read_angle(MOTOR_2);
						angle_m0 = motor_read_angle(MOTOR_4);
						angle_m0 = motor_read_angle(MOTOR_6);
						//printf("Right \n\r");
						motor_microstep(MOTOR_0, 1,  100 * MOTOR_MICROSTEP_CONFIG, 100);
						motor_microstep(MOTOR_2, 1,  100 * MOTOR_MICROSTEP_CONFIG, 100);
						motor_microstep(MOTOR_4, 0,  100 * MOTOR_MICROSTEP_CONFIG, 100);
						motor_microstep(MOTOR_6, 0,  100 * MOTOR_MICROSTEP_CONFIG, 100);
						fangle_m0 = motor_read_angle(MOTOR_0);
						fangle_m0 = motor_read_angle(MOTOR_2);
						fangle_m0 = motor_read_angle(MOTOR_4);
						fangle_m0 = motor_read_angle(MOTOR_6);

						delay_ms(10);

						if(abs(fangle_m0 - angle_m0) < 0.1)
						{
							motor_microstep(MOTOR_0, 1,  100 * MOTOR_MICROSTEP_CONFIG, 100);
							printf("1\n\r");

						}
						if(abs(fangle_m1 - angle_m1) < 0.1)
						{
							motor_microstep(MOTOR_2, 1,  100 * MOTOR_MICROSTEP_CONFIG, 100);
							printf("2\n\r");
						}
						if(abs(fangle_m2 - angle_m2) < 0.1)
						{
							motor_microstep(MOTOR_4, 0,  100 * MOTOR_MICROSTEP_CONFIG, 100);
							printf("3\n\r");
						}
						if(abs(fangle_m3 - angle_m3) < 0.1)
						{
							motor_microstep(MOTOR_6, 0,  100 * MOTOR_MICROSTEP_CONFIG, 100);
							printf("4\n\r");
						}

						delay_ms(150);
					}

					
					process_extended = 0;
				}
			break;
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
				{
					motor_microstep(selected_motor, selected_dir,  MOTOR_SPR * MOTOR_MICROSTEP_CONFIG, 150);
					delay_us(motor_speed);
				}
				break;
			case 'y':
				puts("Toggle dir! \n\r");
				if(selected_dir == MOTOR_FORWARD)
				{
					puts("Motor reverse! \n\r");
					selected_dir = MOTOR_REVERSE;
					motor_set_dir(selected_motor, selected_dir);
				}
				else
				{
					puts("Motor forward! \n\r");
					selected_dir = MOTOR_FORWARD;
					motor_set_dir(selected_motor, selected_dir);
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
					printf("Angle %s \n\r",str);
					angle = motor_get_abs(selected_motor);
					snprintf(str, sizeof(str), "%f", angle);
					printf("Abs %s \n\r",str);
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
			case 'w':
				/* Start the counter. */
				if(run_timer == 0)
				{
					puts("Start motor task \n\r");
					tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
					run_timer = 1;
				}
				else
				{
					puts("Stop motor task \n\r");
					tc_disable_interrupt(TC0, 0, TC_IER_CPCS);
					run_timer = 0;
				}
				break;
			case 'i':
				motor_set_rpm(selected_motor, selected_dir, 150);

			break;
			case 'o':
				motor_set_rpm(selected_motor, selected_dir, 180);

			break;
			case 'b':
				for(int i = MOTOR_0; i < MOTOR_COUNT; i++)
				{
					if(motor_get_status(i) == STATE_MOTOR_OK)
						motor_microstep(i, 0, 0, 0);
				}
			break;
			case 'k':

				if(toggle)
				{
					spi_data1 = spi_sync_transfer(0x7F, SPI_CHIP_PCS_0, 0);
					spi_data2 = spi_sync_transfer(0xff, SPI_CHIP_PCS_0, 0);
					spi_data3 = spi_sync_transfer(187, SPI_CHIP_PCS_0, 1);
					printf("SPI: %x%x%x \n\r", spi_data1, spi_data2, spi_data3);
				}
				else
				{
					spi_data1 = spi_sync_transfer(0x6C, SPI_CHIP_PCS_0, 0);
					spi_data2 = spi_sync_transfer(0x80, SPI_CHIP_PCS_0, 0);
					spi_data3 = spi_sync_transfer(187, SPI_CHIP_PCS_0, 1);
					printf("SPI: %x%x%x \n\r", spi_data1, spi_data2, spi_data3);
				}
				toggle ^= 1;
			break;

			default:
				break;
			}
		}

		if(do_motor_task){
			ioport_set_pin_level(PIN_PC08, IOPORT_PIN_LEVEL_LOW);
			motor_task();
			do_motor_task = 0;
			ioport_set_pin_level(PIN_PC08, IOPORT_PIN_LEVEL_HIGH);
		}

		if(command_mode)
		{

			while(command_mode)
			{
				//1s
				if(js_timeout > 10000)
				{
					motor_set_rpm(MOTOR_5, 0, 0);
					motor_set_rpm(MOTOR_1, 0, 0);

					motor_set_rpm(MOTOR_7, 0, 0);
					motor_set_rpm(MOTOR_3, 0, 0);

					
				}
				
					

				if(usart_is_rx_ready(CONF_UART))
				{
					usart_getchar(CONF_UART, &uc_key);
					switch (uc_key) {
						case '(':
							//wait to get all data
							while(!usart_is_rx_ready(CONF_UART));
							usart_getchar(CONF_UART, &uc_key);
							x_val = (int8_t)uc_key;
							while(!usart_is_rx_ready(CONF_UART));
							usart_getchar(CONF_UART, &uc_key);
							y_val = (int8_t)uc_key;
							while(!usart_is_rx_ready(CONF_UART));
							usart_getchar(CONF_UART, &uc_key);
							if(uc_key == ')')
							{
								//command done, process data
								printf("Got x=%d, y=%d \n\r", x_val, y_val);
							}
							//set RPM on motors here:
							if(y_val < 0)
							{
								//printf("Forward \n\r");
								js_rpm = abs(y_val);

								motor_set_rpm(MOTOR_5, 1, js_rpm);
								motor_set_rpm(MOTOR_1, 1, js_rpm);

								motor_set_rpm(MOTOR_7, 0, js_rpm);
								motor_set_rpm(MOTOR_3, 0, js_rpm);
								//delay_ms(100);
							}
							else
							{
								//printf("Forward \n\r");
								js_rpm = abs(y_val);

								motor_set_rpm(MOTOR_5, 0, js_rpm);
								motor_set_rpm(MOTOR_1, 0, js_rpm);

								motor_set_rpm(MOTOR_7, 1, js_rpm);
								motor_set_rpm(MOTOR_3, 1, js_rpm);
								//delay_ms(100);

							}


							js_timeout = 0;
						break;
						case 'e':
							scanf("%c", (char *)&uc_key);
							if(uc_key == 's')
							{
								scanf("%c", (char *)&uc_key);
								if(uc_key = 's')
									command_mode = 0;
							}
							
						break;

						default:
						break;

					}

				}

				delay_us(100);
				js_timeout++;

			}
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
