#include "uart.h"
#include "twi.h"
#include "firmware_hw.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include "stdint.h"
#include <string.h>
#include "magnetic_sensor.h"
#include "spi.h"
#include "stdlib.h"
#include "firmware_e2p.h"

//this is calculated based on the formula
//motor_step_us = MINUTE_US / (RPM * MOTOR_SPR * MICROSTEP_CONFIG)
//where MINUTE_US is 60000000us, MOTOR_SPR is 200 (nema17) and MICROSTEP_CONFIG is 32
#define RPM_CONST 9375
#define TIMER_US_MIN 30

volatile int timer_timer_trigger = 0;
volatile int timer_led_count = 0;
volatile int timer_sensor_count = 0;
ISR(TIMER1_COMPA_vect)
{
    
    timer_led_count++;
    timer_sensor_count++;
    
}


ISR(TIMER2_COMPA_vect)
{
    
    timer_timer_trigger = 1;
}



#define TWI_MOTOR_DATA_SIZE 0x5
#define TWI_SENSOR_DATA_SIZE 0x7

typedef enum{
    CMD_MOTOR_STEP_CW = 0x1,
    CMD_MOTOR_STEP_CCW = 0x2,
    CMD_MOTOR_RUN_CW = 0x3,
    CMD_MOTOR_RUN_CCW = 0x4,
    CMD_MOTOR_STOP = 0x5,
    CMD_MOTOR_POWEROFF = 0x6,
    CMD_MOTOR_SYNC = 0x7
}MasterCMD;

enum{
    TWI_RX_ADDR_CMD = 0,
    TWI_RX_ADDR_STEPS = 1,
    TWI_RX_ADDR_RPM = 3,
    TWI_RX_ADDR_CMD_DIAG = 4
};
enum{
    TWI_TX_ADDR_SENSOR_RAW = 0,
    TWI_TX_ADDR_SENSOR_DIAG1 = 2,
    TWI_TX_ADDR_SENSOR_DIAG2 = 4,
    TWI_TX_ADDR_MOTOR_STATUS = 5
};

typedef struct{
    union{
        struct{
            uint8_t cmd;
            uint16_t steps;
            uint8_t rpm;
            uint8_t status;
        };
        uint8_t data[TWI_MOTOR_DATA_SIZE];
    }motor_data;
    union
    {
        struct{
            uint16_t raw_value;
            uint16_t diag1;
            uint16_t diag2;
            uint8_t cmd;
        };
        uint8_t sensor_data[TWI_SENSOR_DATA_SIZE];
    }sensor_data;
    uint8_t transfer_status;
}twi_data_t;

twi_data_t twi_data;

volatile uint8_t twi_rx_status = 0;
volatile uint8_t twi_tx_status = 0;
uint8_t twi_chip_address[TWI_CHIP_ADDR_LENGTH];
uint8_t firmware_version[FIRMWARE_VERSION_LENGTH];


int main()
{
    /*Early initialization of CS as it is an input at startup, reading 0 logic*/
	/*this seems to affect the sensor comunication as there is no pullup resistor*/
	DDRB |= 1 << PB2;
	PORTB |= 1 << PB2;
    /*Init uart for debugging/coms*/

    /*read eeprom data*/
    e2p_read_twi_address(twi_chip_address[0]);
    e2p_read_firmware_version(firmware_version[0]);

    firmware_hw328p_timer_init();
    firmware_hw328p_set_pwm_0(0);
    //set timer at 100ms
    firmware_hw328p_timer_start_A(1);


    uart_init(UART_115200BAUD, UART_16384MHZ, UART_PARITY_NONE);

    int led_pwm = 0;
    int led_pwm_dir = 1;

    if(0xFF == twi_chip_address[0])
    {
        uart_printString("Atmega328p - invalid E2P data, program eeprom!", 1);
        sei();
        while(1)
        {
            /*do led patern here*/
            if(timer_led_count >= 10)
            {
                 
                 firmware_hw328p_set_pwm_0((uint8_t)led_pwm);
                 led_pwm = led_pwm + led_pwm_dir;
                 if(led_pwm == 200)
                     led_pwm_dir = -1;
                 else if(led_pwm == 5)
                     led_pwm_dir = 1;
                timer_led_count = 0;
             }
        }
    }
    /*Turn LED off*/
    firmware_hw328p_set_pwm_0(0);

    
    /*Print initial message - print slave address as well*/
    uart_printString("ATmega328p - Rover firmware, version v",0);
    uart_sendByte('0'+ firmware_version[0]);
    uart_sendByte('.');
    uart_sendByte('0'+ firmware_version[1]);
    uart_sendByte('.');
    uart_sendByte('0'+ firmware_version[2]);
    uartNewLine();

    uart_printString("TWI Slave address: ",0);
    uart_printRegister(twi_chip_address[0]);
    uartNewLine();
    /*Init TWI as slave with the give address*/
    I2C_init(twi_chip_address[0]);
    /*Init SPI master*/
    spi_master_init();

    /*Initialize the motor driver, 32microsteps, enable SLP*/
    MICROSTEP_DDR |= MICROSTEP_32;
    MICROSTEP_PORT |= MICROSTEP_32;
    
    SLP_DDR |= 1 << SLP_PIN;
    STEP_DDR |= 1 << STEP_PIN;
    DIR_DDR |= 1 << DIR_PIN;

    SLP_PORT |= 1 << SLP_PIN;
    DIR_PORT |= 1 << DIR_PIN;

    magnetic_sensor_diag();

    /*enable global interrupts*/
    sei();
    uint16_t steps_to_do = 0;
    uint16_t step_wait = 0;
    uint8_t free_running = 0;
    uint8_t invalid_cmd = 0;
    unsigned char print_msg[10];

    DDRB |= 1 << PINB2;

    twi_tx_status = TWI_SLAVE_READY;

    while(1)
    {
        //getting data
        //get all rx data
        if(twi_rx_status == TWI_SLAVE_RX_DONE)
        {
            //copy buffer 
            cli();
            twi_data.motor_data.cmd = rxbuffer[TWI_RX_ADDR_CMD];
            twi_data.motor_data.steps = rxbuffer[TWI_RX_ADDR_STEPS] << 8 | rxbuffer[TWI_RX_ADDR_STEPS+1];
            twi_data.motor_data.rpm = rxbuffer[TWI_RX_ADDR_RPM];
            twi_data.sensor_data.cmd = txbuffer[TWI_RX_ADDR_CMD_DIAG];
            //clear rx buffer
            for(int i = 0;i < 10;i++)
                rxbuffer[i] = 0;
            twi_rx_status = TWI_SLAVE_READY;
            sei();

            

            if(twi_data.motor_data.cmd == CMD_MOTOR_STEP_CW)
            {
                SLP_PORT |= (1 << SLP_PIN);
                DIR_PORT |= 1 << DIR_PIN;
                free_running = 0;
                invalid_cmd = 0;
                steps_to_do = twi_data.motor_data.steps;
            }
            else if(twi_data.motor_data.cmd == CMD_MOTOR_STEP_CCW)
            {
                SLP_PORT |= (1 << SLP_PIN);
                DIR_PORT &= ~(1 << DIR_PIN);
                free_running = 0;
                invalid_cmd = 0;
                steps_to_do = twi_data.motor_data.steps;
            }
            else if(twi_data.motor_data.cmd == CMD_MOTOR_RUN_CW)
            {
                SLP_PORT |= (1 << SLP_PIN);
                DIR_PORT |= 1 << DIR_PIN;
                free_running = 1;
                invalid_cmd = 0;
                steps_to_do = 0;
            }
            else if(twi_data.motor_data.cmd == CMD_MOTOR_RUN_CCW)
            {
                SLP_PORT |= (1 << SLP_PIN);
                DIR_PORT &= ~(1 << DIR_PIN);
                free_running = 1;
                invalid_cmd = 0;
                steps_to_do = 0;
            }
            else if(twi_data.motor_data.cmd == CMD_MOTOR_STOP)
            {
                SLP_PORT |= (1 << SLP_PIN);
                free_running = 0;
                steps_to_do = 0;
                firmware_hw328p_timer_stop_B();
                
                invalid_cmd = 0;
            }
            else if(twi_data.motor_data.cmd == CMD_MOTOR_POWEROFF)
            {
                steps_to_do = 0;
                free_running = 0;
                firmware_hw328p_timer_stop_B();
                SLP_PORT &= ~(1 << SLP_PIN);
                invalid_cmd = 0;
            }
            else
            {
                invalid_cmd = 1;
            }
            
            if(!invalid_cmd)
            {
                firmware_hw328p_set_pwm_0(100);
                timer_led_count = 0;
                step_wait = (RPM_CONST / (uint16_t)twi_data.motor_data.rpm);
                firmware_hw328p_timer_start_B((uint8_t)step_wait);
                
            }
                
        }

        if(twi_tx_status == TWI_SLAVE_TX_DONE)
        {
            //do something once data is out, move it to ready to update the rxbuffer
            twi_tx_status = TWI_SLAVE_READY;
        }
        else if(twi_tx_status == TWI_SLAVE_READY)
        {
            //read every 10 ms the sensor data
            if(timer_sensor_count >= 10)
            {
                cli();
                twi_data.sensor_data.raw_value = magnetic_sensor_read();
                //update txbuffer
                txbuffer[TWI_TX_ADDR_SENSOR_RAW] = (twi_data.sensor_data.raw_value & 0xFF00) >> 8;
                txbuffer[TWI_TX_ADDR_SENSOR_RAW+1] = twi_data.sensor_data.raw_value & 0x00FF;
                sei();

                //debug
                //itoa(twi_data.sensor_data.raw_value,print_msg,10);
                //uart_printString(print_msg,1);
                timer_sensor_count = 0;
            }
        }

        if(timer_timer_trigger)
        {
            if(free_running)
            {
                STEP_PORT |= 1 << STEP_PIN;
                _delay_us(5);
                STEP_PORT &= ~(1 << STEP_PIN);
            }
            else if(steps_to_do)
            {
                STEP_PORT |= 1 << STEP_PIN;
                _delay_us(5);
                STEP_PORT &= ~(1 << STEP_PIN);
                steps_to_do--;
            }
            else
            {
                firmware_hw328p_timer_stop_B();
            }
            timer_timer_trigger = 0;
            
        }

        if(timer_led_count >= 10)
        {
            /*just turn LED off if it was turned ON when command was ACK*/
            firmware_hw328p_set_pwm_0(0);
        }

    }

    return 0;
}