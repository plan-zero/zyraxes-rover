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
#include "twi_common.h"

//this is calculated based on the formula
//motor_step_us = MINUTE_US / (RPM * MOTOR_SPR * MICROSTEP_CONFIG)
//where MINUTE_US is 60000000us, MOTOR_SPR is 200 (nema17) and MICROSTEP_CONFIG is 32
#define RPM_CONST 9375
#define TIMER_US_MIN 30

volatile int timer_timer_trigger = 0;
volatile int timer_led_count = 0;
volatile int timer_sensor_count = 0;
volatile int timer_firmware_exec_count = 0;
volatile int timer_switch_count = 0;

ISR(TIMER1_COMPA_vect)
{
    
    timer_led_count++;
    timer_sensor_count++;
    timer_firmware_exec_count++;
    timer_switch_count++;
}


ISR(TIMER2_COMPA_vect)
{
    
    timer_timer_trigger = 1;
}



#define TWI_MOTOR_DATA_SIZE 0x5
#define TWI_SENSOR_DATA_SIZE 0x6
#define TWI_FIRMWARE_INFO_SIZE 0x3
#define TWI_FIRMWARE_CMD_SIZE 0x1



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
            uint32_t diag;
        };
        uint8_t data[TWI_SENSOR_DATA_SIZE];
    }sensor_data;
    uint8_t firmware_cmd;
    uint8_t firmware_info[TWI_FIRMWARE_INFO_SIZE];
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

    //update information into the TX buffer (this is not update in the main loop)
    txbuffer[TWI_TX_ADDR_FIRMWARE_VERSION] = firmware_version[0];
    txbuffer[TWI_TX_ADDR_FIRMWARE_VERSION + 1] = firmware_version[1];
    txbuffer[TWI_TX_ADDR_FIRMWARE_VERSION + 2] = firmware_version[2];

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

    //configure input, pull down resitor is on PCB
    SWITCH_DDR &= ~(1 << SWITCH_PIN);
    

    magnetic_sensor_diag();

    /*enable global interrupts*/
    sei();
    uint16_t steps_to_do = 0;
    uint16_t step_wait = 0;
    uint8_t free_running = 0;
    uint8_t force_stop = 0;
    uint8_t force_release = 0;
    uint8_t invalid_cmd = 0;
    uint8_t switch_press_threshold = 0;
    uint8_t switch_release_threshold = 0;
    unsigned char print_msg[10];
    twi_data.firmware_cmd = CMD_FIRMWARE_NONE;
    twi_data.motor_data.status = MOTOR_STATUS_IDLE;

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
            twi_data.firmware_cmd = rxbuffer[TWI_RX_ADDR_CMD_FIRMWARE];
            //clear rx buffer
            for(int i = 0;i < 10;i++)
                rxbuffer[i] = 0;
            twi_rx_status = TWI_SLAVE_READY;
            sei();

            
            if(!force_stop)
            {
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
            }
            else
            {
                if(twi_data.motor_data.cmd == CMD_MOTOR_RELEASE)
                {
                    //set the oposite direction
                    if(force_stop)
                    {
                        DIR_PORT ^= (1 << DIR_PIN);
                        force_release = 1;
                        twi_data.motor_data.rpm = 40;
                    }
                }
            }
            
            if(!invalid_cmd)
            {
                twi_data.motor_data.status = MOTOR_STATUS_BUSY;
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
                txbuffer[TWI_TX_ADDR_MOTOR_STATUS] = twi_data.motor_data.status;
                sei();

                //debug
                //itoa(twi_data.sensor_data.raw_value,print_msg,10);
                //uart_printString(print_msg,1);
                timer_sensor_count = 0;

            }
        }
        //do firmware tasks each 100ms
        if(timer_firmware_exec_count >= 100)
        {
            if(CMD_FIRMWARE_SENSOR_DIAG == twi_data.firmware_cmd)
            {
                cli();
                twi_data.sensor_data.diag = magnetic_sensor_diag();
                sei();
                
                txbuffer[TWI_TX_ADDR_SENSOR_DIAG1] = twi_data.sensor_data.data[2];
                txbuffer[TWI_TX_ADDR_SENSOR_DIAG1+1] = twi_data.sensor_data.data[3];
                txbuffer[TWI_TX_ADDR_SENSOR_DIAG2] = twi_data.sensor_data.data[4];
                txbuffer[TWI_TX_ADDR_SENSOR_DIAG2+1] = twi_data.sensor_data.data[5];
                twi_data.firmware_cmd = CMD_FIRMWARE_NONE;
            }

            timer_firmware_exec_count = 0;
        }

        if(timer_timer_trigger)
        {
            if(force_stop)
            {
                
                steps_to_do = 0;
                free_running = 0;
                if(force_release)
                {
                    STEP_PORT |= 1 << STEP_PIN;
                    _delay_us(5);
                    STEP_PORT &= ~(1 << STEP_PIN);
                }
                else
                {
                    firmware_hw328p_timer_stop_B();
                }
            }
            else if(free_running)
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
                twi_data.motor_data.status = MOTOR_STATUS_IDLE;
                firmware_hw328p_timer_stop_B();
            }
            timer_timer_trigger = 0;
            
        }

        //10 ms
        if(timer_switch_count >= 10)
        {
            if( ((SWITCH_OUTP >> SWITCH_PIN) & 0x1) && (switch_press_threshold < 10))
            {
                switch_press_threshold++;
            }
            else if((((SWITCH_OUTP >> SWITCH_PIN) & 0x1) == 0) && switch_release_threshold < 10)
            {
                switch_release_threshold++;
            }

            timer_switch_count = 0;
        }

        if(switch_release_threshold >= 10){
            firmware_hw328p_set_pwm_0(0);
            switch_press_threshold = 0;
            switch_release_threshold = 0;
            force_stop = 0;
            force_release = 0;
            if((twi_data.motor_data.status == MOTOR_STATUS_ENDSTOP_1) 
            || (twi_data.motor_data.status == MOTOR_STATUS_ENDSTOP_2))
            {
                twi_data.motor_data.status = MOTOR_STATUS_IDLE;
            }
        }
        else if(switch_press_threshold >=  10)
        {
            firmware_hw328p_set_pwm_0(100);
            force_stop = 1;
            timer_led_count = 0;
            
            //set error state as limit reach for TWI comm
            //set the LSB bit to 1 if DIR is pin is HIGH, this will send status 4 or 5 (ENDSTOP1 or ENDSTOP2)
            twi_data.motor_data.status = MOTOR_STATUS_ENDSTOP_1 | ((DIR_PORT >> DIR_PIN) & 1);
        }

        if(timer_led_count >= 10)
        {
            /*just turn LED off if it was turned ON when command was ACK*/
            firmware_hw328p_set_pwm_0(0);
        }

    }

    return 0;
}