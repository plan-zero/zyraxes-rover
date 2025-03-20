#include "uart.h"
#include "twi.h"
#include "firmware_hw.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include "stdint.h"
#include <string.h>

//this is calculated based on the formula
//motor_step_us = MINUTE_US / (RPM * MOTOR_SPR * MICROSTEP_CONFIG)
//where MINUTE_US is 60000000us, MOTOR_SPR is 200 (nema17) and MICROSTEP_CONFIG is 32
#define RPM_CONST 9375
#define TIMER_US_MIN 30

volatile int timer_timer_trigger = 0;
volatile int timer_app_count = 1;
ISR(TIMER1_COMPA_vect)
{
    
    timer_app_count++;
    
}


ISR(TIMER2_COMPA_vect)
{
    PORTB ^= 1 << PINB2;
    timer_timer_trigger = 1;
}



//TODO: read this from eprom
#define TWI_SLAVE_ADDRESS   0x70
#define TWI_MOTOR_DATA_SIZE 0x5

typedef struct{
    union{
        struct{
            uint8_t cmd;
            uint8_t steps[2];
            uint8_t rpm;
            uint8_t status;
        };
        uint8_t data[TWI_MOTOR_DATA_SIZE];
    };
    uint8_t transfer_status;
}twi_data_t;

twi_data_t twi_data;

volatile uint8_t twi_rx_status = 0;
volatile uint8_t twi_tx_status = 0;


int main()
{

    /*Init uart for debugging/coms*/
    uart_init(UART_115200BAUD, UART_16384MHZ, UART_PARITY_NONE);
    /*Print initial message - print slave address as well*/
    uart_printString("ATmega328p - Rover firmware, version 2.0",1);
    uart_printString("TWI Slave address: ",0);
    uartPrintHex(TWI_SLAVE_ADDRESS);
    uartNewLine();
    /*Init TWI as slave with the give address*/
    I2C_init(TWI_SLAVE_ADDRESS);

    /*Initialize the motor driver, 32microsteps, enable SLP*/
    MICROSTEP_DDR |= MICROSTEP_32;
    MICROSTEP_PORT |= MICROSTEP_32;
    
    SLP_DDR |= 1 << SLP_PIN;
    STEP_DDR |= 1 << STEP_PIN;
    DIR_DDR |= 1 << DIR_PIN;

    SLP_PORT |= 1 << SLP_PIN;
    DIR_PORT |= 1 << DIR_PIN;



    firmware_hw328p_timer_init();

    int led_pwm = 0;
    int led_pwm_dir = 1;
    firmware_hw328p_set_pwm_0(0);
    //set timer at 100ms
    firmware_hw328p_timer_start_A(1);
    //firmware_hw328p_timer_start_B(200);

    /*enable global interrupts*/
    sei();
    uint16_t steps_to_do = 0;
    uint16_t step_wait = 0;

    DDRB |= 1 << PINB2;

    while(1)
    {
        //getting data
        if(twi_rx_status == TWI_SLAVE_RX_DONE)
        {
            //copy buffer 
            cli();
            twi_data.data[0] = rxbuffer[0];
            twi_data.data[1] = rxbuffer[1];
            twi_data.data[2] = rxbuffer[2];
            twi_rx_status = TWI_SLAVE_READY;
            sei();

            steps_to_do = twi_data.steps[0] << 8 | twi_data.steps[1];
            //set RPM
            step_wait = (RPM_CONST / (uint16_t)300);
            firmware_hw328p_timer_start_B((uint8_t)step_wait);
        }

        if(timer_timer_trigger)
        {
            if(steps_to_do)
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
        


       if(timer_app_count >= 10)
       {
            
            firmware_hw328p_set_pwm_0((uint8_t)led_pwm);
            led_pwm = led_pwm + led_pwm_dir;
            if(led_pwm == 200)
                led_pwm_dir = -1;
            else if(led_pwm == 5)
                led_pwm_dir = 1;
           timer_app_count = 0;
        }

    }

    return 0;
}