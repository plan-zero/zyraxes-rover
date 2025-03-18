#include "uart.h"
#include "twi.h"
#include "firmware_hw.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include "stdint.h"




//TODO: read this from eprom
#define TWI_SLAVE_ADDRESS   0x70
#define TWI_MOTOR_DATA_SIZE 0x5

typedef struct{
    union{
        struct{
            uint8_t cmd;
            uint8_t steps[2];
            uint8_t rpm;
            uint8_t status
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

    /*LED init*/
    LED_DDR |= 1 << LED_PIN;
  

    /*enable global interrupts*/
    sei();
    uint16_t steps_to_do = 0;

    DDRB |= 1 << PINB2;

    while(1)
    {
        //getting data
        if(twi_rx_status == TWI_SLAVE_RX_DONE)
        {
            //copy buffer 
            cli();
            memcpy(twi_data.data, rxbuffer, 3);
            twi_rx_status = TWI_SLAVE_READY;
            sei();

            steps_to_do = twi_data.steps[0] << 8 | twi_data.steps[1];
            uart_printString("A", 0);
        }
        
        if(steps_to_do)
        {
            for(int i = 0; i < steps_to_do; i++)
            {
                STEP_PORT |= 1 << STEP_PIN;
                _delay_us(5);
                STEP_PORT &= ~(1 << STEP_PIN);
                _delay_us(40);
            }

            steps_to_do = 0;
        }


    }

    return 0;
}