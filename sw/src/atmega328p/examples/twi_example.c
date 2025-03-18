#define F_CPU 16384000UL

#include "uart.h"
#include "twi.h"
#include <util/delay.h>
#include <avr/interrupt.h>

#define LED_PIN     PIND6
#define LED_DDR     DDRD
#define LED_PORT    PORTD

int main()
{

    uart_init(UART_115200BAUD, UART_16384MHZ, UART_PARITY_NONE);
    uart_printString("TWI atmega328p example",1);
    I2C_init(0x70);



    for(int i = 0; i < 6; i++)
    {
        txbuffer[i] = '0';
        rxbuffer[i] = '0';
    }

    sei();

    int toggle = 1;
    LED_DDR |= 1 << LED_PIN;

    while(rxbuffer[5] != '\0');

    txbuffer[0] = 'H';
    txbuffer[1] = 'e';
    txbuffer[2] = 'l';
    txbuffer[3] = 'l';
    txbuffer[4] = 'o';
    txbuffer[5] = '\0';

    
    while(1)
    {
        if(toggle)
            uart_sendByte('A');
        else
            uart_sendByte('B');
        uartNewLine();
        for(int i = 0; i < 5; i++)
        {
            uart_sendByte(rxbuffer[i]);
        }
        uartNewLine();
        toggle ^= 1;

        _delay_ms(1000);
    }

    return 0;
}