#ifndef FIRMWARE_HW_H
#define FIRMWARE_HW_H

/*MCU working frequency*/
#ifdef F_CPU
#undef F_CPU
#endif
#define F_CPU 16384000UL

/*Defines for the LED on the PCB*/
#define LED_PIN     PIND6
#define LED_DDR     DDRD
#define LED_PORT    PORTD

/*Motor control defs*/

#define SLP_PIN     PC2
#define SLP_DDR     DDRC
#define SLP_PORT    PORTC

#define STEP_PIN    PC0
#define STEP_DDR    DDRC
#define STEP_PORT   PORTC

#define DIR_PIN     PC1
#define DIR_DDR     DDRC
#define DIR_PORT    PORTC

#define MICROSTEP_32       0x38
#define MICROSTEP_PORT     PORTD
#define MICROSTEP_DDR      DDRD



#endif /*FIRMWARE_HW_H*/