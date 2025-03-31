#ifndef FIRMWARE_HW_H
#define FIRMWARE_HW_H

#include <avr/io.h>

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

#define SWITCH_PIN  PD2
#define SWITCH_PORT PORRD
#define SWITCH_OUTP PIND
#define SWITCH_DDR  DDRD


#define firmware_hw328p_set_pwm_0(x) OCR0A = (uint8_t)x
void firmware_hw328p_timer_init();
//takes the time in ms seconds
int firmware_hw328p_timer_start_A(uint16_t cmp_match);
//takes the time in us seconds
int firmware_hw328p_timer_start_B(uint16_t cmp_match);
void firmware_hw328p_timer_stop_A();
void firmware_hw328p_timer_stop_B();

#endif /*FIRMWARE_HW_H*/