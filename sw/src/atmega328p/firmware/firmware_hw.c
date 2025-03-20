#include "firmware_hw.h"
#include <avr/interrupt.h>

/*1us constant*/
#define TIMER2_CONSTANT_MS (uint32_t)(16384000 / 32 / 1000)
#define TIMER1_CONSTANT_MS (uint32_t)(16384000 / 1024 / 1000)

#define TIMER_MS_THRESHOLD (uint16_t)4000 /*maximum 4 seconds, minimum 1 ms*/
#define TIMER_US_THRESHOLD (uint16_t)499 /*maximum 500 us*/

void firmware_hw328p_timer_init()
{

    /*Init the LED PWM on timer 0*/
    /*clear OC0A on compare match*/
    /*set PWM pahse mode, top 0xFF*/
    TCCR0A |= (1 << COM0A1) | (1<<WGM00);
    /*set 8 prescaller*/
    TCCR0B |= (1 << CS01);
    /*LED init*/
    LED_DDR |= 1 << LED_PIN;

    /*Init CTC on timer 1 for task execution, no prescaler*/
    TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS12);
    TCNT1 = 0;
    OCR1A = TIMER1_CONSTANT_MS * 100;

    /*Init fixed timer for 100us to control the motor RPM*/
    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS21) | (1 << CS20);
    OCR2A = 0;

}

int firmware_hw328p_timer_start_A(uint16_t cmp_match)
{
    if(cmp_match > TIMER_MS_THRESHOLD)
        return -1;
    OCR1A = TIMER1_CONSTANT_MS * cmp_match;
    TCNT1 = 0;
    TIMSK1 |= (1 << OCIE1A);
    return 0;
}


int firmware_hw328p_timer_start_B(uint16_t cmp_match)
{
    if((uint16_t)cmp_match > TIMER_US_THRESHOLD)
        return -1;
    OCR2A = (uint8_t)((uint32_t)(TIMER2_CONSTANT_MS * (uint32_t)cmp_match) / (uint32_t)1000);
    TCNT2 = 0;
    TIMSK2 |= (1 << OCIE2A);
    return 0;
}

void firmware_hw328p_timer_stop_A()
{
	TIMSK1 &= ~(1 << OCIE1A);
}



void firmware_hw328p_timer_stop_B()
{
	TIMSK2 &= ~(1 << OCIE2A);
}