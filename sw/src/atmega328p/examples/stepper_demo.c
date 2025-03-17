/**
 * Copyright (C) 2025 Coding Night Romania
 * 
 * This file is part of zyraxes-rover.
 * 
 * zyraxes-rover is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * zyraxes-rover is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with zyraxes-rover.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>

#define F_CPU 16384000UL
#include <util/delay.h>
#include <math.h>

#define LED_PIN     PIND6
#define LED_DDR     DDRD
#define LED_PORT    PORTD

#define SLP_PIN     PC2
#define SLP_DDR     DDRC
#define SLP_PORT    PORTC

#define STEP_PIN    PC0
#define STEP_DDR    DDRC
#define STEP_PORT   PORTC

#define DIR_PIN     PC1
#define DIR_DDR     DDRC
#define DIR_PORT    PORTC

#define MICROSTEP_PORT_VAL 0x38
#define MICROSTEP_PORT     PORTD
#define MICROSTEP_DDR      DDRD

int main(){

    //set LED outoput
    LED_DDR |= 1 << LED_PIN;
    //set motor driver OUTPUTs

    //set 32microstep
    MICROSTEP_DDR |= MICROSTEP_PORT_VAL;
    MICROSTEP_PORT |= MICROSTEP_PORT_VAL;
    

    SLP_DDR |= 1 << SLP_PIN;
    STEP_DDR |= 1 << STEP_PIN;
    DIR_DDR |= 1 << DIR_PIN;


    SLP_PORT |= 1 << SLP_PIN;
    DIR_PORT |= 1 << DIR_PIN;

    _delay_ms(1000);

    //full rotation microstep 32
    STEP_PORT &= ~(1 << STEP_PORT);
    for(int i = 0; i < 12800; i++)
    {
        STEP_PORT |= 1 << STEP_PIN;
        _delay_us(5);
        STEP_PORT &= ~(1 << STEP_PIN);
        _delay_us(40);
    }

    SLP_PORT &= ~(1 << SLP_PIN);
    

    while(1)
    {
        LED_PORT ^= 1 << LED_PIN;
        _delay_ms(1000);
    }

}

