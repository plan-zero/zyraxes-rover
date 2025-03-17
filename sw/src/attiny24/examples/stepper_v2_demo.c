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

#define F_CPU 8000000UL
#include <util/delay.h>
#include <math.h>

#define LED_PIN     PA3
#define LED_DDR     DDRA
#define LED_PORT    PORTA

#define SLP_PIN     PA0
#define SLP_DDR     DDRA
#define SLP_PORT    PORTA

#define STEP_PIN    PA1
#define STEP_DDR    DDRA
#define STEP_PORT   PORTA

#define DIR_PIN     PA2
#define DIR_DDR     DDRA
#define DIR_PORT    PORTA

int main(){

    //set LED outoput
    LED_DDR |= 1 << LED_PIN;
    //set motor driver OUTPUTs

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
        _delay_us(1);
        STEP_PORT &= ~(1 << STEP_PIN);
        _delay_us(20);
    }

    SLP_PORT &= ~(1 << SLP_PIN);

    LED_PORT |= 1 << LED_PIN;

    while(1)
    {
        //LED_PORT ^= 1 << LED_PIN;
        _delay_ms(1000);
    }

}

