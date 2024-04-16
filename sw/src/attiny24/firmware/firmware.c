/* Test application for the SPI-via-USI-driver. */

#include "spi_driver.h"
#include "sin_lut_microstep_16.h"
#include <avr/interrupt.h>

#define F_CPU 8000000UL
#include <util/delay.h>



#define SPIMODE 0	// Sample on leading _rising_ edge, setup on trailing _falling_ edge.

#define RPM_CONST 18750


#define IN1	PA0
#define IN2 PA1
#define IN3 PA2
#define IN4 PA3

#define INPORT PORTA

#define EFFORT 37


static inline void step(int8_t dir)
{
  int16_t v_coil_A = 0, v_coil_B = 0, angle_A = 0, angle_B = 0;
  static uint16_t pos_a = 0, pos_b = POS_OFFSET;


  angle_A = (int8_t)pgm_read_byte(&sin_lut[pos_a % SIN_LUT_LEN]);
  pos_a += dir;
  angle_B = (int8_t)pgm_read_byte(&sin_lut[pos_b % SIN_LUT_LEN]);
  pos_b += dir;

  v_coil_A = ( angle_A* EFFORT )/128;
  v_coil_B = ( angle_B* EFFORT )/128;

  OCR0B = abs(v_coil_A);
  OCR0A = abs(v_coil_B);

  INPORT &= 0xF0;
  if (v_coil_A >= 0)  {
    INPORT |= _BV(IN2);
    INPORT &= ~_BV(IN1);
  }
  else {
    INPORT &= ~_BV(IN2);
    INPORT |= _BV(IN1);
  }

  if (v_coil_B >= 0) {
    INPORT |= _BV(IN4);
    INPORT &= ~_BV(IN3);
  }
  else {
    INPORT &= ~_BV(IN4);
    INPORT |= _BV(IN3);
  }
}

#define TIMER_INTERVAL_US 100
#define TIMER_CONSTANT	(F_CPU / 8 / 1000000)

void timer_init() {
    // Set CTC mode with OCR1A as top value
    TCCR1B |= (1 << WGM12);
    
    // Set prescaler to 8
    TCCR1B |= (1 << CS11);
    
    // Set compare match value for 100us
    OCR1A = TIMER_CONSTANT * TIMER_INTERVAL_US;

	// Reset timer count reg to zero
	TCNT1 = 0;

	//Configure PWM for motor control
    TCCR0A |= _BV(COM0B1) | _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B |= _BV(CS00);
	DDRB |= _BV(PB2);
	DDRA |= _BV(PA7);
    OCR0A = 0;
    OCR0B = 0;

	
}

static inline void start_timer(uint16_t cmp_match)
{
	OCR1A = TIMER_CONSTANT * cmp_match;
	TCNT1 = 0;
	TIMSK1 |= (1 << OCIE1A);
}

static inline void stop_timer()
{
	TIMSK1 &= ~(1 << OCIE1A);
}

volatile char timer_triggered = 0;
ISR(TIM1_COMPA_vect) {
	timer_triggered = 1;
}



int main()
{

	
	timer_init();
	spiX_initslave(SPIMODE);	// Init SPI driver as slave.
	sei();		// Must do this to make driver work.


	//Configure IN1-4 as output
	DDRA |= _BV(IN1) | _BV(IN2) | _BV(IN3) | _BV(IN4);
	DDRB |= 1 << PB0;
	PORTB |= 1 << PB0;
	
	uint16_t steps_to_do = 0;
	uint8_t task = 0;
	int8_t dir = 0;
	uint8_t master_cmd = 0;
	uint16_t rpm_to_us = 0;
	spiX_put(SlaveSYNC);

	do {

		if(timer_triggered)
		{
			PORTB &= ~(1 << PB0);
			if(steps_to_do)
			{
				
				step(dir);
				//if this is free running
				if(steps_to_do != 0x1fff)
					steps_to_do--;
			}
			else
			{
				stop_timer();
				PORTB |= (1 << PB0);
			}
			timer_triggered = 0;
			PORTB |= (1 << PB0);
		}

		if(spiX_status.transferComplete)
		{
			master_cmd = spi_received_data[0] >> 6;
			if(master_cmd == MasterSYNC)
			{
				spiX_put(SlaveSYNC);
			}
			else if(master_cmd == MasterSTEP)
			{
				
				steps_to_do = ((spi_received_data[0] & 0x1f) << 8) | spi_received_data[1];
				if( (spi_received_data[0] >> 5) & 0x1)
					dir = 1;
				else
					dir = -1;
				
				rpm_to_us = RPM_CONST / (uint16_t) spi_received_data[2];
				if(rpm_to_us < TIMER_INTERVAL_US)
					rpm_to_us = TIMER_INTERVAL_US;
				start_timer(rpm_to_us);
				spiX_put(SlaveSTEP);
			}

			spiX_status.transferComplete = 0;
			
		}
		
		
		
	} while(1);			// Loop forever...
}
