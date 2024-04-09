#ifndef SIN_LUT_MICROSTEP_16
#define SIN_LUT_MICROSTEP_16

#include <avr/pgmspace.h>
#define POS_OFFSET 16
#define SIN_MAX_VALUE 127
#define SIN_LUT_LEN 64

extern const int8_t sin_lut[SIN_LUT_LEN] PROGMEM;


#endif

