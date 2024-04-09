#ifndef SIN_LUT_MICROSTEP_64
#define SIN_LUT_MICROSTEP_64

#include <avr/pgmspace.h>
#define POS_OFFSET 64
#define SIN_MAX_VALUE 127
#define SIN_LUT_LEN 256

extern const int8_t sin_lut[SIN_LUT_LEN] PROGMEM;

#endif

