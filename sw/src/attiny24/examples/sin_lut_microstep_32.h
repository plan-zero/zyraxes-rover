#ifndef SIN_LUT_MICROSTEP_32
#define SIN_LUT_MICROSTEP_32

#include <avr/pgmspace.h>
#define POS_OFFSET 32
#define SIN_MAX_VALUE 127
#define SIN_LUT_LEN 128

extern const int8_t sin_lut[SIN_LUT_LEN] PROGMEM;

#endif

