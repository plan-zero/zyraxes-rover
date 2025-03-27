#ifndef MOTOR_SPI_H
#define MOTOR_SPI_H

#include "motor_calibration.h"

uint16_t _motor_micro_step( uint8_t PCs, uint8_t dir, uint16_t steps, uint8_t rpm);
int _motor_read_raw(uint8_t PCs);
int  _motor_sync(uMotorID motorID, uint8_t PCs);
void _motor_diag(uint8_t PCs);
int _motor_set_power(uint8_t PCs, float power, unsigned char motor_config);

#endif