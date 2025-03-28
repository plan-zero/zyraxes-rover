#ifndef MOTOR_SPI_H
#define MOTOR_SPI_H

#include "motor_calibration.h"

uint16_t motor_micro_step_spi( uint8_t PCs, uint8_t dir, uint16_t steps, uint8_t rpm);
int  motor_read_raw_spi(uint8_t PCs);
int  motor_sync_spi(uint8_t PCs);
void motor_diag_spi(uint8_t PCs);
int motor_set_power_spi(uint8_t PCs, float power, unsigned char motor_config);

#endif