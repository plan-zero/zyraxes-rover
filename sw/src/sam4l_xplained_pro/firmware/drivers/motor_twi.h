#ifndef MOTOR_TWI_H
#define MOTOR_TWI_H

#include "stdint.h"



void motor_diag_twi(uint8_t address);
int motor_sync_twi(uint8_t address);
uint16_t motor_micro_step_twi(uint8_t address, uint8_t dir, uint16_t steps, uint8_t rpm);
int motor_read_raw_twi(uint8_t address);
int motor_get_status_twi(uint8_t address);
int motor_power_off(uint8_t address);

#endif /*MOTOR_TWI_H*/