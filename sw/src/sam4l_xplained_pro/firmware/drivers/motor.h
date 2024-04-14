#ifndef MOTOR_H
#define MOTOR_H

#include "asf.h"
#include "motor_calibration.h"


#define MOTOR_FORWARD   0x80
#define MOTOR_REVERSE   0x40

typedef enum {
    STATE_MOTOR_UNKNOWN = 0,
    STATE_MOTOR_OK,
    STATE_MOTOR_ERROR,
    STATE_MOTOR_COUNT
}sMotorState;

typedef struct {
    uint8_t motorID;
    uint8_t motorUsePCs;
    uint8_t motorPCs;
    uint8_t sensorPCs;
    uint8_t dir;
    sMotorState state;
    uint64_t stepNumber;
    int us_per_microstep;
    uint32_t RPM;
}sMotorInstance;


void motor_init(uMotorID motorID, uint8_t motorPCs, uint8_t sensorPCs, uint8_t motorUsePCs);
float motor_read_angle(uMotorID motorID);
int motor_read_position(uMotorID motorID);
void motor_one_step(uMotorID motorID, uint8_t dir);
void motor_microstep(uMotorID motorID, uint8_t dir);
void motor_diagnoise(uMotorID motorID);
void motor_calibrate(uMotorID motorID);
void motor_sync(uMotorID motorID);
void motor_printout(uMotorID motorID);
void motor_task();
sMotorState motor_get_status(uMotorID motorID);
int motor_set_rpm(uMotorID motorID, uint32_t RPM);

#endif /*MOTOR_H*/