#ifndef MOTOR_H
#define MOTOR_H

#include "asf.h"
#include "motor_calibration.h"


#define MOTOR_FORWARD   1
#define MOTOR_REVERSE   0

typedef enum {
    STATE_MOTOR_UNKNOWN = 0,
    STATE_MOTOR_OK,
    STATE_MOTOR_ERROR,
    STATE_MOTOR_COUNT
}sMotorState;

typedef struct {
    uint8_t motorID;
    uint8_t motorPCs;
    uint8_t sensorPCs;
    uint8_t dir;
    sMotorState state;
    int stepNumber;
    int us_per_microstep;
    
    uint32_t RPM;
    int init_pos;
    float init_angle;
    float angle;
    float angular_speed;
    float abs_angle;
    float calculated_rpm;
    int rotations;
}sMotorInstance;


void motor_init(uMotorID motorID, uint8_t motorPCs, uint8_t sensorPCs);
float motor_read_angle(uMotorID motorID);
int motor_read_position(uMotorID motorID);
void motor_one_step(uMotorID motorID, uint8_t dir);
void motor_microstep(uMotorID motorID, uint8_t dir, uint16_t steps, uint8_t rpm);
void motor_diagnoise(uMotorID motorID);
void motor_calibrate(uMotorID motorID);
void motor_sync(uMotorID motorID);
void motor_printout(uMotorID motorID);
void motor_task();
sMotorState motor_get_status(uMotorID motorID);
void motor_set_rpm(uMotorID motorID, uint8_t dir, uint32_t RPM);

#endif /*MOTOR_H*/