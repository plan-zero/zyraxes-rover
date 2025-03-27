#ifndef MOTOR_H
#define MOTOR_H

#include "asf.h"
#include "motor_calibration.h"


//this is clockwise 
#define MOTOR_FORWARD   1
//this is conterclockwise
#define MOTOR_REVERSE   0

#define MOTORS_TURNING_POS 5
extern int turning_speed_red[MOTORS_TURNING_POS];
extern int motors_turning_position;

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
    
    //user set RPM
    uint32_t RPM;
    //magnetic sensor init raw value & angle
    int init_pos;
    float init_angle;
    //magnetic sensor current angle read
    float angle;
    //current angular speed
    float angular_speed;
    //calculated RPM absed on sensor data
    float calculated_rpm;
    //overall rotations done by motor since start
    int rotations;

    //motor absolute set position and current absolute pos (deg)
    float angle_zero_set;
    float angle_absolute_set;
    int motor_position_needs_update;
    int idle;
    //go to zero position
    uint8_t go_zero;
    //gearbox value
    float gearbox;
    //current power
    float power;
}sMotorInstance;


void motor_init(uMotorID motorID, uint8_t motorPCs, uint8_t sensorPCs, uint8_t go_zero, float gearbox);
float motor_read_angle(uMotorID motorID);
int motor_read_position(uMotorID motorID);
void motor_one_step(uMotorID motorID, uint8_t dir);
void motor_microstep(uMotorID motorID, uint8_t dir, uint16_t steps, uint8_t rpm);
void motor_diagnoise(uMotorID motorID);
void motor_calibrate(uMotorID motorID);
void motor_sync(uMotorID motorID);
void motor_printout(uMotorID motorID);
void motor_task(void);
void motor_set_angle(uMotorID motorID, float angle);
sMotorState motor_get_status(uMotorID motorID);
void motor_set_rpm(uMotorID motorID, uint8_t dir, uint32_t RPM);
void motor_set_dir(uMotorID motorID, uint8_t dir);
void motor_set_power(uMotorID motorID, float power, unsigned char motor_config);
float motor_get_abs(uMotorID motorID);
void motor_soft_start(void);
void motor_soft_stop(void);

#endif /*MOTOR_H*/