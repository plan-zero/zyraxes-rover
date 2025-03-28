#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

typedef enum {
    MOTOR_0 = 0,
    MOTOR_1,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    MOTOR_5,
    MOTOR_6,
    MOTOR_7,
    MOTOR_COUNT

}uMotorID;

#define MOTOR_MICROSTEP_CONFIG      32
#define MOTOR_MICROSTEP_WAIT_US     100

#define MINUTE_TO_US 60000000
#define MOTOR_APS    1.8
#define MOTOR_SPR 200

#define DEFAULT_CPR 16384
#define DEFAULT_CPR_THRESHOLD 15000
//used to reduce the resolution of the magnetic data as calibration data is huge
//anyway for our aplication it's enough to use only 11 bits, increase below number 
//to reduce more bits
#define MAGNETIC_REDUCE_RESOLUTION 2
#define CPR (DEFAULT_CPR >> MAGNETIC_REDUCE_RESOLUTION)
#define CPR_THRESHOLD (DEFAULT_CPR_THRESHOLD >> MAGNETIC_REDUCE_RESOLUTION)
#define MAGNETIC_LUT_SIZE CPR





#endif