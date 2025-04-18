#include "motor_calibration.h"
#include "motor.h"
#include "motor_spi.h"
#include "motor_twi.h"

#include "delay.h"
#include "nvm.h"
#include "string.h"



const float angle_step_conv = 360.0 / ((float)MOTOR_SPR * (float)MOTOR_MICROSTEP_CONFIG);

const float startup_angle[MOTOR_COUNT] = 
{
    211.0,
    0,
    316.68,
    0,
    57.9,
    0,
    120.32,
    0
};

const float __attribute__((__aligned__(512))) lookup[MOTOR_COUNT][MAGNETIC_LUT_SIZE] = {
    //Put lookup table here!
};


int turning_speed_red[MOTORS_TURNING_POS] = {0, 15, 30, 45, 60};
int motors_turning_position = 0;


sMotorInstance _motors[MOTOR_COUNT];


static inline void _motor_diag(uMotorID motorID)
{
    if(_motors[motorID].motorType == TYPE_MOTOR_SPI)
    {
        motor_diag_spi(_motors[motorID].sensorPCs);
    }
    else if(_motors[motorID].motorType == TYPE_MOTOR_TWI)
    {
        motor_diag_twi(_motors[motorID].slaveAddress);
    }
    else
    {
        printf("_motor_diag: error, invalid motor type! \n\r");
    }
}

static inline int _motor_sync(uMotorID motorID)
{
    if(_motors[motorID].motorType == TYPE_MOTOR_SPI)
    {
        return motor_sync_spi(_motors[motorID].motorPCs);
    }
    else if(_motors[motorID].motorType == TYPE_MOTOR_TWI)
    {
        return motor_sync_twi(_motors[motorID].slaveAddress);
    }
    else
    {
        printf("_motor_sync: error, invalid motor type! \n\r");
        return 1;
    }
}

static inline uint16_t _motor_micro_step( uMotorID motorID, uint8_t dir, uint16_t steps, uint8_t rpm)
{
    if(_motors[motorID].motorType == TYPE_MOTOR_SPI)
    {
        return motor_micro_step_spi(_motors[motorID].motorPCs, dir, steps, rpm);
    }
    else if(_motors[motorID].motorType == TYPE_MOTOR_TWI)
    {
        return motor_micro_step_twi(_motors[motorID].slaveAddress, dir, steps, rpm);
    }
    else
    {
        printf("_motor_micro_step: error, invalid motor type! \n\r");
        return 1;
    }
}

static inline int _motor_read_raw(uMotorID motorID)
{
    if(_motors[motorID].motorType == TYPE_MOTOR_SPI)
    {
        return motor_read_raw_spi(_motors[motorID].sensorPCs);
    }
    else if(_motors[motorID].motorType == TYPE_MOTOR_TWI)
    {
        return motor_read_raw_twi(_motors[motorID].slaveAddress);
    }
    else
    {
        printf("_motor_read_raw: error, invalid motor type! \n\r");
        return 1;
    }
}

static inline int _motor_set_power(uMotorID motorID, float power, unsigned char motor_config)
{
    if(_motors[motorID].motorType == TYPE_MOTOR_SPI)
    {
        return motor_set_power_spi(_motors[motorID].motorPCs, power, motor_config);
    }
    else if(_motors[motorID].motorType == TYPE_MOTOR_TWI)
    {
        printf("_motor_read_raw: error, TWI version doesn't support dynamic power setup! \n\r");
        return -1;
    }
    else
    {
        printf("_motor_read_raw: error, invalid motor type! \n\r");
        return -1;
    }
}

static inline int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}


void motor_init(uMotorID motorID, sMotorType motorType, uint8_t slaveAddress, uint8_t motorPCs, uint8_t sensorPCs, uint8_t go_zero, float gearbox)
{
    if(motorType == TYPE_MOTOR_SPI)
        printf("motor_init: Initialize motor ID=%d, PCs selected driver %d senzor %d... \n\r", motorID, motorPCs, sensorPCs);
    else if (motorType == TYPE_MOTOR_TWI)
        printf("motor_init: Initialize motor ID=%d, slave address %x... \n\r", motorID, slaveAddress);
    else
        printf("motor_init: Invalid motor type, exiting initalization... \n\r");

    _motors[motorID].state = MOTOR_STATUS_UNKNOWN;
    _motors[motorID].motorPCs = motorPCs;
    _motors[motorID].sensorPCs = sensorPCs;
    _motors[motorID].motorID = motorID;
    _motors[motorID].us_per_microstep = 0;
    _motors[motorID].RPM = 0;
    _motors[motorID].gearbox = gearbox;
    _motors[motorID].idle = 1;
    _motors[motorID].motorType = motorType;
    _motors[motorID].slaveAddress = slaveAddress;

    printf("motor_init: sync with motor driver, sending... \n\r");


    if(_motor_sync(motorID))
    {
        _motors[motorID].state = MOTOR_STATUS_ERROR;
        printf("motor_init: sync error, id:%d \n\r", motorID);
    }
    else
    {
        _motors[motorID].state = MOTOR_STATUS_IDLE;
        char str[10];
        //get initial senzor data
        _motors[motorID].init_pos = _motor_read_raw(motorID);
        //get few reads to make sure there is no flush data in SPI
        for(int i = 0; i < 2; i++) {
            _motors[motorID].init_angle = motor_read_angle(motorID);
            snprintf(str, sizeof(str), "%f", _motors[motorID].init_angle);
            printf("init_angle: %s, PCs %d \n\r", str, _motors[motorID].motorPCs);
        }
        _motors[motorID].angular_speed = 0;
        _motors[motorID].rotations = 0;
        //set the initial angle
        _motors[motorID].angle_zero_set = startup_angle[motorID];
        _motors[motorID].go_zero = go_zero;
        _motors[motorID].angle_absolute_set = 0;
        _motors[motorID].motor_position_needs_update = 0;

        if(go_zero)
        {
            float delta_angle = _motors[motorID].angle_zero_set - _motors[motorID].init_angle;
            unsigned int steps_to_zero = 0;


            if(abs(delta_angle) <= 180.0)
            {
                steps_to_zero = (unsigned int)(abs(delta_angle) / angle_step_conv);
                if(delta_angle <= 0){
                    _motor_micro_step(motorID, MOTOR_FORWARD, steps_to_zero, 50);
                    printf("1\n\r");
                }
                else
                {
                     _motor_micro_step(motorID, MOTOR_REVERSE, steps_to_zero, 50);
                     printf("2\n\r");
                }
            }
            else
            {
                steps_to_zero = (unsigned int)( (360.0 - abs(delta_angle)) / angle_step_conv);
                if(delta_angle <= 0)
                {
                    _motor_micro_step(motorID, MOTOR_REVERSE, steps_to_zero, 50);
                    printf("3\n\r");
                }
                else
                {
                     _motor_micro_step(motorID, MOTOR_FORWARD, steps_to_zero, 50);
                    printf("4\n\r");
                }
            }
            //debug
            snprintf(str, sizeof(str), "%f", delta_angle);
            printf("delta: %s, steps %d \n\r", str, steps_to_zero);

            _motors[motorID].init_angle = motor_read_angle(motorID);
            _motors[motorID].angle = _motors[motorID].init_angle;
          
            
        }
    }

}

float motor_get_abs(uMotorID motorID)
{
    //read angle from sensor
    float sign = -1.0;
    float abs_pos = (_motors[motorID].init_angle + ( (float)_motors[motorID].rotations * 360.0) + ( _motors[motorID].angle_zero_set) * sign) / _motors[motorID].gearbox; 
    return abs_pos;
}

sMotorState motor_get_status(uMotorID motorID)
{

    if(_motors[motorID].motorType == TYPE_MOTOR_TWI)
    {
        _motors[motorID].state =  motor_get_status_twi(_motors[motorID].slaveAddress);

    }
    return _motors[motorID].state;
}

void motor_microstep(uMotorID motorID, uint8_t dir, uint16_t steps, uint8_t rpm)
{
    uint16_t ret = 0;
    _motors[motorID].dir = dir;

    ret = _motor_micro_step(motorID, dir, steps, rpm);
    if(ret)
        printf("motor_microstep: ID: %d failed \n\r", motorID);
}

void motor_one_step(uMotorID motorID, uint8_t dir)
{
        _motors[motorID].dir = dir;
        _motor_micro_step(motorID, dir, MOTOR_MICROSTEP_CONFIG, 150);
        delay_us(187 * MOTOR_MICROSTEP_WAIT_US);
}

float motor_read_angle(uMotorID motorID)
{
    if((MOTOR_STATUS_UNKNOWN == _motors[motorID].state) ||  (_motors[motorID].state == MOTOR_STATUS_ERROR))
    {   
        printf("motor_read_angle: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return 0;
    }
    const int avg = 1;
    int encoderReading = 0;

    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
        encoderReading += _motor_read_raw(motorID);
    }

    return encoderReading * (360.0 / 4096) / avg;
    //TODO: fix calibration, until then use direct data
    //return lookup[motorID][encoderReading / avg];

}

int motor_read_position(uMotorID motorID)
{
    if((MOTOR_STATUS_UNKNOWN == _motors[motorID].state) ||  (_motors[motorID].state == MOTOR_STATUS_ERROR))
    {   
        printf("motor_read_position: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return 0;
    }
    return _motor_read_raw(motorID);
}

void motor_sync(uMotorID motorID)
{
    if((MOTOR_STATUS_UNKNOWN == _motors[motorID].state) ||  (_motors[motorID].state == MOTOR_STATUS_ERROR))
    {   
        printf("motor_sync: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    _motor_sync(motorID);
}

void motor_diagnoise(uMotorID motorID)
{
    if((MOTOR_STATUS_UNKNOWN == _motors[motorID].state) ||  (_motors[motorID].state == MOTOR_STATUS_ERROR))
    {   
        printf("motor_diagnoise: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    _motor_diag(motorID);
}

void motor_printout(uMotorID motorID)
{
    if((MOTOR_STATUS_UNKNOWN == _motors[motorID].state) ||  (_motors[motorID].state == MOTOR_STATUS_ERROR))
    {   printf("motor_printout: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    puts("motor_printout: print calibration data ... \n\r");
    for(int i = 0; i < MAGNETIC_LUT_SIZE; i++)
    {
        char str[10];
        snprintf(str, sizeof(str), "%f", lookup[motorID][i]);
        printf("%s, ",str);
        if( (i % 16 == 0) && (i != 0) )
            printf("\n\r");
    }
}

void motor_set_angle(uMotorID motorID, float angle)
{
    _motors[motorID].angle_absolute_set = angle;
    _motors[motorID].motor_position_needs_update = 1;
}

void motor_calibrate(uMotorID motorID) { 

    if((MOTOR_STATUS_UNKNOWN == _motors[motorID].state) ||  (_motors[motorID].state == MOTOR_STATUS_ERROR))
    {   
        printf("motor_calibrate: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    
    static int cpr = CPR;
    static float aps = MOTOR_APS;
    int encoderReading = 0;     //or float?  not sure if we can average for more res?
    int currentencoderReading = 0;
    int lastencoderReading = 0;
    int avg = 10;               //how many readings to average

    int iStart = 0;     //encoder zero position index
    int jStart = 0;
    int stepNo = 0;

    int fullStepReadings[MOTOR_SPR];

    //int fullStep = 0;
    int ticks = 0;
    //float lookupAngle = 0.0;
    puts("motor_calibrate: Beginning calibration routine ... \n\r");

    encoderReading = _motor_read_raw(motorID);
    int dir = MOTOR_REVERSE;
    motor_one_step(motorID, dir);
    delay_ms(500);

    if ((_motor_read_raw(motorID) - encoderReading) > 0)   //check which way motor moves when dir = true
    {
        puts("Wired backwards \n\r");    // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...
        return;
    }

    while (_motors[motorID].stepNumber != 0) {       //go to step zero
    if (_motors[motorID].stepNumber > 0) {
        dir = MOTOR_FORWARD;
    }
    else
    {
        dir = MOTOR_REVERSE;
    }
        motor_one_step(motorID, dir);
        delay_ms(100);
    }
    dir = MOTOR_FORWARD;
    for (int x = 0; x < MOTOR_SPR; x++) {     //step through all full step positions, recording their encoder readings

    encoderReading = 0;
    delay_ms(50);                         //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
    lastencoderReading = _motor_read_raw(motorID);
        
    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
        currentencoderReading = _motor_read_raw(motorID);

        if ((currentencoderReading-lastencoderReading)<(-(cpr/2))){
        currentencoderReading += cpr;
        }
        else if ((currentencoderReading-lastencoderReading)>((cpr/2))){
        currentencoderReading -= cpr;
        }

        encoderReading += currentencoderReading;
        delay_ms(10);
        lastencoderReading = currentencoderReading;
    }
    encoderReading = encoderReading / avg;
    if (encoderReading>cpr){
        encoderReading-= cpr;
    }
    else if (encoderReading<0){
        encoderReading+= cpr;
    }

    fullStepReadings[x] = encoderReading;

    //static int lines = 0;
    //debug:
    //printf("%d, ", fullStepReadings[x]);
    //lines++;
    //if (lines % 10 == 0)
    //    printf("\n\r");
    // SerialUSB.println(fullStepReadings[x], DEC);      //print readings as a sanity check
    
    if (x % 20 == 0)
    {
        printf("\n\r DBG: %d \n\r", 100*x/MOTOR_SPR);
        
    } else {
        printf(".");
    }

        motor_one_step(motorID, dir);
    }
        puts("\n\r");


    for (int i = 0; i < MOTOR_SPR; i++) {
        ticks = fullStepReadings[mod((i + 1), MOTOR_SPR)] - fullStepReadings[mod((i), MOTOR_SPR)];
        if (ticks < -CPR_THRESHOLD) {
            ticks += cpr;

        }
        else if (ticks > CPR_THRESHOLD) {
            ticks -= cpr;
        }
        // SerialUSB.println(ticks);

        if (ticks > 1) {                                    //note starting point with iStart,jStart
            for (int j = 0; j < ticks; j++) {
            stepNo = (mod(fullStepReadings[i] + j, cpr));
            // SerialUSB.println(stepNo);
            if (stepNo == 0) {
                iStart = i;
                jStart = j;
            }

            }
        }

        if (ticks < 1) {                                    //note starting point with iStart,jStart
            for (int j = -ticks; j > 0; j--) {
            stepNo = (mod(fullStepReadings[MOTOR_SPR - 1 - i] + j, cpr));
            // SerialUSB.println(stepNo);
            if (stepNo == 0) {
                iStart = i;
                jStart = j;
            }

            }
        }

    }

    // The code below generates the lookup table by intepolating between
    // full steps and mapping each encoder count to a calibrated angle
    // The lookup table is too big to store in volatile memory,
    // so we must generate and store it into the flash on the fly

    // begin the write to the calibration table
    page_count = 0;
    page_ptr = (const uint8_t*) lookup[motorID];
    puts("Writing to flash 0x \n\r");
    printf("%p ", page_ptr);

    for (int i = iStart; i < (iStart + MOTOR_SPR + 1); i++) {
        ticks = fullStepReadings[mod((i + 1), MOTOR_SPR)] - fullStepReadings[mod((i), MOTOR_SPR)];

        if (ticks < -CPR_THRESHOLD) {           //check if current interval wraps over encoder's zero positon
            ticks += cpr;
        }
        else if (ticks > CPR_THRESHOLD) {
            ticks -= cpr;
        }
        //Here we print an interpolated angle corresponding to each encoder count (in order)
        if (ticks > 1) {              //if encoder counts were increasing during cal routine...

            if (i == iStart) { //this is an edge case
                for (int j = jStart; j < ticks; j++) {
                    store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / (float)(ticks))), 360000.0));
                }
            }

            else if (i == (iStart + MOTOR_SPR)) { //this is an edge case
                for (int j = 0; j < jStart; j++) {
                    store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / (float)(ticks))), 360000.0));
                }
            }
            else {                        //this is the general case
                for (int j = 0; j < ticks; j++) {
                    store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / (float)(ticks))), 360000.0));
                }
            }
        }

        else if (ticks < 1) {             //similar to above... for case when encoder counts were decreasing during cal routine
            if (i == iStart) {
                for (int j = - ticks; j > (jStart); j--) {
                    store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / (float)(ticks))), 360000.0));
                }
            }
            else if (i == iStart + MOTOR_SPR) {
                for (int j = jStart; j > 0; j--) {
                    store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / (float)(ticks))), 360000.0));
                }
            }
            else {
                for (int j = - ticks; j > 0; j--) {
                    store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / (float)(ticks))), 360000.0));
                }
            }

        }
    }

    puts("Calibration complete! \n\r");
    //update initial angle
    _motors[motorID].init_angle = motor_read_angle(motorID);
    puts("The calibration table has been written to non-volatile Flash memory! \n\r");
    page_number = 0;

}

//motor MAIN task, shall be called in loop

void motor_set_rpm(uMotorID motorID, uint8_t dir, uint32_t RPM)
{

    if((MOTOR_STATUS_UNKNOWN == _motors[motorID].state) ||  (_motors[motorID].state == MOTOR_STATUS_ERROR))
    {   
        printf("motor_calibrate: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }

    //calculate desired microstep delay
    int microstep_wait = 0;

    microstep_wait = MINUTE_TO_US / (RPM * MOTOR_MICROSTEP_CONFIG * MOTOR_SPR);
    //printf("motor_set_rpm: desired RPM: %d, calulated value microstep us: %d \n\r", RPM, microstep_wait);

    _motors[motorID].dir = dir;
    _motors[motorID].RPM = RPM;
    _motors[motorID].us_per_microstep = microstep_wait;

    //_motor_micro_step( motorID, _motors[motorID].dir, 0x1fff, RPM);
    //user motor task instead
    _motors[motorID].motor_position_needs_update = 1;

}

void motor_set_power(uMotorID motorID, float power, unsigned char motor_config)
{
    int ret = 0;
    if((MOTOR_STATUS_UNKNOWN == _motors[motorID].state) ||  (_motors[motorID].state == MOTOR_STATUS_ERROR))
    {   
        printf("motor_set_power: motor %d is not online, state= %d \n\r", motorID,  _motors[motorID].state);
        return;
    }
    ret = _motor_set_power(motorID, power, motor_config);
    if(0 == ret)
    {
        _motors[motorID].power = power;
    }
  
}

void motor_set_dir(uMotorID motorID, uint8_t dir)
{
    if((MOTOR_STATUS_UNKNOWN == _motors[motorID].state) ||  (_motors[motorID].state == MOTOR_STATUS_ERROR))
    {   
        printf("motor_calibrate: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }

    _motors[motorID].dir = dir;

}

int task_count = 0;
char str[5];
float tmp_angle = 0;
int dir_sign = 0;


float delta_abs = 0;
int steps_to_do = 0;

int soft_start = 0;
int soft_stop = 0;

#define RPM_MAX 3
int rpm_count = 0;
int rpm_soft_start[RPM_MAX] = {20, 60, 90};
int rpm_soft_start_diff[RPM_MAX] = {10, 40, 70};


void motor_soft_start()
{
    soft_start = 1;
    soft_stop = 0;
}

void motor_soft_stop()
{
    soft_start = 0;
    soft_stop = 1;
}

void motor_task()
{

    for(int i = MOTOR_0; i < MOTOR_COUNT; i+=2) //filter only dir motors
    {
        if((_motors[i].state != MOTOR_STATUS_ERROR) && (_motors[i].state != MOTOR_STATUS_UNKNOWN))
        {
            //read current angle
            _motors[i].angle = motor_read_angle(i);
            tmp_angle = (_motors[i].init_angle - _motors[i].angle);
            if(tmp_angle < 0)
                dir_sign = 1;
            if( abs(tmp_angle) > 180.0 )
            {
                //wrap arround, taking this into account
                tmp_angle = 360.0 - abs(tmp_angle);
                //if(_motors[i].dir == MOTOR_FORWARD)
                if(dir_sign == 0)
                    _motors[i].rotations++;
                else
                    _motors[i].rotations--;
            }
            _motors[i].angular_speed = abs(tmp_angle) / 0.05; //50ms task, deg/second
            _motors[i].calculated_rpm = _motors[i].angular_speed * 0.1666; //calculate actual RPM
            _motors[i].init_angle =  _motors[i].angle;
            dir_sign = 0;
        }

        if(_motors[i].motor_position_needs_update)
        {
            //calculate next position
            delta_abs = motor_get_abs(i) - _motors[i].angle_absolute_set;

            steps_to_do = (unsigned int)(abs(delta_abs) / angle_step_conv * _motors[i].gearbox);
            if(delta_abs <= 0){
                _motor_micro_step(i, MOTOR_REVERSE, steps_to_do, 100);
                //printf("1\n\r");
            }
            else
            {
                _motor_micro_step(i, MOTOR_FORWARD, steps_to_do, 100);
                //printf("2\n\r");
            }
            _motors[i].motor_position_needs_update = 0;
        }
    }
    // 100ms
    if( (task_count % 2) == 0)
    {
        if(soft_start)
        {
            if(rpm_count < RPM_MAX)
            {
                printf("Do soft start %d \n\r", rpm_soft_start[rpm_count]);
                if(motors_turning_position > 0)
                {
                    _motor_micro_step( 5, _motors[5].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( 1, _motors[1].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( 7, _motors[7].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 3, _motors[3].dir, 0x1fff, rpm_soft_start[rpm_count]);
                }
                else if(motors_turning_position < 0)
                {
                    _motor_micro_step( 5, _motors[5].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 1, _motors[1].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 7, _motors[7].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( 3, _motors[3].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);           
                }
                else
                {
                    _motor_micro_step( 5, _motors[5].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 1, _motors[1].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 7, _motors[7].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 3, _motors[3].dir, 0x1fff, rpm_soft_start[rpm_count]);
                }
                rpm_count++;
            }
            else
            {
                _motor_micro_step( 5, _motors[5].dir, 0x1fff, _motors[5].RPM);
                _motor_micro_step( 1, _motors[1].dir, 0x1fff, _motors[1].RPM);
                _motor_micro_step( 7, _motors[7].dir, 0x1fff, _motors[7].RPM);
                _motor_micro_step( 3, _motors[3].dir, 0x1fff, _motors[3].RPM);

                printf("full speed \n\r");
                rpm_count = RPM_MAX -1;
                soft_start = 0;
            }
            
        }
        else if(soft_stop)
        {
            if(rpm_count >= 0)
            {
                printf("Do soft stop %d \n\r", rpm_soft_start[rpm_count]);
                if(motors_turning_position > 0)
                {
                    _motor_micro_step( 5, _motors[5].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( 1, _motors[1].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( 7, _motors[7].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 3, _motors[3].dir, 0x1fff, rpm_soft_start[rpm_count]);
                }
                else if(motors_turning_position < 0)
                {
                    _motor_micro_step( 5, _motors[5].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 1, _motors[1].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 7, _motors[7].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( 3, _motors[3].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);           
                }
                else
                {
                    _motor_micro_step( 5, _motors[5].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 1, _motors[1].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 7, _motors[7].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( 3, _motors[3].dir, 0x1fff, rpm_soft_start[rpm_count]);
                }
                rpm_count--;
            }
            else
            {
                printf("stop motors \n\r");
                _motor_micro_step( 5, _motors[5].dir, 0, 0);
                _motor_micro_step( 1, _motors[1].dir, 0, 0);
                _motor_micro_step( 7, _motors[7].dir, 0, 0);
                _motor_micro_step( 3, _motors[3].dir, 0, 0);
                rpm_count = 0;
                soft_stop = 0;
            }
        }
    }
    // 1s
    if( (task_count % 20) == 0)
    {
        //printout stats for motors
        for(int i = MOTOR_0; i < MOTOR_COUNT; i++)
        {
            if((_motors[i].state != MOTOR_STATUS_ERROR) && (_motors[i].state != MOTOR_STATUS_UNKNOWN))
            {
               // snprintf(str, sizeof(str), "%f", _motors[i].angular_speed);
               // printf("ID: %d, as:%s", i, str);
               // snprintf(str, sizeof(str), "%f", _motors[i].calculated_rpm);
               // printf(", rpm:%s", str);
               // printf(", rot:%ld \n\r",  _motors[i].rotations);
            }
        }
    }

    task_count++;

}