#include "motor_calibration.h"
#include "motor.h"
#include "zyra_spi.h"
#include "delay.h"
#include "nvm.h"
#include "string.h"

#define MOTOR_SYNC_CMD  0xc0
#define MOTOR_STEP_CMD  0x40
#define MOTOR_SETUP_CMD 0x80
#define MOTOR_SYNC_ACK  0xAA


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


int turning_speed_red[MOTORS_TURNING_POS] = {0, 15, 30, 45, 60};
int motors_turning_position = 0;

const float __attribute__((__aligned__(512))) lookup[MOTOR_COUNT][MAGNETIC_LUT_SIZE] = {
//Put lookup table here!
};

static inline int  _motor_sync(uMotorID motorID, uint8_t PCs);
static inline void  _motor_diag(uint8_t PCs);

sMotorInstance _motors[MOTOR_COUNT];


static inline int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}

static inline void _motor_diag(uint8_t PCs)
{
	long angleTemp = 0;
	uint16_t data = 0;
	uint8_t b1 = 0, b2 = 0;

	puts("motor_diag: Checking AS5047 diagnostic and error registers ... \n\r");

	spi_sync_transfer(0xFF, PCs, 0);
	spi_sync_transfer(0xFC, PCs, 1);

	delay_ms(1);

	b1 = spi_sync_transfer(0xC0, PCs, 0);
	b2 = spi_sync_transfer(0x00, PCs, 1);

	data = ((b1 << 8) | b2);

	puts("motor_diag: Check DIAAGC register ...  n\r\r");

	angleTemp = (data & 0xFFFF);
	printf("%ld BIN n\r", (angleTemp | 0B1110000000000000000 ));

  	if (angleTemp & (1 << 14))    puts("  Error occurred  \n\r");

  	if (angleTemp & (1 << 11))    puts("  MAGH - magnetic field strength too high, set if AGC = 0x00. This indicates the non-linearity error may be increased \n\r");

  	if (angleTemp & (1 << 10))    puts("  MAGL - magnetic field strength too low, set if AGC = 0xFF. This indicates the output noise of the measured angle may be increased\n\r");

  	if (angleTemp & (1 << 9))     puts("  COF - CORDIC overflow. This indicates the measured angle is not reliable\n\r");

  	if (angleTemp & (1 << 8))     puts("  LF - offset compensation completed. At power-up, an internal offset compensation procedure is started, and this bit is set when the procedure is completed\n\r");

  	if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 11)) | (angleTemp & (1 << 10)) | (angleTemp & (1 << 9))))  puts("Looks good!\n\r");
  	puts("\n\r");
  	delay_ms(1);

	spi_sync_transfer(0x40, PCs, 0);
	spi_sync_transfer(0x01, PCs, 1);


  	delay_ms(1);

	b1 = spi_sync_transfer(0xC0, PCs ,0);
	b2 = spi_sync_transfer(0x00, PCs ,1);
	data = ((b1 << 8) | b2);

  	puts("motor_diag: Check ERRFL register ...  \n\r\r");


  	angleTemp = (data & 0xFFFF);
  	printf("%ld BIN\n\r", (angleTemp | 0B1110000000000000000 ));

  	if (angleTemp & (1 << 14)) {
    	puts("  Error occurred  \n\r");
  	}
  	if (angleTemp & (1 << 2)) {
    	puts("  parity error \n\r");
  	}
  	if (angleTemp & (1 << 1)) {
    	puts("  invalid register  \n\r");
  	}
  	if (angleTemp & (1 << 0)) {
    	puts("  framing error  \n\r");
  	}
  	if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 2)) | (angleTemp & (1 << 1)) | (angleTemp & (1 << 0))))  puts("Looks good! \n\r");

  	puts("\n\r");

  	delay_ms(1);
}

static inline int  _motor_sync(uMotorID motorID, uint8_t PCs)
{

	uint8_t response = 0;
    uint8_t select_slave = 0;
    uint8_t in_data[4];
    uint8_t data[5];
    int tryout = 5;

    data[0] = MOTOR_SYNC_CMD;
    data[1] = 0;
    data[2] = 0;
    //calculate checksum
    data[3] = data[0] + data[1] + data[2];
    data[4] = 0xff;

    spi_sync_transfer(data[0], PCs, 0);
	in_data[0] = spi_sync_transfer(data[1], PCs, 0);
    in_data[1] = spi_sync_transfer(data[2], PCs, 0);
    in_data[2] = spi_sync_transfer(data[3], PCs, 0);
    in_data[3] = spi_sync_transfer(data[4], PCs, 1);

    printf("Ret: %x%x%x%x \n\r", in_data[0], in_data[1], in_data[2], in_data[3]);

    if(in_data[3] != 0xAA)
    {
        while(tryout--)
        {
            spi_sync_transfer(data[0], PCs, 0);
            in_data[0] = spi_sync_transfer(data[1], PCs, 0);
            in_data[1] = spi_sync_transfer(data[2], PCs, 0);
            in_data[2] = spi_sync_transfer(data[3], PCs, 0);
            in_data[3] = spi_sync_transfer(data[4], PCs, 1);

            printf(">Ret: %x%x%x%x \n\r", in_data[0], in_data[1], in_data[2], in_data[3]);

            if(in_data[3] == 0xAA)
            {
                tryout = 0;
                break;
            }
        }

        if(in_data[3] != 0xAA)
        {
            printf("_motor_sync: ID: %d failed\n\r", motorID);
            return 1;
        }

    }

    return 0;
}

static inline uint16_t _motor_micro_step( uint8_t PCs, uint8_t dir, uint16_t steps, uint8_t rpm)
{
    uint8_t in_data[4];
	uint8_t data[5];
    int tryout = 5;

    data[0] = MOTOR_STEP_CMD | (dir << 5) | ((steps & 0x1fff) >> 8);
    data[1] = (steps & 0xff);
    data[2] = rpm;
    //calculate checksum
    data[3] = data[0] + data[1] + data[2];
    data[4] = 0xff;

    spi_sync_transfer(data[0], PCs, 0);
	in_data[0] = spi_sync_transfer(data[1], PCs, 0);
    in_data[1] = spi_sync_transfer(data[2], PCs, 0);
    in_data[2] = spi_sync_transfer(data[3], PCs, 0);
    in_data[3] = spi_sync_transfer(data[4], PCs, 0);
    

    printf("Ret: %x%x%x%x\n\r", in_data[0], in_data[1],in_data[2], in_data[3]);
    
    if(in_data[3] != 0x77)
    {
        while(tryout--)
        {
            spi_sync_transfer(data[0], PCs, 0);
            in_data[0] = spi_sync_transfer(data[1], PCs, 0);
            in_data[1] = spi_sync_transfer(data[2], PCs, 0);
            in_data[2] = spi_sync_transfer(data[3], PCs, 0);
            in_data[3] = spi_sync_transfer(data[4], PCs, 0);
            

            printf(">Ret: %x%x%x%x\n\r", in_data[0], in_data[1],in_data[2], in_data[3]);

            if(in_data[3] == 0x77)
            {
                tryout = 0;
                break;
            }
        }

        if(in_data[3] != 0x77)
        {
            printf("_motor_microstep: failed\n\r");
            return 1;
        }

    }
    
    return 0;

}

static inline int _motor_read_raw(uint8_t PCs)
{
    uint16_t angleTemp;
    uint8_t b1 = 0, b2 = 0;

    b1 = spi_sync_transfer(0xFF, PCs, 0);
    b2 = spi_sync_transfer(0xFF, PCs, 1);

    angleTemp = (((b1 << 8) | b2) );

    if(angleTemp & (1 << 14))
        printf("Angle error! \n\r");

    angleTemp &= 0B0011111111111111;


    return angleTemp >> MAGNETIC_REDUCE_RESOLUTION;
}


void motor_init(uMotorID motorID, uint8_t motorPCs, uint8_t sensorPCs, uint8_t go_zero, float gearbox)
{
    printf("motor_init: Initialize motor ID=%d, PCs selected driver %d senzor %d... \n\r", motorID, motorPCs, sensorPCs);

    _motors[motorID].state = STATE_MOTOR_UNKNOWN;
    _motors[motorID].motorPCs = motorPCs;
    _motors[motorID].sensorPCs = sensorPCs;
    _motors[motorID].motorID = motorID;
    _motors[motorID].us_per_microstep = 0;
    _motors[motorID].RPM = 0;
    _motors[motorID].gearbox = gearbox;
    _motors[motorID].idle = 1;

    printf("motor_init: sync with motor driver, sending... \n\r");


    if(_motor_sync(motorID, _motors[motorID].motorPCs))
    {
        _motors[motorID].state = STATE_MOTOR_ERROR;
        printf("motor_init: sync error \n\r");
    }
    else
    {
        _motors[motorID].state = STATE_MOTOR_OK;
        char str[7];
        //get initial senzor data
        _motors[motorID].init_pos = _motor_read_raw(_motors[motorID].motorPCs = motorPCs);
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
                    _motor_micro_step(_motors[motorID].motorPCs, MOTOR_FORWARD, steps_to_zero, 50);
                    printf("1\n\r");
                }
                else
                {
                     _motor_micro_step(_motors[motorID].motorPCs, MOTOR_REVERSE, steps_to_zero, 50);
                     printf("2\n\r");
                }
            }
            else
            {
                steps_to_zero = (unsigned int)( (360.0 - abs(delta_angle)) / angle_step_conv);
                if(delta_angle <= 0)
                {
                    _motor_micro_step(_motors[motorID].motorPCs, MOTOR_REVERSE, steps_to_zero, 50);
                    printf("3\n\r");
                }
                else
                {
                     _motor_micro_step(_motors[motorID].motorPCs, MOTOR_FORWARD, steps_to_zero, 50);
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
   // if(_motors[motorID].dir == MOTOR_FORWARD)
   // {
  //      sign = 1.0;   
  //  }
    //char str[8];
	//snprintf(str, sizeof(str), "%f", _motors[motorID].init_angle);
	//printf("dbg Angle %s \n\r",str);
    float abs_pos = (_motors[motorID].init_angle + ( (float)_motors[motorID].rotations * 360.0) + ( _motors[motorID].angle_zero_set) * sign) / _motors[motorID].gearbox; 
    return abs_pos;
}

sMotorState motor_get_status(uMotorID motorID)
{
    return _motors[motorID].state;
}

void motor_microstep(uMotorID motorID, uint8_t dir, uint16_t steps, uint8_t rpm)
{
    uint16_t ret = 0;
    uint16_t tryout = 3;
    _motors[motorID].dir = dir;

    ret = _motor_micro_step(_motors[motorID].motorPCs, dir, steps, rpm);
    if(ret)
        printf("motor_microstep: ID: %d failed \n\r", motorID);
}

void motor_one_step(uMotorID motorID, uint8_t dir)
{
        _motors[motorID].dir = dir;
        _motor_micro_step(_motors[motorID].motorPCs, dir, 16, 150);
        delay_us(187 * MOTOR_MICROSTEP_WAIT_US);
}

float motor_read_angle(uMotorID motorID)
{
    if(STATE_MOTOR_OK != _motors[motorID].state)
    {   
        printf("motor_read_angle: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    const int avg = 1;
    int encoderReading = 0;

    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
        encoderReading += _motor_read_raw(_motors[motorID].sensorPCs);
    }

    return encoderReading * (360.0 / 4096) / avg;
    //TODO: fix calibration, until then use direct data
    //return lookup[motorID][encoderReading / avg];

}

int motor_read_position(uMotorID motorID)
{
    if(STATE_MOTOR_OK != _motors[motorID].state)
    {   
        printf("motor_read_position: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    return _motor_read_raw(_motors[motorID].sensorPCs);
}

void motor_sync(uMotorID motorID)
{
    if(STATE_MOTOR_OK != _motors[motorID].state)
    {   
        printf("motor_sync: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    _motor_sync(motorID ,_motors[motorID].motorPCs);
}

void motor_diagnoise(uMotorID motorID)
{
    if(STATE_MOTOR_OK != _motors[motorID].state)
    {   
        printf("motor_diagnoise: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    _motor_diag(_motors[motorID].sensorPCs);
}

void motor_printout(uMotorID motorID)
{
    if(STATE_MOTOR_OK != _motors[motorID].state)
    {   printf("motor_printout: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    puts("motor_printout: print calibration data ... \n\r");
    for(int i = 0; i < MAGNETIC_LUT_SIZE; i++)
    {
        char str[8];
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

    if(STATE_MOTOR_OK != _motors[motorID].state)
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

    int fullStep = 0;
    int ticks = 0;
    float lookupAngle = 0.0;
    puts("motor_calibrate: Beginning calibration routine ... \n\r");

    encoderReading = _motor_read_raw(_motors[motorID].sensorPCs);
    int dir = MOTOR_REVERSE;
    motor_one_step(motorID, dir);
    delay_ms(500);

    if ((_motor_read_raw(_motors[motorID].sensorPCs) - encoderReading) < 0)   //check which way motor moves when dir = true
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
    lastencoderReading = _motor_read_raw(_motors[motorID].sensorPCs);
        
    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
        currentencoderReading = _motor_read_raw(_motors[motorID].sensorPCs);

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
    printf("%x ", page_ptr);

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

    if(STATE_MOTOR_OK != _motors[motorID].state)
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

    //_motor_micro_step( _motors[motorID].motorPCs, _motors[motorID].dir, 0x1fff, RPM);
    //user motor task instead
    _motors[motorID].motor_position_needs_update = 1;

}

void motor_set_power(uMotorID motorID, float power, unsigned char motor_config)
{

    if(STATE_MOTOR_OK != _motors[motorID].state)
    {   
        printf("motor_set_power: motor %d is not online, state= %d \n\r", motorID,  _motors[motorID].state);
        return;
    }

    uint8_t data[5];
    uint8_t in_data[5];
    const float iMAX = 1.0;
    const float rSense = 0.150;
    int uMAX = (255/3.3)*(iMAX*10*rSense);
    int uPower = (255/3.3)*(power*10*rSense);
    int tryout = 5;

    if(uPower < uMAX)
    {
        printf("motor_set_power: ID: %d Calculated uPower %d, sending... \n\r", motorID, uPower);


        data[0] = MOTOR_SETUP_CMD;
        data[1] = uPower;
        data[2] = motor_config; //TBD
        data[3] = data[0] + data[1] + data[2];
        data[4] = 0xff;

        spi_sync_transfer(data[0], _motors[motorID].motorPCs, 0);
        in_data[0] = spi_sync_transfer(data[1], _motors[motorID].motorPCs, 0);
        in_data[1] = spi_sync_transfer(data[2], _motors[motorID].motorPCs, 0);
        in_data[2] = spi_sync_transfer(data[3], _motors[motorID].motorPCs, 0);
        in_data[3] = spi_sync_transfer(0xFF, _motors[motorID].motorPCs, 1);
        
        printf("Ret: %x%x%x%x\n\r", in_data[0], in_data[1], in_data[2], in_data[3]);

        if(in_data[3] != 0x78)
        {
            while(tryout--)
            {
                spi_sync_transfer(data[0], _motors[motorID].motorPCs, 0);
                in_data[0] = spi_sync_transfer(data[1], _motors[motorID].motorPCs, 0);
                in_data[1] = spi_sync_transfer(data[2], _motors[motorID].motorPCs, 0);
                in_data[2] = spi_sync_transfer(data[3], _motors[motorID].motorPCs, 0);
                in_data[3] = spi_sync_transfer(0xFF, _motors[motorID].motorPCs, 1);
                
                printf(">Ret: %x%x%x%x\n\r", in_data[0], in_data[1], in_data[2], in_data[3]);

                if(in_data[3] == 0x78)
                {
                    tryout = 0;
                    _motors[motorID].power = power;
                    break;
                }
            }

            if(in_data[3] != 0x78)
            {
                printf("motor_set_power: ID %d failed\n\r", motorID);
            }

        }
    }
    else
    {
        printf("motor_set_power: Calculated uPower %d is bigger than uMAX: %d, ignoring... \n\r", uPower, uMAX);
    }
}

void motor_set_dir(uMotorID motorID, uint8_t dir)
{
    if(STATE_MOTOR_OK != _motors[motorID].state)
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
        if(_motors[i].state == STATE_MOTOR_OK)
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
                _motor_micro_step(_motors[i].motorPCs, MOTOR_REVERSE, steps_to_do, 100);
                //printf("1\n\r");
            }
            else
            {
                _motor_micro_step(_motors[i].motorPCs, MOTOR_FORWARD, steps_to_do, 100);
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
                    _motor_micro_step( _motors[5].motorPCs, _motors[5].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( _motors[1].motorPCs, _motors[1].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( _motors[7].motorPCs, _motors[7].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[3].motorPCs, _motors[3].dir, 0x1fff, rpm_soft_start[rpm_count]);
                }
                else if(motors_turning_position < 0)
                {
                    _motor_micro_step( _motors[5].motorPCs, _motors[5].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[1].motorPCs, _motors[1].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[7].motorPCs, _motors[7].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( _motors[3].motorPCs, _motors[3].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);           
                }
                else
                {
                    _motor_micro_step( _motors[5].motorPCs, _motors[5].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[1].motorPCs, _motors[1].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[7].motorPCs, _motors[7].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[3].motorPCs, _motors[3].dir, 0x1fff, rpm_soft_start[rpm_count]);
                }
                rpm_count++;
            }
            else
            {
                _motor_micro_step( _motors[5].motorPCs, _motors[5].dir, 0x1fff, _motors[5].RPM);
                _motor_micro_step( _motors[1].motorPCs, _motors[1].dir, 0x1fff, _motors[1].RPM);
                _motor_micro_step( _motors[7].motorPCs, _motors[7].dir, 0x1fff, _motors[7].RPM);
                _motor_micro_step( _motors[3].motorPCs, _motors[3].dir, 0x1fff, _motors[3].RPM);

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
                    _motor_micro_step( _motors[5].motorPCs, _motors[5].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( _motors[1].motorPCs, _motors[1].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( _motors[7].motorPCs, _motors[7].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[3].motorPCs, _motors[3].dir, 0x1fff, rpm_soft_start[rpm_count]);
                }
                else if(motors_turning_position < 0)
                {
                    _motor_micro_step( _motors[5].motorPCs, _motors[5].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[1].motorPCs, _motors[1].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[7].motorPCs, _motors[7].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);
                    _motor_micro_step( _motors[3].motorPCs, _motors[3].dir, 0x1fff, rpm_soft_start_diff[rpm_count]);           
                }
                else
                {
                    _motor_micro_step( _motors[5].motorPCs, _motors[5].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[1].motorPCs, _motors[1].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[7].motorPCs, _motors[7].dir, 0x1fff, rpm_soft_start[rpm_count]);
                    _motor_micro_step( _motors[3].motorPCs, _motors[3].dir, 0x1fff, rpm_soft_start[rpm_count]);
                }
                rpm_count--;
            }
            else
            {
                printf("stop motors \n\r");
                _motor_micro_step( _motors[5].motorPCs, _motors[5].dir, 0, 0);
                _motor_micro_step( _motors[1].motorPCs, _motors[1].dir, 0, 0);
                _motor_micro_step( _motors[7].motorPCs, _motors[7].dir, 0, 0);
                _motor_micro_step( _motors[3].motorPCs, _motors[3].dir, 0, 0);
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
            if(_motors[i].state == STATE_MOTOR_OK)
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