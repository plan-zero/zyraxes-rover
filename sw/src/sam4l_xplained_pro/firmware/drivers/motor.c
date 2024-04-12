#include "motor_calibration.h"
#include "motor.h"
#include "zyra_spi.h"
#include "delay.h"
#include "nvm.h"
#include "string.h"

#define MOTOR_SYNC_CMD  0xc0
#define MOTOR_SYNC_ACK  0xAA

#define BROADCAST_NO_CS 0

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

	spi_sync_transfer(0x40, SPI_CHIP_PCS_0, 0);
	spi_sync_transfer(0x01, SPI_CHIP_PCS_0, 1);


  	delay_ms(1);

	b1 = spi_sync_transfer(0xC0, SPI_CHIP_PCS_0 ,0);
	b2 = spi_sync_transfer(0x00, SPI_CHIP_PCS_0 ,1);
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

    if(_motors[motorID].motorUsePCs == 0)
    {
        //select slave
        select_slave |= 1 << motorID;
        response = spi_sync_transfer(select_slave, PCs, 1);
        printf("motor_sync_driver: send 0x%x response: 0x%x\n\r", select_slave, response);
    }

	response = spi_sync_transfer(MOTOR_SYNC_CMD, PCs, 1);
	printf("motor_sync_driver: send 0x%x response: 0x%x\n\r", MOTOR_SYNC_CMD, response);

    return response;
}

static inline void _motor_micro_step( uint8_t PCs, uint8_t dir)
{

	uint8_t data = 0;

	data = spi_sync_transfer(dir, PCs, 1);

	UNUSED(data);

}

static inline int _motor_read_raw(uint8_t PCs)
{
    long angleTemp;
    uint8_t b1 = 0, b2 = 0;

    b1 = spi_sync_transfer(0xFF, PCs, 0);
    b2 = spi_sync_transfer(0xFF, PCs, 1);

    angleTemp = (((b1 << 8) | b2) & 0B0011111111111111);

    return angleTemp >> MAGNETIC_REDUCE_RESOLUTION;
}


void motor_init(uMotorID motorID, uint8_t motorPCs, uint8_t sensorPCs, uint8_t motorUsePCs)
{
    static int tryout = 3;
    int sync_ack = 0;
    printf("motor_init: Initialize motor ID=%d, PCs selected driver %d senzor %d... \n\r", motorID, motorPCs, sensorPCs);

    _motors[motorID].state = STATE_MOTOR_UNKNOWN;
    _motors[motorID].motorPCs = motorPCs;
    _motors[motorID].sensorPCs = sensorPCs;
    _motors[motorID].motorID = motorID;
    _motors[motorID].motorUsePCs = motorUsePCs;

    printf("motor_init: sync with motor driver, sending... \n\r");

    tryout = 3;
    while(tryout)
    {
        sync_ack = _motor_sync(motorID, _motors[motorID].motorPCs);
        //wait for atiny24 to startup, should take at least 64ms
        delay_ms(100);
        if(sync_ack == MOTOR_SYNC_ACK)
        {
            printf("motor_init: sync complete \n\r");
            _motors[motorID].state = STATE_MOTOR_OK;
            break;
        }
        tryout--;
    }

    if(sync_ack != MOTOR_SYNC_ACK)
    {
        _motors[motorID].state = STATE_MOTOR_ERROR;
        printf("motor_init: sync error \n\r");
    }

}

sMotorState motor_get_status(uMotorID motorID)
{
    return _motors[motorID].state;
}

void motor_microstep(uMotorID motorID, uint8_t dir)
{
    //don't check status as this should be as quick as possible, motor status is checked at a higher level
    if(_motors[motorID].motorUsePCs == 0)
    {
        uint8_t select_slave = 0;
        select_slave |= 1 << motorID;
        spi_sync_transfer(select_slave, _motors[motorID].motorPCs, 1);
    }
    _motor_micro_step(_motors[motorID].motorPCs, dir);
}

void motor_one_step(uMotorID motorID, uint8_t dir)
{
    uint8_t select_slave = 0;
    if(STATE_MOTOR_OK != _motors[motorID].state)
    {   
        printf("motor_one_step: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }

    for(int i = 0; i < MOTOR_MICROSTEP_CONFIG; i++)
    {
        if(_motors[motorID].motorUsePCs == 0)
        {
            
            select_slave |= 1 << motorID;
            spi_sync_transfer(select_slave, _motors[motorID].motorPCs, 1);
        }
        _motor_micro_step(_motors[motorID].motorPCs, dir);
        delay_us(MOTOR_MICROSTEP_WAIT_US);
    }
}

float motor_read_angle(uMotorID motorID)
{
    if(STATE_MOTOR_OK != _motors[motorID].state)
    {   
        printf("motor_read_angle: motor is not online, state= %d \n\r",  _motors[motorID].state);
        return;
    }
    const int avg = 3;
    int encoderReading = 0;

    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
        encoderReading += _motor_read_raw(_motors[motorID].sensorPCs);
        delay_us(10);
    }

    //return encoderReading * (360.0 / 16384.0) / avg;
    return lookup[motorID][encoderReading / avg];

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
    delay_ms(20);                         //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
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
    puts("The calibration table has been written to non-volatile Flash memory! \n\r");
    page_number = 0;

}

//motor MAIN task, shall be called in loop

int motor_set_rpm(uMotorID motorID, uint32_t RPM)
{
    //calculate desired microstep delay
    int microstep_wait = 0;

    microstep_wait = MINUTE_TO_US / (RPM * MOTOR_MICROSTEP_CONFIG * MOTOR_SPR);
    printf("motor_set_rpm: desired RPM: %d, calulated value microstep us: %d \n\r", RPM, microstep_wait);
    //set the threshold
    if(microstep_wait < MOTOR_MICROSTEP_WAIT_US)
    {
        microstep_wait = MOTOR_MICROSTEP_WAIT_US;
    }

    //set calculated values
    _motors[motorID].RPM = RPM;
    _motors[motorID].us_per_microstep = microstep_wait;

    return microstep_wait;
}

uint8_t broadcast = 0;
void motor_task()
{
    //printf("Motor thread \n\r");
    //selecting motors
    //test M0 and M2
    broadcast |= (1 << MOTOR_0) | (1 << MOTOR_2); 
    spi_sync_transfer(broadcast, BROADCAST_NO_CS, 1);
    _motor_micro_step(BROADCAST_NO_CS, MOTOR_FORWARD);
}