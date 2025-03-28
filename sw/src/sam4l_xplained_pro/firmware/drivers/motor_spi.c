#include "zyra_spi.h"
#include "delay.h"
#include "motor_spi.h"

#define MOTOR_SYNC_CMD  0xc0
#define MOTOR_STEP_CMD  0x40
#define MOTOR_SETUP_CMD 0x80
#define MOTOR_SYNC_ACK  0xAA



void motor_diag_spi(uint8_t PCs)
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


int  motor_sync_spi(uint8_t PCs)
{

	//uint8_t response = 0;
    //uint8_t select_slave = 0;
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
            printf("_motor_sync: failed\n\r");
            return 1;
        }

    }

    return 0;
}


uint16_t motor_micro_step_spi( uint8_t PCs, uint8_t dir, uint16_t steps, uint8_t rpm)
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


int motor_read_raw_spi(uint8_t PCs)
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


int motor_set_power_spi(uint8_t PCs, float power, unsigned char motor_config)
{
    uint8_t data[5];
    uint8_t in_data[5];
    const float iMAX = 1.0;
    const float rSense = 0.150;
    int uMAX = (255/3.3)*(iMAX*10*rSense);
    int uPower = (255/3.3)*(power*10*rSense);
    int tryout = 5;

    if(uPower < uMAX)
    {
        printf("motor_set_power: Calculated uPower %d, sending... \n\r", uPower);


        data[0] = MOTOR_SETUP_CMD;
        data[1] = uPower;
        data[2] = motor_config; //TBD
        data[3] = data[0] + data[1] + data[2];
        data[4] = 0xff;

        spi_sync_transfer(data[0], PCs, 0);
        in_data[0] = spi_sync_transfer(data[1], PCs, 0);
        in_data[1] = spi_sync_transfer(data[2], PCs, 0);
        in_data[2] = spi_sync_transfer(data[3], PCs, 0);
        in_data[3] = spi_sync_transfer(0xFF, PCs, 1);
        
        printf("Ret: %x%x%x%x\n\r", in_data[0], in_data[1], in_data[2], in_data[3]);

        if(in_data[3] != 0x78)
        {
            while(tryout--)
            {
                spi_sync_transfer(data[0], PCs, 0);
                in_data[0] = spi_sync_transfer(data[1], PCs, 0);
                in_data[1] = spi_sync_transfer(data[2], PCs, 0);
                in_data[2] = spi_sync_transfer(data[3], PCs, 0);
                in_data[3] = spi_sync_transfer(0xFF, PCs, 1);
                
                printf(">Ret: %x%x%x%x\n\r", in_data[0], in_data[1], in_data[2], in_data[3]);

                if(in_data[3] == 0x78)
                {
                    tryout = 0;
                    
                    return 0;
                    break;
                }
            }

            if(in_data[3] != 0x78)
            {
                printf("motor_set_power: failed\n\r");
                return -1;
            }

        }
    }
    else
    {
        printf("motor_set_power: Calculated uPower %d is bigger than uMAX: %d, ignoring... \n\r", uPower, uMAX);
    }

    return -1;
}