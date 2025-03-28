#include "motor_twi.h"
#include "zyra_twi.h"
#include "twi_common.h"
#include "delay.h"
#include "string.h"
#include "motor_calibration.h"


#define TWI__PACKAGE_LENGTH 10

uint8_t firmware_tx_data[TWI__PACKAGE_LENGTH];
uint8_t firmware_rx_data[TWI__PACKAGE_LENGTH];

twi_package_t packet_tx, packet_rx;

static status_code_t _twi_write_package(uint8_t chip_address, uint8_t chip_offset, uint8_t buffer_offset, uint8_t length)
{
	/* TWI chip address to communicate with */
	packet_tx.chip = chip_address;
	packet_tx.ten_bit = false;
	/* TWI address/commands to issue to the other chip (node) */
	packet_tx.addr[0] = chip_offset;
	packet_tx.addr[1] = 0;
	/* Length of the TWI data address segment (1-3 bytes) */
	packet_tx.addr_length = 1;
	/* Where to find the data to be written */
	packet_tx.buffer = (void *) &firmware_tx_data[buffer_offset];
	/* How many bytes do we want to write */
	packet_tx.length = length;
	/* Write data to TARGET */
	return twi_master_write(EXAMPLE_TWIM, &packet_tx);
}

static status_code_t _twi_read_package(uint8_t chip_address, uint8_t chip_offset, uint8_t buffer_offset, uint8_t length)
{
	/* TWI chip address to communicate with */
	packet_rx.chip = chip_address;
	packet_rx.ten_bit = false;
	/* Length of the TWI data address segment (1-3 bytes) */
	packet_rx.addr_length = 1;
	/* How many bytes do we want to write */
	packet_rx.length = length;
	/* TWI address/commands to issue to the other chip (node) */
	packet_rx.addr[0] = chip_offset;
	packet_rx.addr[1] = 0;
	/* Where to find the data to be written */
	packet_rx.buffer = &firmware_rx_data[buffer_offset];
	/* Read data from TARGET */
	return twi_master_read(EXAMPLE_TWIM, &packet_rx);
}


void motor_diag_twi(uint8_t address)
{
    uint16_t angleTemp = 0;
    puts("motor_diag: Checking AS5047 diagnostic and error registers ... \n\r");
    //send DIAG command to the firmware
    firmware_tx_data[TWI_RX_ADDR_CMD_FIRMWARE] = CMD_FIRMWARE_SENSOR_DIAG;
    _twi_write_package(address, TWI_RX_ADDR_CMD_FIRMWARE, TWI_RX_ADDR_CMD_FIRMWARE, 1);
    //wait for slave to finish the diagnose
    delay_ms(150);
    _twi_read_package(address, TWI_TX_ADDR_SENSOR_DIAG1, TWI_TX_ADDR_SENSOR_DIAG1, 2);
    _twi_read_package(address, TWI_TX_ADDR_SENSOR_DIAG2, TWI_TX_ADDR_SENSOR_DIAG2, 2);

    puts("motor_diag: Check DIAAGC register ...  n\r\r");
    memcpy(&angleTemp, &firmware_rx_data[TWI_TX_ADDR_SENSOR_DIAG1], 2);//*(uint16_t*)&firmware_rx_data[TWI_TX_ADDR_SENSOR_DIAG1];
    printf("%d BIN n\r", (angleTemp | 0B1110000000000000000 ));

  	if (angleTemp & (1 << 14))    puts("  Error occurred  \n\r");
  	if (angleTemp & (1 << 11))    puts("  MAGH - magnetic field strength too high, set if AGC = 0x00. This indicates the non-linearity error may be increased \n\r");
  	if (angleTemp & (1 << 10))    puts("  MAGL - magnetic field strength too low, set if AGC = 0xFF. This indicates the output noise of the measured angle may be increased\n\r");
  	if (angleTemp & (1 << 9))     puts("  COF - CORDIC overflow. This indicates the measured angle is not reliable\n\r");
  	if (angleTemp & (1 << 8))     puts("  LF - offset compensation completed. At power-up, an internal offset compensation procedure is started, and this bit is set when the procedure is completed\n\r");
  	if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 11)) | (angleTemp & (1 << 10)) | (angleTemp & (1 << 9))))  puts("Looks good!\n\r");

    puts("motor_diag: Check ERRFL register ...  \n\r\r");
    memcpy(&angleTemp, &firmware_rx_data[TWI_TX_ADDR_SENSOR_DIAG2], 2);//*(uint16_t*)&firmware_rx_data[TWI_TX_ADDR_SENSOR_DIAG1];
    printf("%d BIN\n\r", (angleTemp | 0B1110000000000000000 ));

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
}


int motor_sync_twi(uint8_t address)
{
    status_code_t status = STATUS_OK;
    status = _twi_read_package(address, TWI_TX_ADDR_FIRMWARE_VERSION, TWI_TX_ADDR_FIRMWARE_VERSION, 3);
    if (status == STATUS_OK)
    {
        printf("motor_sync_twi: sync completed, slave firmware version v%d.%d.%d \r\n", firmware_rx_data[TWI_TX_ADDR_FIRMWARE_VERSION], 
                    firmware_rx_data[TWI_TX_ADDR_FIRMWARE_VERSION + 1],
                    firmware_rx_data[TWI_TX_ADDR_FIRMWARE_VERSION + 2]);
        return 0;
    }
    else
    {
        //com error
        printf("motor_sync_twi:\tFAILED, comm error: %d\r\n", status);
        return -1;
    }
    //unhandled exception
    return -1;
    
}

uint16_t motor_micro_step_twi(uint8_t address, uint8_t dir, uint16_t steps, uint8_t rpm)
{
    status_code_t status = STATUS_OK;
    firmware_tx_data[TWI_RX_ADDR_CMD] = CMD_MOTOR_STEP_CCW - dir;
    firmware_tx_data[TWI_RX_ADDR_STEPS] = steps >> 8;
    firmware_tx_data[TWI_RX_ADDR_STEPS + 1] = steps;
    firmware_tx_data[TWI_RX_ADDR_RPM] = rpm;

    status = _twi_write_package(address, TWI_RX_ADDR_CMD, TWI_RX_ADDR_CMD, 4);
    if(status != STATUS_OK){
        printf("motor_micro_step_twi: comm error, %d \n\r", status);
        return 1;
    }

    return 0;
}

int motor_read_raw_twi(uint8_t address)
{
    status_code_t status = STATUS_OK;
    uint16_t angle = 0;
    status = _twi_read_package(address, TWI_TX_ADDR_SENSOR_RAW, TWI_TX_ADDR_SENSOR_RAW, 2);
    if(status != STATUS_OK){
        printf("motor_read_raw_twi: comm error, %d \n\r", status);
        return 1;
    }

    angle = firmware_rx_data[TWI_TX_ADDR_SENSOR_RAW] << 8 | firmware_rx_data[TWI_TX_ADDR_SENSOR_RAW + 1];

    if(angle & (1 << 14)){
        printf("Angle error! \n\r");
        return 1;
    }
    angle &= 0B0011111111111111;
    return angle >> MAGNETIC_REDUCE_RESOLUTION;
}

