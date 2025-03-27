#include "firmware_e2p.h"
#include "stdint.h"
#include "firmware_version.h"


union e2p_data
{
    struct{
        uint8_t e2p_chip_twi_address[TWI_CHIP_ADDR_LENGTH];
        uint8_t e2p_firmware_version[FIRMWARE_VERSION_LENGTH];
    }e2p_fields;
    uint8_t e2p_raw[E2P_SIZE];
};

union e2p_data e2p_default_data EEP_MEMORY_SECTION =
{
    .e2p_fields.e2p_chip_twi_address = {SLAVE_TWI_CHIP_ADDRESS},
    .e2p_fields.e2p_firmware_version = {VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH}
};
