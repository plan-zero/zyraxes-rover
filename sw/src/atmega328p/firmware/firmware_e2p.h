#ifndef E2P_LAYOUT_H_
#define E2P_LAYOUT_H_

#include "stdlib.h"
#include <avr/eeprom.h>

#define TWI_CHIP_ADDRESS                       (const uint8_t*) 0
#define TWI_CHIP_ADDR_LENGTH                   (uint8_t) 1

#define FIRMWARE_VERSION_ADDRESS               (const uint8_t*) 1
#define FIRMWARE_VERSION_LENGTH                (uint8_t) 3

#define E2P_SIZE                       (uint16_t)4

#define EEP_MEMORY_SECTION __attribute__((section(".eeprom")))


#define e2p_update_twi_address(value) eeprom_update_block ((void*)value, (void*)TWI_CHIP_ADDRESS, TWI_CHIP_ADDR_LENGTH)
#define e2p_read_twi_address(value)   eeprom_read_block ((void*)&value,(void*)TWI_CHIP_ADDRESS,TWI_CHIP_ADDR_LENGTH)

#define e2p_update_firmware_version(value) eeprom_update_block ((void*)value, (void*)FIRMWARE_VERSION_ADDRESS, FIRMWARE_VERSION_LENGTH)
#define e2p_read_firmware_version(value)   eeprom_read_block ((void*)&value,(void*)FIRMWARE_VERSION_ADDRESS,FIRMWARE_VERSION_LENGTH)

#endif
