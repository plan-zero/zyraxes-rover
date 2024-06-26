#ifndef NVM_H
#define NVM_H

/* NVRAM test page address */
#define NVRAM_PAGE_ADDRESS(x) (FLASH_ADDR + FLASH_SIZE - (FLASH_PAGE_SIZE*x))

/* Structure type containing variables to store in NVRAM using a specific
memory map. */
#define CALIBRATION_DATA_SIZE (FLASH_PAGE_SIZE/sizeof(float))

void flash_rw_calibration(float *write_data, const void *nvram_data);



#endif