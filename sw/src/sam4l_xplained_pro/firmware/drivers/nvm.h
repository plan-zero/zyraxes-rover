#ifndef NVM_H
#define NVM_H

#include "asf.h"

/* NVRAM test page address */
#define NVRAM_PAGE_ADDRESS(x) (FLASH_ADDR + FLASH_SIZE - (FLASH_PAGE_SIZE*x))

#define CALIBRATION_DATA_SIZE (FLASH_PAGE_SIZE/sizeof(float))

extern unsigned page_count;
extern const void * page_ptr;
extern int page_number;
void store_lookup(float lookupAngle);



#endif