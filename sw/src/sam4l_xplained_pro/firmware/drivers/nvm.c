#include "nvm.h"
#include "string.h"

unsigned page_count = 0;
float page[CALIBRATION_DATA_SIZE];
int page_number = 0;
const void * page_ptr;

void store_lookup(float lookupAngle)
{
  page[page_count++] = lookupAngle;
  if(page_count != CALIBRATION_DATA_SIZE)
    return;

  // we've filled an entire page, write it to the flash
  printf("Add to NVM, page no: %d, page_addr: %p \n\r",page_number , page_ptr);
  //flash_rw_calibration(page, page_ptr);
  flashcalw_memcpy(page_ptr, page, FLASH_PAGE_SIZE, true);

  // reset our counters and increment our flash page
  page_number++;
  page_ptr += FLASH_PAGE_SIZE;
  page_count = 0;
  memset(page, 0, sizeof(page));
  
}