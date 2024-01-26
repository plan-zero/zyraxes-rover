#include <asf.h>

#include "nvm.h"
#include "stdio.h"

/* NVRAM test user page address. The first 8 bytes in the user page is
 * reserved as fuse settings and should not be programmed */
#define USER_PAGE_ADDRESS (FLASH_USER_PAGE_ADDR + 8)



/*! \brief Prints the variables stored in NVRAM.
 *
 * \param nvram_data  Pointer to the NVRAM data structure to print.
 */
static void print_nvram_variables(nvram_data_t *nvram_data)
{
    char str[5];
    for(unsigned int i = 0; i < CALIBRATION_DATA_SIZE; i++)
    {
        snprintf(str, sizeof(str), "%f", nvram_data->calibration_data[i]);
        printf("%s ",str);
        if( (i % 16 == 0) && (i != 0) )
            printf("\n\r");
    }
}

/*! \brief This is an example demonstrating flash read / write data accesses
 *         using the FLASHCALW driver.
 *
 * \param caption     Caption to print before running the example.
 * \param nvram_data  Pointer to the NVRAM data structure to use in the example.
 */
void flash_rw_calibration(float *write_data, nvram_data_t *nvram_data)
{

	printf("Previous calibration data\r\n");
	print_nvram_variables(nvram_data);

	printf("\r\nClearing NVRAM variables... \n\r");

	/* Clear NVRAM */
	flashcalw_memset((void *)nvram_data, 0x0, 8, sizeof(*nvram_data),
			true);
        
	printf("Writing calibration data: \r\n");
	flashcalw_memcpy((void *)&nvram_data->calibration_data, write_data,
			sizeof(nvram_data->calibration_data), true);
	print_nvram_variables(nvram_data);
}


	/* Apply the example to the flash array. */
	//flash_rw_example(
	//	"\x0C=== Using a piece of the flash array as NVRAM ===\r\n",
	//	(nvram_data_t *)NVRAM_PAGE_ADDRESS);

