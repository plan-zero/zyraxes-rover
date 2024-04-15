#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_spi_example.h"
#include "delay.h"
#if (SAMG55)
#include "flexcom.h"
#include <ioport.h>
#endif

#include "zyra_spi.h"

/* Clock polarity. */
#define SPI_CLK_POLARITY 0

/* Clock phase. */
#define SPI_CLK_PHASE 1

/* Delay before SPCK. */
#define SPI_DLYBS 200

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 25


/* Chip select. */
#define SPI_CHIP_SEL_0 0
#define SPI_CHIP_SEL_1 1
#define SPI_CHIP_SEL_2 2
#define SPI_CHIP_SEL_3 3

/* SPI clock setting (Hz). */
static uint32_t gs_ul_spi_clock = 3000000;
#define SPI_TIMEOUT_READ 100 //500us


uint8_t spi_sync_transfer(uint8_t in_data, uint8_t cs, uint8_t last);
void spi_master_initialize(void);

/**
 * \brief Initialize SPI as master.
 */
void spi_master_initialize(void)
{
	puts("-I- Initialize SPI as master\r");

#if (SAMG55)
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);
#else
	/* Configure an SPI peripheral. */
	spi_enable_clock(SPI_MASTER_BASE);
#endif
	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);

	spi_enable_peripheral_select_decode(SPI_MASTER_BASE);
	spi_set_variable_peripheral_select(SPI_MASTER_BASE);
	
	//configure CS0 AS5047
	spi_configure_cs_behavior(SPI_MASTER_BASE, SPI_CHIP_SEL_0, SPI_CS_RISE_NO_TX);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL_0, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL_0, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL_0,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL_0,
			(
#if (SAM4L)
			sysclk_get_pba_hz()
#else
			sysclk_get_peripheral_hz()
#endif
			/ gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL_0, SPI_DLYBS,
			SPI_DLYBCT);

	//configure CS1 attiny24
	spi_configure_cs_behavior(SPI_MASTER_BASE, SPI_CHIP_SEL_1, SPI_CS_RISE_NO_TX);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL_1, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL_1, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL_1,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL_1,
			(
#if (SAM4L)
			sysclk_get_pba_hz()
#else
			sysclk_get_peripheral_hz()
#endif
			/ gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL_1, SPI_DLYBS,
			SPI_DLYBCT);

	//CS2 devices
	spi_configure_cs_behavior(SPI_MASTER_BASE, SPI_CHIP_SEL_2, SPI_CS_RISE_NO_TX);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL_2, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL_2, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL_2,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL_2,
			(
#if (SAM4L)
			sysclk_get_pba_hz()
#else
			sysclk_get_peripheral_hz()
#endif
			/ gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL_2, SPI_DLYBS,
			SPI_DLYBCT);


	//CS3 devices
	spi_configure_cs_behavior(SPI_MASTER_BASE, SPI_CHIP_SEL_3, SPI_CS_RISE_NO_TX);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL_3, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL_3, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL_3,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL_3,
			(
#if (SAM4L)
			sysclk_get_pba_hz()
#else
			sysclk_get_peripheral_hz()
#endif
			/ gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL_3, SPI_DLYBS,
			SPI_DLYBCT);


	spi_enable(SPI_MASTER_BASE);
}

/**
 * \brief Initialize SPI 8 bit sync transfer
 */
uint8_t spi_sync_transfer(uint8_t in_data, uint8_t cs, uint8_t last)
{
	uint32_t timeout = 0;
	uint16_t out_data = 0;
	uint8_t uc_pcs;

	spi_write(SPI_MASTER_BASE, in_data, cs, last);
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0 && timeout < SPI_TIMEOUT_READ)
	{
		delay_us(5);
		timeout++;
	}
	if(timeout >= SPI_TIMEOUT_READ)
	{
		printf("Timeout error: 0x%x", cs);
		return -1;
	}
	spi_read(SPI_MASTER_BASE, &out_data, &uc_pcs);
	return out_data;
}