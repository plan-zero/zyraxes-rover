#include "zyra_twi.h"
#include "asf.h"
#include "delay.h"

uint32_t cpu_speed = 0;

status_code_t zyra_init_twi(void)
{
	/* Set TWIM options */
	cpu_speed = sysclk_get_peripheral_bus_hz(EXAMPLE_TWIM);
	struct twim_config opts = {
		.twim_clk = cpu_speed,
		.speed = TWIM_MASTER_SPEED,
		.hsmode_speed = 0,
		.data_setup_cycles = 0,
		.hsmode_data_setup_cycles = 0,
		.smbus = false,
		.clock_slew_limit = 0,
		.clock_drive_strength_low = 0,
		.data_slew_limit = 0,
		.data_drive_strength_low = 0,
		.hs_clock_slew_limit = 0,
		.hs_clock_drive_strength_high = 0,
		.hs_clock_drive_strength_low = 0,
		.hs_data_slew_limit = 0,
		.hs_data_drive_strength_low = 0,
	};
	/* Initialize the TWIM Module */
	twim_set_callback(EXAMPLE_TWIM, 0, twim_default_callback, 1);

	return twim_set_config(EXAMPLE_TWIM, &opts);
}