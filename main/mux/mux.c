#include "driver/gpio.h"
#include "mux.h"

void initialize_mux_pins() {
	gpio_config_t io_conf = {
			.intr_type = GPIO_INTR_DISABLE,
			.mode = GPIO_MODE_OUTPUT,
			.pin_bit_mask = GPIO_OUTPUT_MUX_PIN_SEL,
			.pull_down_en = 0,
			.pull_up_en = 0
	};

	gpio_config(&io_conf);
}


void select_mux_input(uint8_t input, uint8_t mux_select_pins[]) {
	uint8_t mux_select_bits[3] = {0,0,0};
	for(int j = 0; j< 3; ++j) {
		mux_select_bits[j] = (input & (1 << j)) ? 1: 0;
		gpio_set_level(mux_select_pins[j], mux_select_bits[j]);
	}
}
