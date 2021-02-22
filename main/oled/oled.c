#include "oled.h"
#include <esp_log.h>

void initialize_display(u8g2_t* display, gpio_num_t sdaPin, gpio_num_t sclPin) {
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda = sdaPin;
	u8g2_esp32_hal.scl = sclPin;
	u8g2_esp32_hal_init(u8g2_esp32_hal);


	u8g2_Setup_sh1106_i2c_128x64_noname_f(
			display,
			U8G2_R0,
			u8g2_esp32_i2c_byte_cb,
			u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
	u8x8_SetI2CAddress(&display->u8x8, 0x78);

	ESP_LOGI(TAG, "u8g2_InitDisplay");
	u8g2_InitDisplay(display); // send init sequence to the display, display is in sleep mode after this,

	ESP_LOGI(TAG, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(display, 0); // wake up display
}


void drawCenteredString(u8g2_t* display, int16_t ox, int16_t oy, const char *text, const uint8_t  *font) {
	u8g2_SetFont(display, font);
	u8g2_uint_t text_width = u8g2_GetStrWidth(display, text);
	u8g2_uint_t text_position = (display->width - text_width) / 2;
	u8g2_DrawStr(display, ox+text_position, oy, text);
}
