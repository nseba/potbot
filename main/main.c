#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/_timeval.h>
#include <sys/time.h>

#include "../components/u8g2/csrc/u8g2.h"
#include "../components/u8g2/csrc/u8x8.h"
#include "u8g2_esp32_hal.h"

// SDA - GPIO21
#define PIN_SDA 21

// SCL - GPIO22
#define PIN_SCL 22

#define GPIO_OUTPUT_PIN_SEL  (1ULL<<5)

static const char *TAG = "sh1106";

//void updateDisplay(void* pvParameters) {
//
//
//
//}

//void task_test_SSD1306i2c(void *ignore) {
void app_main(void) {
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda = PIN_SDA;
	u8g2_esp32_hal.scl = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

	u8g2_t u8g2; // a structure which will contain all the data for one display
	u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2,
	U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
	u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

	ESP_LOGI(TAG, "u8g2_InitDisplay");
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

	ESP_LOGI(TAG, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(&u8g2, 0); // wake up display

//	u8g2_t* display = (u8g2_t*)pvParameters;
	u8g2_t *display = &u8g2;
	struct timeval tv_now;
	gettimeofday(&tv_now, NULL);

	time_t initialTime = tv_now.tv_sec;

	while (1) {
		gettimeofday(&tv_now, NULL);
		time_t currentTime = tv_now.tv_sec;
		int delta = (currentTime - initialTime) % 60;

		ESP_LOGI(TAG, "u8g2_ClearBuffer");
		u8g2_ClearBuffer(display);

		ESP_LOGI(TAG, "u8g2_DrawBox");
		u8g2_DrawBox(display, 0, 26, delta, 6);
		u8g2_DrawFrame(display, 0, 26, 60, 6);

		ESP_LOGI(TAG, "u8g2_SetFont");
		u8g2_SetFont(display, u8g2_font_ncenB14_tr);

		char *text;
		asprintf(&text, "Time %d", delta);

		ESP_LOGI(TAG, "u8g2_DrawStr");
		u8g2_DrawStr(display, 2, 17, text);
		free(text);

		ESP_LOGI(TAG, "u8g2_SendBuffer");
		u8g2_SendBuffer(display);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

//	xTaskCreatePinnedToCore(&updateDisplay, "updateDisplay", 1000, &u8g2, (configMAX_PRIORITIES -1 ), NULL, 0);

	//	ESP_LOGI(TAG, "u8g2_ClearBuffer");
	//	u8g2_ClearBuffer(&u8g2);
	//
	//	ESP_LOGI(TAG, "u8g2_DrawBox");
	//	u8g2_DrawBox(&u8g2, 0, 26, 80,6);
	//	u8g2_DrawFrame(&u8g2, 0,26,100,6);
	//
	//	ESP_LOGI(TAG, "u8g2_SetFont");
	//    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
	//
	//    ESP_LOGI(TAG, "u8g2_DrawStr");
	//    u8g2_DrawStr(&u8g2, 2,17,"Hi Lera!");
	//
	//    ESP_LOGI(TAG, "u8g2_SendBuffer");
	//	u8g2_SendBuffer(&u8g2);

	ESP_LOGI(TAG, "All done!");

	vTaskDelete(NULL);
}

