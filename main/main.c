#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/_timeval.h>
#include <sys/time.h>
#include <math.h>
#include "esp_timer.h"

#include "pins.h"
#include "oled/oled.h"
#include "adc/adc.h"
#include "mux/mux.h"

#define SENSOR_COUNT 3
#define SENSOR_MIN 1250
#define SENSOR_MAX 3605

uint8_t sensor = 0;
uint32_t sensor_values[SENSOR_COUNT];
static const char* sensor_labels[SENSOR_COUNT] = { "Busuioc", "Menta", "Oregano" };
static uint8_t mux_select_pins[3] = {PIN_S0, PIN_S1, PIN_S2};

adc_t config;
u8g2_t u8g2; // a structure which will contain all the data for one display
display_data_t displayData;

void task_sensor_read(void* configParam) {
	adc_t* config = (adc_t*)configParam;
	while(1) {
		for(sensor = 0; sensor < SENSOR_COUNT; ++sensor) {
			select_mux_input(sensor, mux_select_pins);
			uint32_t adc_reading = read_adc_pin(config);
			sensor_values[sensor] = adc_reading;
		}
	}
}

void renderFrames(display_data_t* data, uint8_t frames[2], int16_t x, int16_t y, bool transitioning) {
	u8g2_t* display = data->display;

	ESP_LOGI(TAG, "u8g2_ClearBuffer");
	u8g2_ClearBuffer(display);
	for(uint8_t i = 0; i< 2; ++i){
		data->frames[frames[i]](display, x + i * display->width, 0, frames[i], transitioning, data->frame_data);
	}

	ESP_LOGI(TAG, "u8g2_SendBuffer");
	u8g2_SendBuffer(display);
}


void task_update_display(void* displayParam) {
	display_data_t* data = (display_data_t*)displayParam;
	u8g2_t* display = data->display;


	int16_t x = 0;
	uint8_t frames[2];
	uint8_t current_frame = 0;

	while (1) {

		frames[0] = current_frame;
		frames[1] = (current_frame + 1) % data->frame_count;

		renderFrames(data, frames, x, 0, false);

		vTaskDelay(data->frame_duration_ms / portTICK_RATE_MS);

		TickType_t xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount ();
		const TickType_t xFrequency = 10;

		TickType_t xWakeTime = xLastWakeTime;
		for(;;) {
			// Wait for the next cycle.
			vTaskDelayUntil( &xLastWakeTime, xFrequency );

			int16_t delta = display->width  * (xLastWakeTime - xWakeTime) / data->frame_transition_ms;
			if(delta > 0) {
				x = fmax(-display->width, x-delta);
				xWakeTime = xLastWakeTime;
			}

			renderFrames(data, frames, x, 0, true);

			if (x == - display->width) {
				x = 0;
				current_frame=frames[1];

				break;
			}
		}


		//		ESP_LOGI(TAG, "u8g2_DrawBox");
		//		u8g2_DrawBox(display, 0, 26, sensor_values[sensor], 6);
		//		u8g2_DrawFrame(display, 0, 26, 100, 6);


		//		ESP_LOGI(TAG, "u8g2_SetFont");
		//		u8g2_SetFont(display, u8g2_font_8x13_mf);
		//
		//		u8g2_uint_t text_width = u8g2_GetStrWidth(display, sensor_labels[sensor]);
		//		u8g2_uint_t display_width = u8g2_GetDisplayWidth(display);
		//		u8g2_uint_t text_position = (display_width - text_width) / 2;
		//		u8g2_DrawStr(display, text_position, 17, sensor_labels[sensor]);
		//
		//		char *text;
		//		asprintf(&text, "%2d", delta);
		//		//		asprintf(&text, "T%2d S%d-%d%d%d V%d", delta, sensor, mux_select_bits[2], mux_select_bits[1],mux_select_bits[0], sensor_values[sensor]);
		//		//		asprintf(&text, "S%d-%d%d%d V%d", sensor, mux_select_bits[2], mux_select_bits[1],mux_select_bits[0], sensor_values[sensor]);
		//
		//		ESP_LOGI(TAG, "u8g2_DrawStr");
		//		u8g2_DrawStr(display, 2, 35, text);
		//		free(text);


		//vTaskDelay(10 / portTICK_RATE_MS);
	}
}

void sensor_reading_render(u8g2_t* display, int16_t ox, int16_t oy, uint8_t sensor, bool transitioning, void* ignore) {
	u8g2_SetFont(display, u8g2_font_8x13_mf);
	u8g2_uint_t text_width = u8g2_GetStrWidth(display, sensor_labels[sensor]);
	u8g2_uint_t display_width = u8g2_GetDisplayWidth(display);
	u8g2_uint_t text_position = (display_width - text_width) / 2;
	u8g2_DrawStr(display, ox+text_position, oy+17, sensor_labels[sensor]);

	uint32_t normalizedSensorValue = fmax(0, fmin(SENSOR_MAX, SENSOR_MAX - sensor_values[sensor]));
	float sensorPercentage = fmax(0, fmin(1, (normalizedSensorValue * 1.0 ) / (SENSOR_MAX - SENSOR_MIN)));

	char *text;
	asprintf(&text, "%2.1f%%", sensorPercentage*100.0);
	text_width = u8g2_GetStrWidth(display, text);
	text_position = (display_width - text_width) / 2;
	u8g2_DrawStr(display, ox+text_position, oy+50, text);
	free(text);

//	if(!transitioning) {
		ESP_LOGI(TAG, "u8g2_DrawBox");
		u8g2_DrawBox(display, ox + 20, 26, (u8g2_uint_t)(sensorPercentage * (display_width - 40)), 6);
		u8g2_DrawFrame(display, ox + 20, 26, display_width - 40, 6);
//	}

//			asprintf(&text, "%d", sensor_values[sensor]);
//		u8g2_DrawStr(display, ox, oy+65, text);
//		free(text);
	//	char *text;

}

void app_main(void) {
	initialize_mux_pins();
	config.unit = ADC_UNIT;
	config.channel = ADC_CHANNEL;
	config.atten = ADC_ATTENUATION;
	config.width = ADC_WIDTH;

	initialize_adc_pin(&config);
	initialize_display(&u8g2, PIN_SDA, PIN_SCL);

	displayData.display = &u8g2;
	displayData.frames = malloc(sizeof(frame_callback) * SENSOR_COUNT);
	displayData.frame_count= SENSOR_COUNT;
	displayData.frame_duration_ms = 1000;
	displayData.frame_transition_ms = 100;

	for(uint8_t i = 0; i < SENSOR_COUNT; ++i) {
		displayData.frames[i] = sensor_reading_render;
	}

	xTaskCreate(&task_sensor_read, "sensor_read", 5000, &config, 1, NULL);
	xTaskCreate(&task_update_display, "update_display", 5000, &displayData, 5, NULL);


	ESP_LOGI(TAG, "All done!");

	vTaskDelete(NULL);
}

