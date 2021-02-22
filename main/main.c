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
#include <stdatomic.h>

#define SENSOR_COUNT 3

// pins for ADC
#define ADC_CHANNEL ADC_CHANNEL_6
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTENUATION ADC_ATTEN_DB_11
#define ADC_UNIT ADC_UNIT_1


volatile uint16_t sensorMin =  4400;
volatile uint16_t sensorMax =  0;
uint8_t sensor = 0;
volatile uint16_t sensorValues[SENSOR_COUNT];
static const char* sensorLabels[SENSOR_COUNT] = { "Busuioc", "Menta", "Oregano" };
static uint8_t muxSelectPins[3] = {PIN_S0, PIN_S1, PIN_S2};

adc_t config;
u8g2_t u8g2; // a structure which will contain all the data for one display
display_data_t displayData;

void taskSensorRead(void* configParam) {
	adc_t* config = (adc_t*)configParam;
	while(1) {
		for(sensor = 0; sensor < SENSOR_COUNT; ++sensor) {
			select_mux_input(sensor, muxSelectPins);
			uint16_t adc_reading = readAdcPin(config);
			atomic_store(&sensorValues[sensor], adc_reading);
			ESP_LOGI(TAG, "S:%d V:%d m:%d M:%d",sensor, adc_reading, sensorMin, sensorMax);
			uint16_t currentSensorMin = atomic_load(&sensorMin);
			uint16_t currentSensorMax = atomic_load(&sensorMax);
			atomic_store(&sensorMin, fmin(currentSensorMin, adc_reading));
			atomic_store(&sensorMax, fmax(currentSensorMax, adc_reading));
		}
	}
}

void renderFrames(display_data_t* data, uint8_t frames[2], int16_t x, int16_t y, int64_t elapsedTime, int64_t stateElapsedTime, bool transitioning) {
	u8g2_t* display = data->display;

	//ESP_LOGI(TAG, "u8g2_ClearBuffer");
	u8g2_ClearBuffer(display);
	for(uint8_t i = 0; i< 2; ++i){
		data->frames[frames[i]](display, x + i * display->width, 0, frames[i], elapsedTime, stateElapsedTime, transitioning, data->frame_data);
	}

	//ESP_LOGI(TAG, "u8g2_SendBuffer");
	u8g2_SendBuffer(display);
}


void taskUpdateDisplay(void* displayParam) {
	display_data_t* data = (display_data_t*)displayParam;
	u8g2_t* display = data->display;
	u8g2_uint_t display_width = u8g2_GetDisplayWidth(display);


	double x = 0;
	uint8_t frames[2];
	uint8_t current_frame = 0;

	int64_t previousTime = esp_timer_get_time();
	int64_t stateStartTime = previousTime;
	bool transitioning = false;

	while (1) {

		int64_t currentTime = esp_timer_get_time();
		int64_t elapsedTime = currentTime - previousTime;
		int64_t stateElapsedTime = currentTime - stateStartTime;
		previousTime = currentTime;

		frames[0] = current_frame;
		frames[1] = (current_frame + 1) % data->frame_count;

		if(transitioning) {
			x = fmax(-display_width, -1.0 * display_width * stateElapsedTime / (data->frame_transition_ms * 1000.0));
		}

		renderFrames(data, frames, (int64_t)x, 0, elapsedTime, stateElapsedTime, transitioning);
		if(transitioning && stateElapsedTime > (data->frame_transition_ms * 1000)) {
			transitioning = false;
			stateStartTime = esp_timer_get_time();
			x = 0;
			current_frame=frames[1];

		} else if(!transitioning && stateElapsedTime > (data->frame_duration_ms * 1000)) {
			transitioning = true;
			stateStartTime = esp_timer_get_time();
		}

		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void sensorReadingRender(u8g2_t* display, int16_t ox, int16_t oy, uint8_t sensor, int64_t elapsedTime, int64_t stateElapsedTime,  bool transitioning, void* ignore) {

	const uint8_t* font = u8g2_font_8x13_mf;
	drawCenteredString(display, ox, oy+ 17, sensorLabels[sensor], font);

	uint16_t currentSensorMin = atomic_load(&sensorMin);
	uint16_t currentSensorMax = atomic_load(&sensorMax);
	uint16_t sensorValue = atomic_load(&sensorValues[sensor]);

	//uint32_t normalizedSensorValue = fmax(0, fmin(sensorMax, sensorMax - sensorValues[sensor]));
	float sensorPercentage = (sensorValue - currentSensorMin) * 1.0 / (currentSensorMax-currentSensorMin);

	char *text;
	switch((stateElapsedTime / 2000000)% 2) {
//	case 1:
//		asprintf(&text, "%d", normalizedSensorValue);
//		break;
	case 1:
		asprintf(&text, "%d[%d-%d]", sensorValues[sensor], sensorMin, sensorMax);
		break;
	default:
		asprintf(&text, "%2.1f%%", sensorPercentage*100.0);
		break;

	}
	drawCenteredString(display, ox, oy+50, text, font);
	free(text);


//	ESP_LOGI(TAG, "u8g2_DrawBox");
	u8g2_uint_t display_width = u8g2_GetDisplayWidth(display);
	u8g2_DrawBox(display, ox + 20, 26, (u8g2_uint_t)(sensorPercentage * (display_width - 40)), 6);
	u8g2_DrawFrame(display, ox + 20, 26, display_width - 40, 6);

}

void app_main(void) {
	initialize_mux_pins();
	config.unit = ADC_UNIT;
	config.channel = ADC_CHANNEL;
	config.atten = ADC_ATTENUATION;
	config.width = ADC_WIDTH;

	initializeAdcPin(&config);
	initialize_display(&u8g2, PIN_SDA, PIN_SCL);

	displayData.display = &u8g2;
	displayData.frames = malloc(sizeof(frame_callback) * SENSOR_COUNT);
	displayData.frame_count= SENSOR_COUNT;
	displayData.frame_duration_ms = 4000;
	displayData.frame_transition_ms = 300;

	for(uint8_t i = 0; i < SENSOR_COUNT; ++i) {
		displayData.frames[i] = sensorReadingRender;
	}

	xTaskCreatePinnedToCore(&taskSensorRead, "sensor_read", 5000, &config, 1, NULL, CORE_ID_APP);
	xTaskCreate(&taskUpdateDisplay, "update_display", 5000, &displayData, 5, NULL);


	ESP_LOGI(TAG, "All done!");

	vTaskDelete(NULL);
}

