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

#include "driver/gpio.h"
#include "driver/adc.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif

// SDA - GPIO21
#define PIN_SDA 21
// SCL - GPIO22
#define PIN_SCL 22

// pins for selecting the serial MUX input
#define PIN_S0 16
#define PIN_S1 17
#define PIN_S2 18
#define GPIO_OUTPUT_MUX_PIN_SEL ((1ULL<<PIN_S0) | (1ULL<<PIN_S1) | (1ULL<<PIN_S2))

#define SENSOR_COUNT 3

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#if CONFIG_IDF_TARGET_ESP32
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static const char *TAG = "sh1106";


#if CONFIG_IDF_TARGET_ESP32
static void check_efuse(void)
{
	//Check TP is burned into eFuse
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
		printf("eFuse Two Point: Supported\n");
	} else {
		printf("eFuse Two Point: NOT supported\n");
	}

	//Check Vref is burned into eFuse
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
		printf("eFuse Vref: Supported\n");
	} else {
		printf("eFuse Vref: NOT supported\n");
	}
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
	if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
		printf("Characterized using Two Point Value\n");
	} else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
		printf("Characterized using eFuse Vref\n");
	} else {
		printf("Characterized using Default Vref\n");
	}
}
#endif

uint8_t sensor = 0;
uint8_t mux_select_bits[3] = {0,0,0};
uint8_t mux_select_pins[3] = {PIN_S0, PIN_S1, PIN_S2};
uint32_t sensor_values[SENSOR_COUNT];

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

void initialize_adc_pins() {
#if CONFIG_IDF_TARGET_ESP32
	//Check if Two Point or Vref are burned into eFuse
	check_efuse();
#endif
	//Configure ADC
	if (unit == ADC_UNIT_1) {
		adc1_config_width(width);
		adc1_config_channel_atten(channel, atten);
	} else {
		adc2_config_channel_atten((adc2_channel_t)channel, atten);
	}

#if CONFIG_IDF_TARGET_ESP32
	//Characterize ADC
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
	print_char_val_type(val_type);
#endif
}

void select_mux_input(uint8_t input) {
	for(int j = 0; j< 3; ++j) {
		mux_select_bits[j] = (sensor & (1 << j)) ? 1: 0;
		gpio_set_level(mux_select_pins[j], mux_select_bits[j]);
	}
}



void task_sensor_read(void* ignore) {
	while(1) {
		for(sensor = 0; sensor < SENSOR_COUNT; ++sensor) {
			select_mux_input(sensor);

			uint32_t adc_reading = 0;
			//Multisampling
			for (int i = 0; i < NO_OF_SAMPLES; i++) {
				if (unit == ADC_UNIT_1) {
					adc_reading += adc1_get_raw((adc1_channel_t)channel);
				} else {
					int raw;
					adc2_get_raw((adc2_channel_t)channel, width, &raw);
					adc_reading += raw;
				}
			}
			adc_reading /= NO_OF_SAMPLES;

			sensor_values[sensor] = adc_reading;

			vTaskDelay(1000 / portTICK_RATE_MS);
		}
	}
}


void app_main(void) {
	initialize_mux_pins();
	initialize_adc_pins();

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

	xTaskCreate(&task_sensor_read, "sensor_read", 1000, NULL, 1, NULL);

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
		u8g2_SetFont(display, u8g2_font_8x13_mf);

		char *text;
		asprintf(&text, "T%2d S%d-%d%d%d V%d", delta, sensor, mux_select_bits[2], mux_select_bits[1],mux_select_bits[0], sensor_values[sensor]);

		ESP_LOGI(TAG, "u8g2_DrawStr");
		u8g2_DrawStr(display, 2, 17, text);
		free(text);

		ESP_LOGI(TAG, "u8g2_SendBuffer");
		u8g2_SendBuffer(display);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

	ESP_LOGI(TAG, "All done!");

	vTaskDelete(NULL);
}

