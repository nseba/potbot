#include "adc.h"

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



void initializeAdcPin(adc_t* config) {
#if CONFIG_IDF_TARGET_ESP32
	//Check if Two Point or Vref are burned into eFuse
	check_efuse();
#endif
	//Configure ADC
	if (config->unit == ADC_UNIT_1) {
		adc1_config_width(config->width);
		adc1_config_channel_atten(config->channel, config->atten);
	} else {
		adc2_config_channel_atten((adc2_channel_t)config->channel, config->atten);
	}

#if CONFIG_IDF_TARGET_ESP32
	//Characterize ADC
	config->adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(config->unit, config->atten, config->width, DEFAULT_VREF, config->adc_chars);
	print_char_val_type(val_type);
#endif
}

uint16_t readAdcPin(adc_t* config) {

	uint32_t adc_reading = 0;
	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++) {
		if (config->unit == ADC_UNIT_1) {
			adc_reading += adc1_get_raw((adc1_channel_t)config->channel);
		} else {
			int raw;
			adc2_get_raw((adc2_channel_t)config->channel, config->width, &raw);
			adc_reading += raw;
		}
	}
	adc_reading /= NO_OF_SAMPLES;

#if CONFIG_IDF_TARGET_ESP32
	//Convert adc_reading to voltage in mV
	uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, config->adc_chars);
	printf("ADC%d CH%d Raw: %d Voltage %dmV\t\n", config->unit, config->channel, adc_reading, voltage);
#elif CONFIG_IDF_TARGET_ESP32S2
	printf("ADC%d CH%d Raw: %d\t\n", config->unit, config->channel, adc_reading);
#endif
	return (uint16_t)adc_reading;
}
