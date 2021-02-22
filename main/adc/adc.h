#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>


#include "driver/adc.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   20          //Multisampling



//static const adc_atten_t atten = ADC_ATTEN_DB_11;
//static const adc_unit_t unit = ADC_UNIT_1;

struct adc_config_struct {
	adc_unit_t unit;
	adc_channel_t channel;
	adc_atten_t atten;
	adc_bits_width_t width;

#if CONFIG_IDF_TARGET_ESP32
	esp_adc_cal_characteristics_t *adc_chars;
	//static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
	//static const adc_bits_width_t width = ADC_WIDTH_BIT_11;
#elif CONFIG_IDF_TARGET_ESP32S2
	//static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
	//static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
};

typedef struct adc_config_struct adc_t;

void initializeAdcPin(adc_t* config);
uint16_t readAdcPin(adc_t* config);
