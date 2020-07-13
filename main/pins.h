// SDA - GPIO21
#define PIN_SDA 21
// SCL - GPIO22
#define PIN_SCL 22

// pins for selecting the serial MUX input
#define PIN_S0 16
#define PIN_S1 17
#define PIN_S2 18
#define GPIO_OUTPUT_MUX_PIN_SEL ((1ULL<<PIN_S0) | (1ULL<<PIN_S1) | (1ULL<<PIN_S2))


// pins for ADC
#define ADC_CHANNEL ADC_CHANNEL_6
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTENUATION ADC_ATTEN_DB_11
#define ADC_UNIT ADC_UNIT_1
