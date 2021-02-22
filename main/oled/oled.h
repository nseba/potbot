#include "u8g2.h"
#include "u8x8.h"
#include "u8g2_esp32_hal.h"

static const char *TAG = "sh1106";

typedef void (*frame_callback)(u8g2_t* display, int16_t ox, int16_t oy, uint8_t current_frame, int64_t elapsedTime, int64_t stateElapsedTime, bool transitioning, void* data);

struct display_data_struct {
	u8g2_t* display;
	uint8_t frame_count;
	void* frame_data;
	frame_callback* frames;
	uint64_t frame_duration_ms;
	uint64_t frame_transition_ms;
};

typedef struct display_data_struct display_data_t;

void initialize_display(u8g2_t* display, gpio_num_t sdaPin, gpio_num_t sclPin);


void drawCenteredString(u8g2_t* display, int16_t x, int16_t y, const char *text, const uint8_t  *font);
