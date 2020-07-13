# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES)
#set(COMPONENT_PRIV_REQUIRES "u8g2")

set(COMPONENT_SRCS  "main.c"
                    "oled/u8g2_esp32_hal.c"
                    "oled/oled.c"
                    "adc/adc.c"
                    "mux/mux.c")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()