#include "board.h"

#include "hardware/adc.h"

void board_temperature_init(void) {
    adc_init();
    adc_set_temp_sensor_enabled(true);
}

float board_read_temperature(void) {
    adc_select_input(4);
    uint16_t raw = adc_read();
    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = raw * conversion_factor;
    return 27.0f - (voltage - 0.706f) / 0.001721f;
}
