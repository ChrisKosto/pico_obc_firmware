#ifndef MOCK_PICO_STDLIB_H
#define MOCK_PICO_STDLIB_H
#define GPIO_FUNC_I2C 3u
void gpio_set_function(unsigned pin, unsigned mode);
void gpio_pull_up(unsigned pin);
#endif
