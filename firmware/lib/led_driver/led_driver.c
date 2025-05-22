#include "led_driver.h"

void led_init(gpio_num_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

void multiple_led_init(const gpio_num_t *pins, int num_pins) {
    for (int i = 0; i < num_pins; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
    }
}

void led_drive(gpio_num_t pin, int state) {
    gpio_set_level(pin, state);
}

void led_toggle(gpio_num_t pin) {
    static int state = 0;
    gpio_set_level(pin, state);
    state = !state;
}

void multiple_led_drive(const gpio_num_t *pins, int num_pins, int state) {
    for (int i = 0; i < num_pins; i++) {
        gpio_set_level(pins[i], state);
    }
}