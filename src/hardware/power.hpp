#ifndef POWER_HPP
#define POWER_HPP

#include "servo2040.hpp"

using namespace servo;


class PowerTrace {

private:
    uint8_t pin;
    bool state;

public:

    PowerTrace(uint8_t gpio_pin) : pin(gpio_pin), state(false) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0); // Ensure starts at OFF state
    }

    // Turn power to the servos ON
    void on() {
        gpio_put(pin, 1);
        state = true;
    }

    // Turn power to the servos OFF
    void off() {
        gpio_put(pin, 0);
        state = false;
    }

    // Toggle power
    void toggle() {
        state = !state;
        gpio_put(pin, state);
    }

    // Get the current state
    bool is_on() const {
        return state;
    }
};

#endif // POWER_HPP
