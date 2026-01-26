#ifndef SET_LED_HPP
#define SET_LED_HPP

#include "control/command.hpp"
#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace plasma;


class SetLEDCommand : public Command {

private:

    WS2812* leds;

public:

    SetLEDCommand(WS2812* l) : leds(l) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {
        
        *response_len = 1;

        // We expect 1 byte for led index and 3 bytes for red, green and blue channels
        if (args_len != 4) {
            response[0] = 0x00;
            return false;
        }

        // Extract args
        uint8_t led_id = args[0];
        if (led_id >= servo2040::NUM_LEDS) {
            response[0] = 0x00;
            return false;
        }

        // Set servo angle
        leds->set_rgb(led_id, args[1], args[2], args[3]);
        response[0] = 0x01;
        
        return true;
    }
};

#endif // SET_LED_HPP
