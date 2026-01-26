#ifndef GET_LED_HPP
#define GET_LED_HPP

#include "control/command.hpp"
#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace plasma;


class GetLEDCommand : public Command {

private:

    WS2812* leds;

public:

    GetLEDCommand(WS2812* l) : leds(l) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        // We should return 3 bytes (red, green, blue)
        *response_len = 3;

        // We expect 1 byte for led index
        if (args_len != 1) {
            memset(response, 0x00, *response_len);
            return false;
        }

        // Extract LED index
        uint8_t led_id = args[0];
        if (led_id >= servo2040::NUM_LEDS) {
            memset(response, 0x00, *response_len);
            return false;
        }

        // Get current RGB state
        WS2812::RGB color = leds->get(led_id);
        
        // Pack response: r, g, b (3 bytes)
        response[0] = color.r;
        response[1] = color.g;
        response[2] = color.b;

        return true;
    }
};

#endif // GET_LED_HPP
