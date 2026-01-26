#ifndef GET_LEDS_HPP
#define GET_LEDS_HPP

#include "control/command.hpp"
#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace plasma;


class GetLEDsCommand : public Command {

private:

    WS2812* leds;

public:

    GetLEDsCommand(WS2812* l) : leds(l) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        // Extract number of LEDs
        uint8_t num_leds = args[0];

        // Set response length (3 bytes per LED)
        *response_len = num_leds * 3;
        
        // Expected size: 1 byte (count) + num_leds * (1 byte ID)
        uint8_t expected_len = 1 + num_leds;
        if (args_len < expected_len) {
            memset(response, 0x00, *response_len);
            return false;
        }

        // Build response with all LED colors
        uint8_t offset = 0;
        for (uint8_t i = 0; i < num_leds; i++) {
            // Extract LED ID
            uint8_t led_id = args[1 + i];
            
            if (led_id >= servo2040::NUM_LEDS) {
                memset(response, 0x00, *response_len);
                return false;
            }
            
            // Get current RGB state
            WS2812::RGB color = leds->get(led_id);
            
            // Pack into response
            response[offset] = color.r;
            response[offset + 1] = color.g;
            response[offset + 2] = color.b;
            offset += 3;
        }

        return true;
    }
};

#endif // GET_LEDS_HPP