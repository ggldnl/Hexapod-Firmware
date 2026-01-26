#ifndef SET_LEDS_HPP
#define SET_LEDS_HPP

#include "control/command.hpp"
#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace plasma;


class SetLEDsCommand : public Command {

private:

    WS2812* leds;

public:

    SetLEDsCommand(WS2812* l) : leds(l) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {
        
        *response_len = 1;

        // Need at least 1 byte for count
        if (args_len < 1) {
            response[0] = 0x00;
            return false;
        }

        // Extract number of LEDs
        uint8_t num_leds = args[0];
        
        // Expected size: 1 byte (count) + num_leds * (1 byte ID + 3 bytes RGB)
        uint8_t expected_len = 1 + num_leds * 4;        
        if (args_len < expected_len) {
            response[0] = 0x00;
            return false;
        }

        // Process each LED
        uint8_t offset = 1;  // Start after the count byte
        for (uint8_t i = 0; i < num_leds; i++) {
            // Extract LED ID
            uint8_t led_id = args[offset];
            offset += 1;
            
            if (led_id >= servo2040::NUM_LEDS) {
                response[0] = 0x00;
                return false;
            }
            
            // Extract RGB values
            uint8_t r = args[offset];
            uint8_t g = args[offset + 1];
            uint8_t b = args[offset + 2];
            offset += 3;
            
            // Set the LED
            leds->set_rgb(led_id, r, g, b);
        }

        response[0] = 0x01;

        return true;
    }
};

#endif // SET_LEDS_HPP