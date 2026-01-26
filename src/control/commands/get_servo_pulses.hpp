#ifndef GET_SERVO_PULSES_HPP
#define GET_SERVO_PULSES_HPP

#include "control/command.hpp"
#include "hardware/reader.hpp"

using namespace servo;


class GetServoPulsesCommand : public Command {

private:

    ServoCluster* servos;

public:

    GetServoPulsesCommand(ServoCluster* s) : servos(s) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        /*
        * Command format:
        * - 1 byte for number of servos to set
        * - 1 byte for each servo id
        * 1 + n total bytes
        */

        // Extract number of servos
        uint8_t num_servos = args[0];  // args_len > 1 otherwise the dispatcher won't try to execute the command

        // Set response length (4 bytes per servo)
        *response_len = num_servos * sizeof(float);
        
        // Expected size: 1 byte (count) + num_servos * (1 byte ID)
        uint8_t expected_len = 1 + num_servos;
        if (args_len < expected_len) {
            memset(response, 0x00, *response_len);
            return false;
        }

        // Build response with all servo pulses
        uint8_t offset = 0;
        for (uint8_t i = 0; i < num_servos; i++) {

            // Extract servo ID
            uint8_t servo_id = args[1 + i];
            
            // Get current pulse
            float pulse = servos->pulse(servo_id);
            
            // Pack into response
            memcpy(&response[offset], &pulse, sizeof(float));
            offset += sizeof(float);
        }

        return true;
    }
};

#endif // GET_SERVO_PULSE_HPP
