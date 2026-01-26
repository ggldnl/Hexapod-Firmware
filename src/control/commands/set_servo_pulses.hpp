#ifndef SET_SERVO_PULSES_HPP
#define SET_SERVO_PULSES_HPP

#include "control/command.hpp"
#include "hardware/reader.hpp"

using namespace servo;


class SetServoPulsesCommand : public Command {

private:

    ServoCluster* servos;

public:

    SetServoPulsesCommand(ServoCluster* s) : servos(s) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        /*
        * Command format:
        * - 1 byte for number of servos to set
        * - 1 byte for each servo id
        * - 4 bytes (float) for each servo pulse
        * 1 + n + 4*n total bytes
        */

        *response_len = 1;

        // Extract number of servos
        uint8_t num_servos = args[0];
        
        // Expected size: 1 byte (count) + num_servos * (1 byte id + 4 bytes pulse)
        uint8_t expected_len = 1 + num_servos * 5;
        if (args_len < expected_len) {
            response[0] = 0x00;
            return false;
        }

        // Process each servo
        uint8_t offset = 1;  // start after the count byte
        for (uint8_t i = 0; i < num_servos; ++i) {

            uint8_t servo_id = args[offset];
            offset += 1;

            float pulse;
            memcpy(&pulse, &args[offset], sizeof(float));
            offset += sizeof(float);

            // Ensure servo id is within valid range
            if (servo_id >= servo2040::NUM_SERVOS) {
                response[0] = 0x00;
                return false;
            }

            servos->pulse(servo_id, pulse);
        }

        response[0] = 0x01;

        return true;
    }
};

#endif // SET_SERVO_PULSES_HPP
