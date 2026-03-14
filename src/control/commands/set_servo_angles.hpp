#ifndef SET_SERVO_ANGLES_HPP
#define SET_SERVO_ANGLES_HPP

#include "control/command.hpp"
#include "hardware/reader.hpp"

using namespace servo;


class SetServoAnglesCommand : public Command {

private:

    ServoCluster* servos;

public:

    SetServoAnglesCommand(ServoCluster* s) : servos(s) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        /*
        * Command format:
        * - 1 byte for number of servos to set
        * - 1 byte for each servo id
        * - 4 bytes (float) for each servo angle
        * 1 + n + 4*n total bytes
        */

        *response_len = 1;

        // Need at least 1 byte for count
        if (args_len < 1) {
            response[0] = 0x00;
            return false;
        }

        // Extract number of servos
        uint8_t num_servos = args[0];
        
        // Expected size: 1 byte (count) + num_servos * (1 byte id + 4 bytes angle)
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

            // Ensure servo id is within valid range
            if (servo_id >= servo2040::NUM_SERVOS) {
                response[0] = 0x00;
                return false;
            }

            float angle;
            memcpy(&angle, &args[offset], sizeof(float));
            offset += sizeof(float);

            servos->value(servo_id, angle, false);
        }
        
        servos->load();
        response[0] = 0x01;

        return true;
    }
};

#endif // SET_SERVO_ANGLES_HPP
