#ifndef GET_SERVO_PULSE_HPP
#define GET_SERVO_PULSE_HPP

#include "control/command.hpp"
#include "hardware/reader.hpp"

using namespace servo;


class GetServoPulseCommand : public Command {

private:

    ServoCluster* servos;

public:

    GetServoPulseCommand(ServoCluster* s) : servos(s) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {
        
        *response_len = sizeof(float);

        // We expect 1 byte for servo index
        if (args_len != 1) {
            memset(response, 0x00, *response_len);
            return false;
        }

        // Extract args
        uint8_t servo = args[0];

        // Get servo pulse
        float pulse = servos->pulse(servo);
        
        // Pack response
        memcpy(response, &pulse, sizeof(float));

        return true;
    }
};

#endif // GET_SERVO_PULSE_HPP
