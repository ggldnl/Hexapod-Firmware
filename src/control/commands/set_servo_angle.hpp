#ifndef SET_SERVO_ANGLE_HPP
#define SET_SERVO_ANGLE_HPP

#include "control/command.hpp"
#include "hardware/reader.hpp"

using namespace servo;


class SetServoAngleCommand : public Command {

private:

    ServoCluster* servos;

public:

    SetServoAngleCommand(ServoCluster* s) : servos(s) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {
        
        *response_len = 1;
        
        // We expect 1 byte for servo index and 4 bytes for the angle
        if (args_len != 5) {
            response[0] = 0x00;
            return false;
        }

        // Extract args
        uint8_t servo = args[0];
        float angle;
        memcpy(&angle, &args[1], sizeof(float));

        // Set servo angle
        servos->value(servo, angle);

        response[0] = 0x01;

        return true;
    }
};

#endif // SET_SERVO_ANGLE_HPP
