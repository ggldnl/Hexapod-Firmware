#ifndef DETACH_SERVOS_HPP
#define DETACH_SERVOS_HPP

#include "control/command.hpp"
#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace servo;


class DetachServosCommand : public Command {

private:

    ServoCluster* servos;

public:

    DetachServosCommand(ServoCluster* servos) : servos(servos) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        (void)args;
        (void)args_len;
        
        // Disable the servos
        servos->disable_all();

        // No response data
        *response_len = 1;
        response[0] = 0x01;

        return true;
    }
};

#endif // DETACH_SERVOS_HPP