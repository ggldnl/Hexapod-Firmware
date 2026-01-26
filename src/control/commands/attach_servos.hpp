#ifndef ATTACH_SERVOS_HPP
#define ATTACH_SERVOS_HPP

#include "control/command.hpp"
#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace servo;


class AttachServosCommand : public Command {

private:

    ServoCluster* servos;

public:

    AttachServosCommand(ServoCluster* servos) : servos(servos) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        (void)args;
        (void)args_len;

        // Eanble the servos
        servos->enable_all();

        // No response data
        *response_len = 1;
        response[0] = 0x01;

        return true;
    }
};

#endif // ATTACH_SERVOS_HPP