#ifndef CONNECT_POWER_HPP
#define CONNECT_POWER_HPP

#include "control/command.hpp"
#include "hardware/power.hpp"


class ConnectPowerCommand : public Command {

private:

    PowerTrace* power;

public:

    ConnectPowerCommand(PowerTrace* p) : power(p) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        (void)args;
        (void)args_len;

        // Give power to the servos
        power->on();

        // No response data
        *response_len = 1;
        response[0] = 0x01;

        return true;
    }
};

#endif // CONNECT_POWER_HPP
