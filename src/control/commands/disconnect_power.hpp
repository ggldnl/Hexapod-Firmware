#ifndef DISCONNECT_POWER_HPP
#define DISCONNECT_POWER_HPP

#include "control/command.hpp"
#include "hardware/power.hpp"


class DisconnectPowerCommand : public Command {

private:

    PowerTrace* power;

public:

    DisconnectPowerCommand(PowerTrace* p) : power(p) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        (void)args;
        (void)args_len;

        // Cut power from the servos
        power->off();

        // No response data
        *response_len = 1;
        response[0] = 0x01;

        return true;
    }
};

#endif // DISCONNECT_POWER_HPP
