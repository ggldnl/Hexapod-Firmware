#ifndef GET_VOLTAGE_HPP
#define GET_VOLTAGE_HPP

#include "control/command.hpp"
#include "hardware/reader.hpp"


class GetVoltageCommand : public Command {

private:

    AnalogReader* reader;

public:

    GetVoltageCommand(AnalogReader* r) : reader(r) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        (void)args;      // Unused
        (void)args_len;  // Unused

        *response_len = sizeof(float);

        if (reader == nullptr) {
            // Error: return 0x00 * 4
            memset(response, 0x00, *response_len);
            return false;
        }

        float voltage = reader->readVoltage();
        memcpy(response, &voltage, *response_len);

        return true;
    }
};

#endif // GET_VOLTAGE_HPP
