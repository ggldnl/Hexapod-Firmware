#ifndef GET_CURRENT_HPP
#define GET_CURRENT_HPP

#include "control/command.hpp"
#include "hardware/reader.hpp"


class GetCurrentCommand : public Command {

private:

    AnalogReader* reader;

public:

    GetCurrentCommand(AnalogReader* r) : reader(r) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        (void)args;      // Unused
        (void)args_len;  // Unused

        *response_len = sizeof(float);

        if (reader == nullptr) {
            memset(response, 0x00, *response_len);
            return false;
        }

        float current = reader->readCurrent();
        memcpy(response, &current, *response_len);

        return true;
    }
};

#endif // GET_CURRENT_HPP
