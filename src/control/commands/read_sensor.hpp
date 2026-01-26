#ifndef READ_SENSOR_HPP
#define READ_SENSOR_HPP

#include "control/command.hpp"
#include "hardware/reader.hpp"


class ReadSensorCommand : public Command {

private:

    AnalogReader* reader;

public:

    ReadSensorCommand(AnalogReader* r) : reader(r) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {

        *response_len = sizeof(float);

        // We expect 1 byte for the pin we need to read from
        if (args_len != 1 || reader == nullptr) {
            // Error: return 0x00 * 4
            memset(response, 0x00, *response_len);
            return false;
        }
        
        uint8_t pin = args[0];
        float value = reader->readSensor(pin);
        memcpy(response, &value, *response_len);

        return true;
    }
};

#endif // READ_SENSOR_HPP
