#ifndef ANALOG_READER_HPP
#define ANALOG_READER_HPP

#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"

using namespace servo;

class AnalogReader {

private:
    Analog sen_adc;    // Shared Analog input for sensors
    Analog vol_adc;    // Shared Analog input for voltage
    Analog cur_adc;    // Shared Analog input for current
    AnalogMux mux;     // Analog multiplexer

public:
    // Constructor to initialize the AnalogReader with shared ADC and multiplexer
    AnalogReader() :
        sen_adc(servo2040::SHARED_ADC),
        vol_adc(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN),
        cur_adc(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN, servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET),
        mux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2, PIN_UNUSED, servo2040::SHARED_ADC) 
    {
        // Configure the sensor addresses with pull-downs
        for(auto i = 0u; i < servo2040::NUM_SENSORS; i++) {
            mux.configure_pulls(servo2040::SENSOR_1_ADDR + i, false, true);
        }
    }

    // Method to read the voltage from a specific sensor (1 to NUM_SENSORS)
    float readSensor(uint8_t sensorIndex) {
        if (sensorIndex >= 1 && sensorIndex <= servo2040::NUM_SENSORS) {
            mux.select(servo2040::SENSOR_1_ADDR + sensorIndex - 1);
            return sen_adc.read_voltage();
        }
        return 0.0f; // Return 0.0 for invalid sensor index
    }

    // Method to read the voltage sense
    float readVoltage() {
        mux.select(servo2040::VOLTAGE_SENSE_ADDR);
        return vol_adc.read_voltage();
    }

    // Method to read the current sense
    float readCurrent() {
        mux.select(servo2040::CURRENT_SENSE_ADDR);
        return cur_adc.read_current();
    }
};

#endif // ANALOG_READER_HPP
