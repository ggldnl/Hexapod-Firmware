#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "hardware/reader.hpp"
#include "hardware/power.hpp"
#include "HDLC/protocol.hpp"
#include "config.hpp"

#include "control/command.hpp"
#include "control/dispatcher.hpp"
#include "control/commands/attach_servos.hpp"
#include "control/commands/detach_servos.hpp"
#include "control/commands/get_voltage.hpp"
#include "control/commands/get_current.hpp"
#include "control/commands/connect_power.hpp"
#include "control/commands/disconnect_power.hpp"
#include "control/commands/set_servo_angle.hpp"
#include "control/commands/get_servo_angle.hpp"
#include "control/commands/set_servo_angles.hpp"
#include "control/commands/get_servo_angles.hpp"
#include "control/commands/set_servo_pulse.hpp"
#include "control/commands/get_servo_pulse.hpp"
#include "control/commands/set_servo_pulses.hpp"
#include "control/commands/get_servo_pulses.hpp"
#include "control/commands/set_led.hpp"
#include "control/commands/get_led.hpp"
#include "control/commands/set_leds.hpp"
#include "control/commands/get_leds.hpp"
#include "control/commands/read_sensor.hpp"

using namespace plasma;
using namespace servo;


class CommandProtocol : public Protocol {

private:
    static const size_t MAX_RESPONSE_SIZE = 64;
    Dispatcher* _dispatcher;

public:
    explicit CommandProtocol(uart_inst_t* uart, Dispatcher* dispatcher)
        : Protocol(uart), _dispatcher(dispatcher) {}

protected:
    void onFrame(
        uint8_t opcode,
        const uint8_t* data,
        uint8_t data_len
    ) override {

        uint8_t response_len = 0;
        uint8_t response_buffer[MAX_RESPONSE_SIZE];
        bool status = _dispatcher->dispatch(opcode, data, data_len,
                                        response_buffer, &response_len);

        if (status) {
            // Command executed successfully
            sendFrame(opcode, response_buffer, response_len);
        } else {
            // Send error response frame
            uint8_t error_code = 0xFF;
            sendFrame(0xFF, &error_code, 1);
        }
    }
};

int main() {
    
    // Initialize stdio (optional, for debugging)
    stdio_init_all();

    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Optional UART configuration
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    // Shared hardware control objects pool
    ServoCluster servos(pio0, 0, servo2040::SERVO_1, servo2040::NUM_SERVOS);
    WS2812 leds(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);
    PowerTrace power(AUTO_DISCONNECT_PIN);
    AnalogReader reader;

    servos.init();
    leds.start();

    // Create commands assigning the hardware resources they need to handle
    AttachServosCommand attachServosCommand(&servos);
    DetachServosCommand detachServosCommand(&servos);
    GetVoltageCommand getVoltageCommand(&reader);
    GetCurrentCommand getCurrentCommand(&reader);
    ConnectPowerCommand connectPowerCommand(&power);
    DisconnectPowerCommand disconnectPowerCommand(&power);
    SetServoAngleCommand setServoAngleCommand(&servos);
    GetServoAngleCommand getServoAngleCommand(&servos);
    SetServoAnglesCommand setServoAnglesCommand(&servos);
    GetServoAnglesCommand getServoAnglesCommand(&servos);
    SetServoPulseCommand setServoPulseCommand(&servos);
    GetServoPulseCommand getServoPulseCommand(&servos);
    SetServoPulsesCommand setServoPulsesCommand(&servos);
    GetServoPulsesCommand getServoPulsesCommand(&servos);
    SetLEDCommand setLEDCommand(&leds);
    GetLEDCommand getLEDCommand(&leds);
    SetLEDsCommand setLEDsCommand(&leds);
    GetLEDsCommand getLEDsCommand(&leds);
    ReadSensorCommand readSensorCommand(&reader);

    // Register commands that will be dispatched on frame reception
    Dispatcher dispatcher;
    dispatcher.registerCommand(ATTACH_SERVOS_COMMAND, &attachServosCommand);
    dispatcher.registerCommand(DETACH_SERVOS_COMMAND, &detachServosCommand);
    dispatcher.registerCommand(GET_VOLTAGE_COMMAND, &getVoltageCommand);
    dispatcher.registerCommand(GET_CURRENT_COMMAND, &getCurrentCommand);
    dispatcher.registerCommand(CONNECT_POWER_COMMAND, &connectPowerCommand);
    dispatcher.registerCommand(DISCONNECT_POWER_COMMAND, &disconnectPowerCommand);
    dispatcher.registerCommand(SET_SERVO_ANGLE_COMMAND, &setServoAngleCommand);
    dispatcher.registerCommand(GET_SERVO_ANGLE_COMMAND, &getServoAngleCommand);
    dispatcher.registerCommand(SET_SERVO_ANGLES_COMMAND, &setServoAnglesCommand);
    dispatcher.registerCommand(GET_SERVO_ANGLES_COMMAND, &getServoAnglesCommand);
    dispatcher.registerCommand(SET_SERVO_PULSE_COMMAND, &setServoPulseCommand);
    dispatcher.registerCommand(GET_SERVO_PULSE_COMMAND, &getServoPulseCommand);
    dispatcher.registerCommand(SET_SERVO_PULSES_COMMAND, &setServoPulsesCommand);
    dispatcher.registerCommand(GET_SERVO_PULSES_COMMAND, &getServoPulsesCommand);
    dispatcher.registerCommand(SET_LED_COMMAND, &setLEDCommand);
    dispatcher.registerCommand(GET_LED_COMMAND, &getLEDCommand);
    dispatcher.registerCommand(SET_LEDS_COMMAND, &setLEDsCommand);
    dispatcher.registerCommand(GET_LEDS_COMMAND, &getLEDsCommand);
    dispatcher.registerCommand(READ_SENSOR_COMMAND, &readSensorCommand);

    // Display an animation with the LEDs to signal that the robot is ready
    uint8_t response[1];  // we know the SET_LED_COMMAND will have response_len = 1
    uint8_t response_len;
    for (uint8_t i = 0; i < servo2040::NUM_LEDS; ++i) {
        uint8_t led_data[] = {i, 208, 107, 51};  // LED index and RGB values
        dispatcher.dispatch(SET_LED_COMMAND, led_data, 4, response, &response_len);
        sleep_ms(50);
        
        uint8_t led_off[] = {i, 0, 0, 0};
        dispatcher.dispatch(SET_LED_COMMAND, led_off, 4, response, &response_len);
    }

    // Create protocol instance
    CommandProtocol proto(UART_ID, &dispatcher);
    
    // Main loop
    while (true) {
        // Poll UART RX
        while (uart_is_readable(UART_ID)) {
            uint8_t byte = uart_getc(UART_ID);
            proto.onByte(byte);
        }

        // Optional: small sleep to reduce power / CPU usage
        tight_loop_contents();
    }
}
