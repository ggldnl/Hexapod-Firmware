#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <cstdint>
#include "hardware/uart.h"

// UART config
static uart_inst_t* UART_ID = uart1;
static constexpr uint32_t BAUD_RATE   = 115200;
static constexpr uint UART_TX_PIN     = 20;
static constexpr uint UART_RX_PIN     = 21;

// HDLC / framing
static constexpr uint8_t COMMAND_START = 0xAA;
static constexpr uint8_t COMMAND_END   = 0xFF;

// Automatic current cutoff
// If the current exceeds CURRENT_THRESHOLD, power to the servos will be cut
static constexpr uint AUTO_DISCONNECT_PIN = 19;
static constexpr float CURRENT_THRESHOLD  = 6.0f;

// Opcodes
static constexpr uint8_t GET_VOLTAGE_COMMAND        = 0x01;
static constexpr uint8_t GET_CURRENT_COMMAND        = 0x02;
static constexpr uint8_t READ_SENSOR_COMMAND        = 0x03;
static constexpr uint8_t SET_LED_COMMAND            = 0x04;
static constexpr uint8_t SET_LEDS_COMMAND           = 0x05;
static constexpr uint8_t GET_LED_COMMAND            = 0x06;
static constexpr uint8_t GET_LEDS_COMMAND           = 0x07;
static constexpr uint8_t ATTACH_SERVOS_COMMAND      = 0x08;
static constexpr uint8_t DETACH_SERVOS_COMMAND      = 0x09;
static constexpr uint8_t SET_SERVO_PULSE_COMMAND    = 0x0A;
static constexpr uint8_t SET_SERVO_PULSES_COMMAND   = 0x0B;
static constexpr uint8_t SET_SERVO_ANGLE_COMMAND    = 0x0C;
static constexpr uint8_t SET_SERVO_ANGLES_COMMAND   = 0x0D;
static constexpr uint8_t GET_SERVO_PULSE_COMMAND    = 0x0E;
static constexpr uint8_t GET_SERVO_PULSES_COMMAND   = 0x0F;
static constexpr uint8_t GET_SERVO_ANGLE_COMMAND    = 0x10;
static constexpr uint8_t GET_SERVO_ANGLES_COMMAND   = 0x11;
static constexpr uint8_t CONNECT_POWER_COMMAND      = 0x12;
static constexpr uint8_t DISCONNECT_POWER_COMMAND   = 0x13;

#endif // CONFIG_HPP
