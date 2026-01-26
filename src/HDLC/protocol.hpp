#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <cstdint>  // uint8_t, uint16_t
#include "hardware/uart.h"


static constexpr uint8_t SOF = 0xAA;
static constexpr uint8_t EOF_ = 0x55;
static constexpr uint8_t MAX_PAYLOAD = 64;

static uint16_t crc16Update(uint16_t crc, uint8_t data) {
    crc ^= uint16_t(data) << 8;
    for (int i = 0; i < 8; ++i) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

class Protocol {
public:

    Protocol(uart_inst_t* uid) : uart_id(uid) {
        reset();
    }

    // Feed one byte at a time from UART
    void onByte(uint8_t b) {
        switch (state) {

            case State::WAIT_SOF:
                if (b == SOF) {
                    reset();
                    state = State::READ_LEN;
                }
                break;

            case State::READ_LEN:
                length = b;
                if (length == 0 || length > MAX_PAYLOAD) {
                    reset();
                    break;
                }
                calc_crc = crc16Update(calc_crc, b);
                payload_index = 0;
                state = State::READ_PAYLOAD;
                break;

            case State::READ_PAYLOAD:
                payload[payload_index++] = b;
                calc_crc = crc16Update(calc_crc, b);
                if (payload_index >= length) {
                    state = State::READ_CRC_LO;
                }
                break;

            case State::READ_CRC_LO:
                rx_crc = b;
                state = State::READ_CRC_HI;
                break;

            case State::READ_CRC_HI:
                rx_crc |= uint16_t(b) << 8;
                state = State::WAIT_EOF;
                break;

            case State::WAIT_EOF:
                if (b == EOF_ && rx_crc == calc_crc) {
                    uint8_t opcode = payload[0];
                    onFrame(opcode, payload + 1, length - 1);
                }
                reset();
                break;
        }
    }

protected:

    // Called when a valid frame received
    virtual void onFrame(uint8_t, const uint8_t*, uint8_t) {
        // To be defined in derived classes
    }

    virtual void sendFrame(uint8_t opcode, const uint8_t* data, uint8_t data_len) {
        uint8_t length = 1 + data_len;
        uint16_t crc = 0xFFFF;

        uart_putc_raw(uart_id, SOF);

        uart_putc_raw(uart_id, length);
        crc = crc16Update(crc, length);

        uart_putc_raw(uart_id, opcode);
        crc = crc16Update(crc, opcode);

        for (uint8_t i = 0; i < data_len; ++i) {
            uart_putc_raw(uart_id, data[i]);
            crc = crc16Update(crc, data[i]);
        }

        uart_putc_raw(uart_id, crc & 0xFF);
        uart_putc_raw(uart_id, crc >> 8);

        uart_putc_raw(uart_id, EOF_);
    }

private:

    uart_inst_t* uart_id;

    enum class State : uint8_t {
        WAIT_SOF,
        READ_LEN,
        READ_PAYLOAD,
        READ_CRC_HI,
        READ_CRC_LO,
        WAIT_EOF
    };

    State state;

    uint8_t length;
    uint16_t rx_crc;
    uint16_t calc_crc;
    uint8_t payload[MAX_PAYLOAD];
    uint8_t payload_index;

    void reset() {
        state = State::WAIT_SOF;
        payload_index = 0;
        length = 0;
        calc_crc = 0xFFFF;
    }
};

#endif  // PROTOCOL_HPP