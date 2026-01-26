import serial
import struct
import time


# UART
PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

# HDLC
SOF = 0xAA
EOF = 0x55

# Communication loop
HZ = 50.0
PERIOD = 1.0 / HZ

class Protocol:

    def __init__(self, port: str, baud: int, timeout=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def close(self):
        self.ser.close()

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
                crc &= 0xFFFF
        return crc

    def send(self, opcode: int, data: bytes = b''):
        payload = bytes([opcode]) + data
        length = len(payload)

        frame = bytearray()
        frame.append(SOF)
        frame.append(length)
        frame.extend(payload)

        crc = self.crc16(bytes([length]) + payload)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        frame.append(EOF)

        self.ser.write(frame)

    def read_frame(self):
        # Simple blocking receiver
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b[0] == SOF:
                break

        length = self.ser.read(1)
        if not length:
            return None
        length = length[0]

        payload = self.ser.read(length)
        if len(payload) != length:
            return None

        crc_bytes = self.ser.read(2)
        if len(crc_bytes) != 2:
            return None

        eof = self.ser.read(1)
        if eof != bytes([EOF]):
            return None

        rx_crc = crc_bytes[0] | (crc_bytes[1] << 8)
        calc_crc = self.crc16(bytes([length]) + payload)

        if rx_crc != calc_crc:
            return None

        opcode = payload[0]
        data = payload[1:]
        return opcode, data


if __name__ == '__main__':

    proto = Protocol(PORT, BAUD_RATE)

    try:
        next_tick = time.monotonic()

        while True:

            # Send request (no payload, just a random opcode with no argument e.g. get_voltage)
            opcode = 0x5F
            print(f"Sending opcode 0x{opcode:x}...")
            proto.send(opcode)

            # Blocking read of response
            frame = proto.read_frame()
            if frame is not None:
                opcode, data = frame
                if len(data) == 4:
                    value = struct.unpack("<f", data)[0]
                    print(f"Response: {value:.2f} V")

            # Enforce 50 Hz
            next_tick += PERIOD
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        proto.close()
