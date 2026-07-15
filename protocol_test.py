"""
Standalone serial smoke test for the Servo2040 firmware.

Mirrors the wire protocol in src/communication/protocol.hpp by hand and pokes a
connected board with read-only queries, so you can check "is the board alive and
speaking the protocol?" over the UART with nothing installed but pyserial.

It only ever reads (telemetry/voltage/current); it never enables the robot or
moves a servo. For the full high-level client (enable, gaits, velocities,
provisioning) use the `hexapod` package in the Hexapod-Controller repo.

    python3 protocol_test.py            # poll telemetry on /dev/ttyAMA0

Frame layout (see protocol.hpp for the authoritative description):

    SOF | LEN | opcode | payload[LEN] | CRC lo | CRC hi
    0xAA  1B     1B       LEN bytes      2 bytes (CRC16-CCITT)

  - LEN counts the payload only; the opcode is NOT included.
  - CRC16-CCITT (poly 0x1021, init 0xFFFF) over LEN + opcode + payload,
    little-endian. There is no end-of-frame byte: LEN bounds the frame.
  - All multi-byte fields are little-endian; floats are IEEE-754.
"""
import struct
import time

import serial

# UART (must match cfg::BAUD in src/core/config.hpp)
PORT = "/dev/ttyAMA0"
BAUD_RATE = 921600

# Framing
SOF = 0xAA
MAX_PAYLOAD = 128

# Poll rate for this test (independent of the board's 50 Hz control loop)
HZ = 5.0
PERIOD = 1.0 / HZ

# Opcodes (subset; read-only queries + the error reply)
GET_TELEMETRY = 0x40  # () -> TelemetryReply  "<Bfffff"
GET_VOLTAGE = 0x41    # () -> VoltageReply    "<f"
GET_CURRENT = 0x42    # () -> CurrentReply    "<f"
ERROR = 0xEE          # board-initiated ErrorReply "<B"

# proto::State, for pretty-printing telemetry
STATE_NAMES = {
    0: "SETUP",
    1: "IDLE",
    2: "WALK",
    3: "SHUTDOWN",
    4: "FAULT",
    5: "OFF",
}


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

    def send(self, opcode: int, data: bytes = b""):
        length = len(data)  # payload length; opcode is not counted

        frame = bytearray()
        frame.append(SOF)
        frame.append(length)
        frame.append(opcode)
        frame.extend(data)

        crc = self.crc16(bytes([length, opcode]) + data)
        frame.append(crc & 0xFF)         # CRC lo
        frame.append((crc >> 8) & 0xFF)  # CRC hi

        self.ser.write(frame)

    def read_frame(self):
        """Blocking receiver for one valid frame. Returns (opcode, payload) or None."""
        # Sync to start-of-frame
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
        if length > MAX_PAYLOAD:  # implausible length -> give up, caller retries
            return None

        opcode = self.ser.read(1)
        if not opcode:
            return None
        opcode = opcode[0]

        payload = self.ser.read(length)
        if len(payload) != length:
            return None

        crc_bytes = self.ser.read(2)
        if len(crc_bytes) != 2:
            return None

        rx_crc = crc_bytes[0] | (crc_bytes[1] << 8)
        calc_crc = self.crc16(bytes([length, opcode]) + payload)
        if rx_crc != calc_crc:
            return None

        return opcode, payload


def describe(opcode: int, data: bytes) -> str:
    """Human-readable one-liner for a reply frame."""
    if opcode == GET_TELEMETRY and len(data) == 21:
        state, ox, oy, oyaw, volts, amps = struct.unpack("<Bfffff", data)
        name = STATE_NAMES.get(state, f"?{state}")
        return (f"[{name:8}] odom=({ox:7.1f}, {oy:7.1f}) mm  "
                f"yaw={oyaw:6.1f} deg  {volts:5.2f} V  {amps:5.2f} A")
    if opcode in (GET_VOLTAGE, GET_CURRENT) and len(data) == 4:
        (value,) = struct.unpack("<f", data)
        unit = "V" if opcode == GET_VOLTAGE else "A"
        return f"{value:.2f} {unit}"
    if opcode == ERROR and len(data) == 1:
        return f"ERROR status=0x{data[0]:02X}"
    return f"opcode=0x{opcode:02X} data={data.hex()}"


if __name__ == "__main__":

    proto = Protocol(PORT, BAUD_RATE)
    print(f"Polling telemetry on {PORT} @ {BAUD_RATE} baud (Ctrl-C to stop)...")

    try:
        next_tick = time.monotonic()

        while True:

            proto.send(GET_TELEMETRY)
            frame = proto.read_frame()
            if frame is not None:
                print(describe(*frame))
            else:
                print("(no reply)")

            next_tick += PERIOD
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        proto.close()
