# Hexapod Firmware

This repository contains the firmware for the Servo2040 board that drives the hexapod.

The Servo2040 runs the whole control loop: finite-state machine (stand/walk/sit), gait generator and inverse kinematics. The Pi only streams high-level setpoints (a velocity, a gait, a body pose) and reads telemetry back.

For a complete overview of the project refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod.git). See also the [Controller](https://github.com/ggldnl/Hexapod-Controller.git) (the Pi-side `hexapod` client that talks to this board) and the [Simulation](https://github.com/ggldnl/Hexapod-Simulation.git) (which runs this exact firmware on the host).

Below you will find how to build and deploy the code, and how the communication protocol between the two boards works.

## 🗂️ Repository layout

Everything is header-only C++17 and rooted at `src/`, so every include shows its folder (e.g. `#include "core/robot.hpp"`).

```
src/
  main.cpp                    # entry point: drain UART -> dispatch -> execute
  core/
    config.hpp                # geometry, gaits, pins, limits (baked defaults + provisionable overrides)
    robot.hpp                 # the state machine and the robot's intent API (enable/walk/pose ...)
    gait.hpp                  # foot-trajectory generator (constant-cadence, velocity-scaled stride)
    kinematics.hpp            # per-leg forward/inverse kinematics
    router.hpp                # turns decoded frames into a call on the robot, builds the reply
  communication/
    protocol.hpp              # wire protocol: opcodes, payload structs, framing (CRC + parser)
    hardware.hpp              # abstract hardware interface the robot drives (so the core is testable)
    servo2040_hal.hpp         # the only file that touches the Pimoroni SDK (servos, LEDs, power, ADC)
  utils/
    math.hpp                  # small geometry helpers (Vec3, deg/rad, clamp, angle wrap)
test/                         # host-side tests, pure C++ (no board, no SDK) -> ./test/run.sh
protocol_test.py              # serial test (pyserial only)
```

Everything under `core/` (plus `communication/protocol.hpp` and `utils/`) is SDK-free and compiles on your PC against a fake hardware interface, which is what the host tests exercise. Only `servo2040_hal.hpp` pulls in the Pico SDK/Pimoroni libraries.

## 🛠️ Build and deployment

Before you start, take a look at this [template](https://github.com/pimoroni/pico-boilerplate?tab=readme-ov-file#before-you-start). This served as the starting point for the firmware.

It's easiest to make a `pico` directory that holds the SDK, the Pimoroni libraries and this repo side by side, so the build can find them:

```
pico
├── Hexapod-Firmware
├── pico-sdk
└── pimoroni-pico
```

Feel free to use another name for the `pico` directory; I'll use this out of simplicity.

### Prepare the build environment

Install the build requirements:

```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi build-essential
```

### Download the Pico SDK

Download the Pico SDK into the `pico` directory:

```bash
cd pico
git clone https://github.com/raspberrypi/pico-sdk
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=`pwd`
cd ../
```

`PICO_SDK_PATH` set this way lasts only for the session. To make it persistent, add it to your `.bashrc`:

```bash
echo 'export PICO_SDK_PATH="/path/to/pico-sdk"' >> ~/.bashrc
```

### Download the Pimoroni libraries

Download the Pimoroni libraries into the `pico` directory:

```bash
git clone https://github.com/pimoroni/pimoroni-pico
```

### Clone the project

```bash
git clone https://github.com/ggldnl/Hexapod-Firmware
```

`CMakeLists.txt` includes `pico_sdk_import.cmake` and `pimoroni_pico_import.cmake` (both shipped in this repo). They locate the SDK and the Pimoroni libraries using `PICO_SDK_PATH` (and, by default, expect `pimoroni-pico` next to `pico-sdk`). If you don't want to set `PICO_SDK_PATH` and you use VS Code, you can pass the path to CMake from `.vscode/settings.json` instead.

### Build

Create a build directory in the root of the project and compile:

```bash
cd Hexapod-Firmware
mkdir build
cd build
cmake ..
make
```

This produces **`hexapod_firmware.uf2`** inside the `build` directory.

### Deploy

- Connect the Servo2040 board to the computer;
- Hold down the `boot/user` button, press `reset` at the same time, then release both. The Servo2040 appears as a USB drive;
- Drag and drop `hexapod_firmware.uf2` onto that drive. The board reboots and runs the firmware.

If you built on the Raspberry Pi over SSH:

- Connect the Servo2040 to the Pi via USB;
- Enter boot mode as above (`boot/user` + `reset`). It appears as a block device (`lsblk`);
- Find the new drive (e.g. `/dev/sda1`, mounted at `/media/<username>/RPI-RP2`);
- From `build`, run `mv hexapod_firmware.uf2 /media/<username>/RPI-RP2`. The board reboots and runs the firmware.

## 🧪 Host tests

The behaviour layer is SDK-free, so it can be built and run on your PC with plain `g++` (no board, no Pico SDK):

```bash
./test/run.sh          # or  CXX=clang++ ./test/run.sh
```

This compiles each `test/test_*.cpp` against a fake hardware interface and exercises kinematics, gait, the state machine and the protocol framing.

If you have the real board connected to your Raspberry Pi, you can run the `protocol_test.py` script to check if it is alive and responding correctly. It only ever reads (telemetry/voltage/current); it never enables the robot or moves a servo.

Wire up the board (see [Connection](#-connection)), then, on the Pi:

```bash
pip install pyserial
python3 protocol_test.py
```

It polls `GetTelemetry` a few times a second and prints the decoded reply:

```
Polling telemetry on /dev/ttyAMA0 @ 921600 baud (Ctrl-C to stop)...
[OFF     ] odom=(    0.0,     0.0) mm  yaw=   0.0 deg   7.42 V   0.03 A
[OFF     ] odom=(    0.0,     0.0) mm  yaw=   0.0 deg   7.41 V   0.03 A
...
```

`(no reply)` means nothing came back, usually a swapped TX/RX pair, the wrong baud, or the serial port not enabled on the Pi. The port and baud are constants at the top of the file (`PORT = "/dev/ttyAMA0"`, `BAUD_RATE = 921600` according to the default config); edit them if yours differ.

## 🔌 Connection

The board talks to the Pi over UART. The firmware uses **UART1 on GP20 (TX) / GP21 (RX)** (`cfg::UART_TX_PIN` / `cfg::UART_RX_PIN`), which are broken out on the Servo2040's SDA/SCL (Qwiic) header. Wire it as a crossover (each side's TX goes to the other side's RX) and share ground:

| Raspberry Pi   | Servo2040        |
|----------------|------------------|
| 5V             | 5V               |
| GND            | GND              |
| GPIO14 (TXD)   | GP21 / SCL (RX)  |
| GPIO15 (RXD)   | GP20 / SDA (TX)  |

> Double-check TX/RX against your board's silkscreen: the firmware transmits on GP20 and receives on GP21. If nothing comes back, a swapped TX/RX pair is the usual cause.

The link runs at **921600 baud** (`cfg::BAUD`).

Enable the hardware UART on the Pi:
- `sudo raspi-config` > `Interface Options` > `Serial Port`
- Would you like a login shell over serial? > **No**
- Would you like the serial port hardware enabled? > **Yes**
- Save and reboot.

## 📡 Communication protocol

Commands travel between the Raspberry Pi and the Servo2040 over the serial link. The authoritative definition is [`src/communication/protocol.hpp`](src/communication/protocol.hpp); the Pi-side Python client mirrors it by hand and is checked byte-for-byte against this C++ encoder in the Controller's tests.

### Frame layout

A compact, HDLC-style frame:

| SOF  | LEN | opcode | payload  | CRC lo | CRC hi |
|------|-----|--------|----------|--------|--------|
| 1B   | 1B  | 1B     | LEN B    | 1B     | 1B     |


| Field   | Size | Description                                                       |
|---------|------|-------------------------------------------------------------------|
| SOF     | 1    | Start-of-frame marker (`0xAA`)                                     |
| LEN     | 1    | Payload length only. Opcode is **not** counted (`0..128`)    |
| opcode  | 1    | Command identifier                                                |
| payload | n    | Arguments (binary, little-endian)                                 |
| CRC     | 2    | CRC16-CCITT (poly `0x1021`, init `0xFFFF`) over `LEN + opcode + payload`, little-endian |

Total frame size is `LEN + 5`. There is **no end-of-frame byte**: `LEN` bounds the frame and the CRC validates it. On a bad CRC (or a `LEN` larger than 128) the receiver drops the frame and rescans for the next SOF. All multi-byte fields are little-endian; floats are IEEE-754.

### Message kinds

Every opcode is one of two kinds:

- **Fire-and-forget**: no reply. Setpoints and lifecycle commands. Each one also pets a command watchdog: if the board hears nothing for `cfg::WATCHDOG_TIMEOUT_MS` (500 ms) while moving, it stops on its own.
- **Request/reply**: exactly one reply frame, carrying the **same opcode** as the request (queries), or an `Error` frame (`0xEE`) if the request was refused or malformed.

### Instruction set

The high nibble of the opcode groups it by purpose.

**Low-level debug (`0x0x`)**: request/reply, acked with an `AckReply` (`<B` status).

| Operation | OpCode | Payload (`struct` fmt)         | Reply     |
|-----------|--------|--------------------------------|-----------|
| Jog Servo | `0x01` | `<BH` channel, pulse_us (0=release) | AckReply |

**Provisioning (`0x1x`)**: request/reply, acked. The Pi pushes the full runtime config at connect, one section per message; these are honoured only while de-energized (OFF/FAULT) so nothing reconfigures mid-motion.

| Operation | OpCode | Payload (`struct` fmt) |
|-----------|--------|------------------------|
| Provision Body       | `0x10` | `<fffffff` link lengths, coxa offset, standing height, stance radius, cycle time |
| Provision Mounts     | `0x11` | `<24f` per leg: x, y, z, yaw |
| Provision Direction  | `0x12` | `<18f` per servo: +/-1       |
| Provision Trim       | `0x13` | `<18f` per servo: trim deg   |
| Provision Ranges     | `0x14` | `<6f` per joint type: min, max deg |
| Provision Gaits      | `0x15` | `<12f` per gait: duty, step height, max stride, overlap |
| Provision Limits     | `0x16` | `<ffffffff` velocity clamps, over-current, low-voltage |
| Provision Body Pose  | `0x17` | `<12f` per axis: min, max    |
| Provision Servo Cal  | `0x18` | `<54H` per servo: min, mid, max µs |
| Provision Pins       | `0x19` | `<18B` physical pin per logical channel |

**Setpoints & lifecycle (`0x3x`)**: fire-and-forget, no reply.

| Operation | OpCode | Payload (`struct` fmt) |
|-----------|--------|------------------------|
| Set Velocity  | `0x30` | `<fff` vx, vy (mm/s), wz (deg/s) |
| Set Body Pose | `0x31` | `<ffffff` x, y, z (mm), roll, pitch, yaw (deg) |
| Set Gait      | `0x32` | `<B` gait id (0 tripod, 1 wave, 2 ripple) |
| Enable        | `0x33` | none; run stand-up, end IDLE |
| Shutdown      | `0x34` | none; run sit-down, end OFF  |
| Stop          | `0x35` | none; zero velocity (soft stop) |
| Set LED       | `0x36` | `<BBBBf` mode, r, g, b, blink freq |
| Heartbeat     | `0x37` | none; keepalive, pets the watchdog |

**Queries (`0x4x`)**: request/reply; the board answers with the same opcode.

| Operation | OpCode | Reply (`struct` fmt) |
|-----------|--------|----------------------|
| Get Telemetry | `0x40` | `<Bfffff` state, odom x, odom y, odom yaw, voltage, current |
| Get Voltage   | `0x41` | `<f` volts   |
| Get Current   | `0x42` | `<f` amps    |
| Get Joints    | `0x43` | `<18f` servo-space angles (deg), leg-major |

**Board-initiated**

| Operation | OpCode | Payload (`struct` fmt) |
|-----------|--------|------------------------|
| Error | `0xEE` | `<B` status code |

Status codes: `0x00` rejected (wrong state), `0x01` OK, `0x02` unreachable (IK out of range), `0x03` bad opcode, `0x04` bad length.

### States

Telemetry reports the state machine (`proto::State`):

| Value | State    | Meaning |
|-------|----------|---------|
| 0 | `SETUP`    | rising: stand-up animation running |
| 1 | `IDLE`     | standing, zero velocity |
| 2 | `WALK`     | executing a gait |
| 3 | `SHUTDOWN` | lowering: sit-down animation running |
| 4 | `FAULT`   | emergency-stopped (over-current / low-voltage); needs Enable to recover |
| 5 | `OFF`      | de-energized standby, awaiting Enable |

The stand-up and sit-down animations are uninterruptible; only a fault can break in.

## 🧩 Architecture

`main.cpp` is a small scheduler that does two things forever:

1. **Transport**: drain the UART, feeding each byte to a `proto::FrameParser`. When a complete, CRC-valid frame arrives, `router::dispatch` turns it into a call on the `robot::Robot` intent API (`enable()`, `set_velocity()`, `set_gait()`, ...) and, for a query, writes a reply frame back.
2. **Control**: at a fixed `cfg::CONTROL_RATE_HZ` (50 Hz) tick, call `robot.update(dt)`, which advances the state machine, runs the gait, solves IK, and writes the 18 servo angles through the hardware interface.

The robot reasons in a clean kinematic frame and only maps to servo space (direction/trim/clamp, degrees) at the very end. Because that whole behaviour layer talks to hardware through the abstract `hw::Interface`, it builds and is tested on the host against a fake. `servo2040_hal.hpp` is the only piece that needs a board.

Most of `config.hpp` is a baked default that also makes the board fully functional standalone; the values the Pi is allowed to override at connect (geometry, kinematic map, gaits, limits) are mutable globals that the provisioning opcodes assign into.

### Adding a new opcode

1. Add a value to `enum class Opcode` in `protocol.hpp` (mind the nibble groups).
2. If it carries data, add a `#pragma pack`-ed payload `struct` and a `static_assert` pinning its size; note the Python `struct` format in the `// py:` comment.
3. If it takes no reply, list it in `kind_of()` as fire-and-forget.
4. Handle it in `router::dispatch` (validate `len`, decode the payload, call the robot, build the reply/ack).
5. Mirror the opcode and payload format in the Pi-side `hexapod` client (Controller repo) so both ends stay in lockstep.

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.
