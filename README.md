# Hexapod Firmware

This repository contains the firmware for the Servo2040 board.

For a complete overview of the project refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod.git). Take also a look to the [repository containing the Controller's code](https://github.com/ggldnl/Hexapod-Controller.git). 

Below, you will find instructions on how to build and deploy the code and info on how the communication protocol between the two boards works.

## 🛠️ Build and deployment

Before you start, take a look at this [template](https://github.com/pimoroni/pico-boilerplate?tab=readme-ov-file#before-you-start). This served as starting point to develop the firmware.

It's easier if you make a `pico` directory or similar in which you keep the SDK, Pimoroni Libraries and this repo. This makes it easier to include libraries. At the end you will have this directory structure:

```
pico
├── Hexapod-Firmware
├── pico-sdk
└── pimoroni-pico
```

Feel free to use another name for the `pico` directory, I'll use this out of simplicity. 

### Prepare the build environment

Install build requirements:

```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi build-essential
```

### Download the pico SDK

Download the pico SDK in the `pico` directory:

```bash
cd pico
git clone https://github.com/raspberrypi/pico-sdk
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=`pwd`
cd ../
```

The `PICO_SDK_PATH` set above will only last the duration of your session. To make it persistant you can add it to your `.bashrc`.

```bash
echo 'export PICO_SDK_PATH="/path/to/pico-sdk"' >> ~/.bashrc
```

### Download the Pimoroni libraries

Download the Pimoroni libraries in the `pico` directory:

```bash
git clone https://github.com/pimoroni/pimoroni-pico
```

### Clone the project

```bash
git clone https://github.com/ggldnl/Hexapod-Firmware
```

If you have not or don't want to set `PICO_SDK_PATH` and you are using vscode, you can edit `.vscode/settings.json` to pass the path directly to CMake.

### Build

Create a build directory in the root folder of the project and compile.

```bash
mkdir build
cd build
cmake ..
make
```

Once you compile the project you will end up with a `Hexapod.uf2` file inside the `build` directory.

### Delpoy

- Connect the servo2040 board to the computer;
- Hold down the `boot/user` button, press the `reset` button at the same time, and let go of both buttons. The Servo2040 should now appear as a drive on the computer;
- Drag and drop the `Hexapod.uf2` image file to the Servo2040 drive, the device will automatically reboot and start the loaded program.

If you built the firmware on the Raspberry Pi that you will use for the Hexapod and you happen to be connected to it with ssh, you can:

- Connect the servo2040 board to the raspberry through usb;
- Hold down the `boot/user` button, press the `reset` button at the same time, and let go of both buttons. The Servo2040 should now appear as a block device when issuing `lsblk`;
- Look for the new drive (e.g. `/dev/sda1` mounted at `/media/<username>/RPI-RP2`);
- From the `build` directory, `mv Hexapod.uf2 /media/<username>/RPI-RP2`, the device will automatically reboot and start the loaded program.

## 🔌 Connection

Connect the Servo2040 board to the raspberry pi as follows:

<div align="center">

| Raspberry    | Servo2040 |
|--------------|-----------|
| 5V           | 5V        |
| GND          | GND       |
| GPIO14 (TXD) | SDA (RX)  |
| GPIO15 (RXD) | SCL (TX)  |

</div>

Remember to enable hardware uart: 
- `sudo raspi-config` > `Interface Options` > `Serial Port`
- Would you like a login shell to be accessible over serial? > `No`
- Would you like the serial port hardware to be enabled? > `Yes`
- Save and reboot.

## 📡 Communication protocol

This paragraph outlines the specifications for the communication protocol. Commands are sent from the Raspberry Pi to the Servo2040 and backwards, over a serial connection. 

### HDLC

The protocol I decided to use is essentially a compact version of HDLC (which stands for High-Level Data Link Control). 

HDLC is a communication protocol used for transmitting data between devices reliably. Originally, it was used in multi-device networks, where one device acted as the master and others as slaves. Currently, HDLC is primarily employed in point-to-point connections, such as between routers or network interfaces.

It works by sending frames like this:

<div align="center">

| SOF | LEN | OPCODE | DATA    | CRC | EOF |
|-----|-----|--------|---------|-----|-----|
| 1B  | 1B  | 1B     | nB      | 2B  | 1B  |

</div>

<div align="center">

| Field  | Size (bytes) | Description                       |
|--------|--------------|-----------------------------------|
| SOF    | 1            | Start-of-frame marker (`0xAA`)    |
| LEN    | 1            | Length of `OPCODE + DATA`         |
| OPCODE | 1            | Command identifier                |
| DATA   | n            | Arguments (binary)                |
| CRC    | 2            | CRC-16 over `LEN + OPCODE + DATA` |
| EOF    | 1            | End-of-frame marker (`0x55`)      |

</div>

CRC-16 is a 16-bit cyclic redundancy check used to detect errors in transmitted frames. When a receiver gets a frame, it recomputes the CRC-16 and compares it to the received FCS. If they differ, the frame is considered corrupted.

### Instruction set

I used the Command design pattern to dispatch commands once extracted from a message.

The following table describes the supported operations, their opcodes, the expected arguments, and the response:

<div align="center">

| Operation | OpCode | Arguments | Response |
|-----------|--------|-----------|----------|
| Get Voltage | `0x01` | None | voltage(4b) |
| Get Current | `0x02` | None | current(4b) |
| Read Sensor | `0x03` | pin (1b) | value(4b) |
| Set LED | `0x04` | pin(1b), r(1b), g(1b), b(1b) | status(1b) |
| Set LEDs | `0x05` | count(1b), [pin(1b), r(1b), g(1b), b(1b)] × count | status(1b) |
| Get LED | `0x06` | pin (1b) | r(1b), g(1b), b(1b) |
| Get LEDs | `0x07` | count(1b), [pin(1b)] × count | [r(1b), g(1b), b(1b)] × count |
| Attach Servos | `0x08` | None | status(1b) |
| Detach Servos | `0x09` | None | status(1b) |
| Set Servo Pulse Width | `0x0A` | pin(1b), pulse_width(4b) | status(1b) |
| Set Servo Pulse Widths | `0x0B` | count(1b), [pin(1b), pulse_width(4b)] × count | status(1b) |
| Set Servo Angle | `0x0C` | pin(1b), angle(4b) | status(1b) |
| Set Servo Angles | `0x0D` | count(1b), [pin(1b), angle(4b)] × count | status(1b) |
| Get Servo Pulse Width | `0x0E` | pin (1b) | pulse_width(4b) |
| Get Servo Pulse Widths | `0x0F` | count(1b), [pin(1b)] × count | pulse_width(4b) × count |
| Get Servo Angle | `0x10` | pin(1b) | angle(4b) |
| Get Servo Angles | `0x11` | count(1b), [pin(1b)] × count | angle(4b) × count |
| Connect Power | `0x12` | None | status(1b) |
| Disconnect Power | `0x13` | None | status(1b) |

</div>

The response is always guaranteed. For commands that return data (e.g. `get_voltage`), a successful execution returns the actual requested data, while a failure returns all bytes set to `0x00`. For commands that perform actions (e.g. `set_led`), a successful execution returns `0x01`, while a failure returns `0x00`. The length of arguments and responses are expressed in bytes (e.g. 4b means 4 bytes i.e. a float).

Description table:

<div align="center">

| Operation | Description |
|-----------|-------------|
| Get Voltage | Reads the voltage present on the external power line. |
| Get Current | Reads the current flowing through the external power line. |
| Read Sensor | Reads the analog value of the specified input pin. |
| Set LED | Sets the RGB color of a single LED connected to the specified pin. |
| Set LEDs | Sets the RGB color of multiple LEDs in a single command. |
| Get LED | Reads the current RGB color of the specified LED. |
| Get LEDs | Reads the current RGB color of multiple LEDs. |
| Attach Servos | Initializes and attaches all configured servo outputs. |
| Detach Servos | Detaches all servo outputs and disables signal generation. |
| Set Servo Pulse Width | Sets the pulse width for a single servo. |
| Set Servo Pulse Widths | Sets the pulse width for multiple servos. |
| Set Servo Angle | Sets the target angle for a single servo. |
| Set Servo Angles | Sets the target angle for multiple servos. |
| Get Servo Pulse Width | Reads the current pulse width of the specified servo. |
| Get Servo Pulse Widths | Reads the current pulse width of multiple servos. |
| Get Servo Angle | Reads the current angle of the specified servo. |
| Get Servo Angles | Reads the current angle of multiple servos. |
| Connect Power | Enables external power delivery to the servos. |
| Disconnect Power | Disables external power delivery to the servos. |

</div>

### Implementation details

We start defining a shared object pool. 

```cpp
// Shared hardware control objects pool
ServoCluster servos(pio0, 0, servo2040::SERVO_1, servo2040::NUM_SERVOS);
WS2812 leds(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);
PowerTrace power(AUTO_DISCONNECT_PIN);
AnalogReader reader;

servos.init();
leds.start();
```

Each command will take a reference to the object(s) it needs to work with. Commands that need, for example, to read from a sensor (internal or external), will have a reference to the `AnalogReader`, a utility class that encapsulates the logic for multiplexing and reading; the same way, commands that need to work with servos will take a reference to a unique `ServoCluster` object. This limits potential interference between commands and redundancy.

We create a `Dispatcher`, some `Command` objects and register them on the dispatcher:

```cpp
// Initialize the dispatcher
Dispatcher dispatcher;

// Create commands assigning the hardware resources they need to handle
AttachServosCommand attachServosCommand(&servos);
...
ReadSensorCommand readSensorCommand(&reader);

// Register commands
dispatcher.registerCommand(ATTACH_SERVOS_COMMAND, &attachServosCommand);
...
dispatcher.registerCommand(READ_SENSOR_COMMAND, &readSensorCommand);
```

Upon receipt of a frame, the `dispatcher` extracts the information from its payload. The first byte is the length of the rest of the payload (`opcode` + `data`). The `opcode` is used to lookup for a command among the registered ones; if a match is found, the `dispatcher` executes it with the remainig bytes in the frame (`data`) as arguments. The result is then sent back to the controlling machine as a new frame.

### Adding a new command

As an example we can add a command that simply toggles the status of a variable. It will need no argument and return the state of the variable each time it changes.

Create a new header file named `toggle_status_command.hpp` in the commands directory. Implement the class as follows:

```cpp
#ifndef TOGGLE_STATUS_COMMAND_HPP
#define TOGGLE_STATUS_COMMAND_HPP

#include "command.hpp"

class ToggleStatusCommand : public Command {

private:

    bool status;

public:

    // No hardware lirbary
    ToggleStatusCommand() : status(false) {}

    bool execute(const uint8_t* args, uint8_t args_len, 
                uint8_t* response, uint8_t* response_len) override {
        
        // We expect no input
        (void) args;
        (void) args_len;

        // Perform the action
        status = !status;

        // Build response
        *response_len = 1;
        response[0] = status;
        
        return true;
    }
};

#endif // TOGGLE_STATUS_COMMAND_HPP
```

To be a valid command, the new class must extend the `Command` base class and implement the `execute(...)` method.

Next, include the new command in your main script and register it on the dispatcher using a new opcode. Here’s how to do it:

```cpp
#include "commands/toggle_status_command.hpp"

// ...

int main() {

  // ...

  // Initialize the dispatcher
  Dispatcher dispatcher;

  // Create the commands
  // ...
  ToggleStatusCommand toggleStatusCommand();

  // Register the commands
  // ...
  dispatcher.registerCommand(0x1F, &toggleStatusCommand);  // 0x1F is a random free opcode

  // ...
}
```

Once registered, the dispatcher will automatically invoke the new `ToggleStatusCommand` when the opcode `0x1F` is received. The following bytes are treated as arguments and interpreted.

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.
