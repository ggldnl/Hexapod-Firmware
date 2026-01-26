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

<!-- Table is small, this way it fits the whole page -->
<table style="width:100%; border-collapse: collapse;">
  <tr>
    <th>Raspberry</th>
    <th>Servo2040</th>
  </tr>
  <tr>
    <td>5V</td>
    <td>5V</td>
  </tr>
  <tr>
    <td>GND</td>
    <td>GND</td>
  </tr>
  <tr>
    <td>GPIO14 (TXD)</td>
    <td>SDA (RX)</td>
  </tr>
  <tr>
    <td>GPIO15 (RXD)</td>
    <td>SCL (TX)</td>
  </tr>
</table>

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

<table style="width:100%; border-collapse: collapse;">
  <tr>
    <th>SOF</th>
    <th>LEN</th>
    <th>OPCODE</th>
    <th>DATA</th>
    <th>CRC</th>
    <th>EOF</th>
  </tr>
  <tr>
    <td>1B</td>
    <td>1B</td>
    <td>1B</td>
    <td>N bytes</td>
    <td>2B</td>
    <td>1B</td>
  </tr>
</table>

<!-- Table is small, this way it fits the whole page -->
<table style="width:100%; border-collapse: collapse;">
  <tr>
    <th>Field</th>
    <th>Size (bytes)</th>
    <th>Description</th>
  </tr>
<tr>
    <td>SOF</td>
    <td>1</td>
    <td>Start-of-frame marker (`0xAA`)</td>
  </tr>
  <tr>
    <td>LEN</td>
    <td>1</td>
    <td>Length of `OPCODE + DATA`</td>
  </tr>
  <tr>
    <td>OPCODE</td>
    <td>1</td>
    <td>Command identifier</td>
  </tr>
  <tr>
    <td>DATA</td>
    <td>N</td>
    <td>Arguments (binary)</td>
  </tr>
  <tr>
    <td>CRC</td>
    <td>2</td>
    <td>CRC-16 over `LEN + OPCODE + DATA`</td>
  </tr>
  <tr>
    <td>EOF</td>
    <td>1</td>
    <td>End-of-frame marker (`0x55`)</td>
  </tr>
</table>

CRC-16 is a 16-bit cyclic redundancy check used to detect errors in transmitted frames. When a receiver gets a frame, it recomputes the CRC-16 and compares it to the received FCS. If they differ, the frame is considered corrupted.

### Instruction set

I used the Command design pattern to dispatch commands once extracted from a message.

The following table describes the supported operations, their opcodes, the expected arguments, and the response:

<table style="width:100%; border-collapse:collapse;">
  <tr>
    <th style="text-align:left;">Operation</th>
    <th style="text-align:left;">OpCode</th>
    <th style="text-align:left;">Arguments</th>
    <th style="text-align:left;">Response</th>
  </tr>
  <tr>
    <td>Get Voltage</td>
    <td><code>0x01</code></td>
    <td>None</td>
    <td>voltage(4b)</td>
  </tr>

  <tr>
    <td>Get Current</td>
    <td><code>0x02</code></td>
    <td>None</td>
    <td>current(4b)</td>
  </tr>

  <tr>
    <td>Read Sensor</td>
    <td><code>0x03</code></td>
    <td>pin (1b)</td>
    <td>value(4b)</td>
  </tr>

  <tr>
    <td>Set LED</td>
    <td><code>0x04</code></td>
    <td>pin(1b), r(1b), g(1b), b(1b)</td>
    <td>status(1b)</td>
  </tr>

  <tr>
    <td>Set LEDs</td>
    <td><code>0x05</code></td>
    <td>
      count(1b),<br>
      [pin(1b), r(1b), g(1b), b(1b)] × count
    </td>
    <td>status(1b)</td>
  </tr>

  <tr>
    <td>Get LED</td>
    <td><code>0x06</code></td>
    <td>pin (1b)</td>
    <td>r(1b), g(1b), b(1b)</td>
  </tr>

  <tr>
    <td>Get LEDs</td>
    <td><code>0x07</code></td>
    <td>count(1b), [pin(1b)] × count</td>
    <td>[r(1b), g(1b), b(1b)] × count</td>
  </tr>

  <tr>
    <td>Attach Servos</td>
    <td><code>0x08</code></td>
    <td>None</td>
    <td>status(1b)</td>
  </tr>

  <tr>
    <td>Detach Servos</td>
    <td><code>0x09</code></td>
    <td>None</td>
    <td>status(1b)</td>
  </tr>

  <tr>
    <td>Set Servo Pulse Width</td>
    <td><code>0x0A</code></td>
    <td>pin(1b), pulse_width(4b)</td>
    <td>status(1b)</td>
  </tr>

  <tr>
    <td>Set Servo Pulse Widths</td>
    <td><code>0x0B</code></td>
    <td>
      count(1b),<br>
      [pin(1b), pulse_width(4b)] × count
    </td>
    <td>status(1b)</td>
  </tr>

  <tr>
    <td>Set Servo Angle</td>
    <td><code>0x0C</code></td>
    <td>pin(1b), angle(4b)</td>
    <td>status(1b)</td>
  </tr>

  <tr>
    <td>Set Servo Angles</td>
    <td><code>0x0D</code></td>
    <td>
      count(1b),<br>
      [pin(1b), angle(4b)] × count
    </td>
    <td>status(1b)</td>
  </tr>

  <tr>
    <td>Get Servo Pulse Width</td>
    <td><code>0x0E</code></td>
    <td>pin (1b)</td>
    <td>pulse_width(4b)</td>
  </tr>

  <tr>
    <td>Get Servo Pulse Widths</td>
    <td><code>0x0F</code></td>
    <td>count(1b), [pin(1b)] × count</td>
    <td>pulse_width(4b) × count</td>
  </tr>

  <tr>
    <td>Get Servo Angle</td>
    <td><code>0x10</code></td>
    <td>pin(1b)</td>
    <td>angle(4b)</td>
  </tr>

  <tr>
    <td>Get Servo Angles</td>
    <td><code>0x11</code></td>
    <td>count(1b), [pin(1b)] × count</td>
    <td>angle(4b) × count</td>
  </tr>

  <tr>
    <td>Connect Power</td>
    <td><code>0x12</code></td>
    <td>None</td>
    <td>status(1b)</td>
  </tr>

  <tr>
    <td>Disconnect Power</td>
    <td><code>0x13</code></td>
    <td>None</td>
    <td>status(1b)</td>
  </tr>
</table>

The response is always guaranteed. For commands that return data (e.g. `get_voltage`), a successful execution returns the actual requested data, while a failure returns all bytes set to `0x00`. For commands that perform actions (e.g. `set_led`), a successful execution returns `0x01`, while a failure returns `0x00`. The length of arguments and responses are expressed in bytes (e.g. 4b means 4 bytes i.e. a float).

Description table:

<table style="width:100%; border-collapse:collapse;">
  <tr>
    <th style="text-align:left;">Operation</th>
    <th style="text-align:left;">Description</th>
  </tr>

  <tr>
    <td>Get Voltage</td>
    <td>Reads the voltage present on the external power line.</td>
  </tr>

  <tr>
    <td>Get Current</td>
    <td>Reads the current flowing through the external power line.</td>
  </tr>

  <tr>
    <td>Read Sensor</td>
    <td>Reads the analog value of the specified input pin.</td>
  </tr>

  <tr>
    <td>Set LED</td>
    <td>Sets the RGB color of a single LED connected to the specified pin.</td>
  </tr>

  <tr>
    <td>Set LEDs</td>
    <td>Sets the RGB color of multiple LEDs in a single command.</td>
  </tr>

  <tr>
    <td>Get LED</td>
    <td>Reads the current RGB color of the specified LED.</td>
  </tr>

  <tr>
    <td>Get LEDs</td>
    <td>Reads the current RGB color of multiple LEDs.</td>
  </tr>

  <tr>
    <td>Attach Servos</td>
    <td>Initializes and attaches all configured servo outputs.</td>
  </tr>

  <tr>
    <td>Detach Servos</td>
    <td>Detaches all servo outputs and disables signal generation.</td>
  </tr>

  <tr>
    <td>Set Servo Pulse Width</td>
    <td>Sets the pulse width for a single servo.</td>
  </tr>

  <tr>
    <td>Set Servo Pulse Widths</td>
    <td>Sets the pulse width for multiple servos.</td>
  </tr>

  <tr>
    <td>Set Servo Angle</td>
    <td>Sets the target angle for a single servo.</td>
  </tr>

  <tr>
    <td>Set Servo Angles</td>
    <td>Sets the target angle for multiple servos.</td>
  </tr>

  <tr>
    <td>Get Servo Pulse Width</td>
    <td>Reads the current pulse width of the specified servo.</td>
  </tr>

  <tr>
    <td>Get Servo Pulse Widths</td>
    <td>Reads the current pulse width of multiple servos.</td>
  </tr>

  <tr>
    <td>Get Servo Angle</td>
    <td>Reads the current angle of the specified servo.</td>
  </tr>

  <tr>
    <td>Get Servo Angles</td>
    <td>Reads the current angle of multiple servos.</td>
  </tr>

  <tr>
    <td>Connect Power</td>
    <td>Enables external power delivery to the servos.</td>
  </tr>

  <tr>
    <td>Disconnect Power</td>
    <td>Disables external power delivery to the servos.</td>
  </tr>
</table>


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

    // No hardwrae lirbary
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
