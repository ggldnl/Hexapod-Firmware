cmake_minimum_required(VERSION 3.12)

set(NAME hexapod_firmware)

# Copy pico_sdk_import.cmake and pimoroni_pico_import.cmake into this directory
# (they come with the Pico SDK / Pimoroni Pico libraries, the same ones the old
# firmware used). PICO_SDK_PATH and PIMORONI_PICO_PATH must be set in the env.
include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# src/ is the single include root; every #include is rooted there, so the folder
# shows in the path (e.g. #include "core/config.hpp", "communication/protocol.hpp").
include_directories(src)

add_executable(${NAME} src/main.cpp)

# Pimoroni drivers used by servo2040_hal.hpp.
include(libraries/servo2040/servo2040)
include(drivers/analogmux/analogmux)
include(drivers/analog/analog)

target_link_libraries(${NAME}
    hardware_uart
    pico_stdlib
    servo2040
    analogmux
    analog
)

# UART1 is the Pi link; keep both stdio backends off so nothing else drives it.
pico_enable_stdio_usb(${NAME} 0)
pico_enable_stdio_uart(${NAME} 0)

pico_add_extra_outputs(${NAME})
