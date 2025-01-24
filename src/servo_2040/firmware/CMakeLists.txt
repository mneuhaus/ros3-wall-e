cmake_minimum_required(VERSION 3.13)

# Pull in SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(servo_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(servo_controller
    main.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(servo_controller 
    pico_stdlib
    hardware_pwm
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(servo_controller 1)
pico_enable_stdio_uart(servo_controller 0)

# Create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(servo_controller)
