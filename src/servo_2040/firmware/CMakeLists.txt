cmake_minimum_required(VERSION 3.13)

# Pull in SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set Pimoroni Pico path - check env var first, then try local directory
if(DEFINED ENV{PIMORONI_PICO_PATH})
    set(PIMORONI_PICO_PATH $ENV{PIMORONI_PICO_PATH})
else()
    set(PIMORONI_PICO_PATH ${CMAKE_CURRENT_SOURCE_DIR}/pimoroni-pico)
endif()

include(${PIMORONI_PICO_PATH}/pimoroni_pico_import.cmake)

project(servo_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Include Pimoroni libraries
include_directories(${PIMORONI_PICO_PATH}/drivers)

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
