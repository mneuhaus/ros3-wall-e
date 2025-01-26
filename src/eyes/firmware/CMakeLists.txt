cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(eyes_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(eyes
    main.cpp
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(eyes 
    pico_stdlib 
    hardware_spi 
    hardware_dma
    hardware_pio
    pico_bootrom
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(eyes 1)
pico_enable_stdio_uart(eyes 0)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(eyes)
