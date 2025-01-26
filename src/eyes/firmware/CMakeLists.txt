cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(eyes_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(eyes_firmware
    main.cpp
)

target_link_libraries(eyes_firmware 
    pico_stdlib
    hardware_spi
    hardware_dma
    hardware_pio
    pico_bootrom
)

pico_enable_stdio_usb(eyes_firmware 1)
pico_enable_stdio_uart(eyes_firmware 0)

pico_add_extra_outputs(eyes_firmware)
