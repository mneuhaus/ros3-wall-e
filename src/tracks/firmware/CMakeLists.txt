cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(tracks_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../external/pico-examples/ws2812)

link_directories(libmicroros)
add_executable(tracks_firmware
    main.cpp
    pico_uart_transport.c
    libraries/Adafruit_NeoPixel/Adafruit_NeoPixel.cpp
)
target_link_libraries(tracks_firmware
    pico_stdlib
    microros
)

target_include_directories(tracks_firmware PUBLIC
    libmicroros/include
    libraries/Adafruit_NeoPixel
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(tracks_firmware 1)
pico_enable_stdio_uart(tracks_firmware 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
add_compile_definitions(PICO_DEFAULT_USB_PRODUCT="wall-e: tracks controller")

# Generate UF2
pico_add_extra_outputs(tracks_firmware)


