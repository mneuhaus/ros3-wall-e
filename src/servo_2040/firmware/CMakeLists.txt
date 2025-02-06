cmake_minimum_required(VERSION 3.12)

set(NAME servo_controller)

# Pull in SDK
include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(${NAME}
    main.cpp
)

# Include Pimoroni libraries
include_directories(${PIMORONI_PICO_PATH}/libraries)

# Link the libraries we need
target_link_libraries(${NAME}
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_pio
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(${NAME})

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
