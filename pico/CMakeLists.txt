cmake_minimum_required(VERSION 3.13)

# Import Pico SDK (must be before project())
include(pico_sdk_import.cmake)

project(rc_car_controller C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Main executable
add_executable(rc_car_controller
    main.c
)

# Link required libraries
target_link_libraries(rc_car_controller
    pico_stdlib
    pico_i2c_slave
    hardware_i2c
    hardware_pwm
)

# Enable USB serial output (disable UART)
pico_enable_stdio_usb(rc_car_controller 1)
pico_enable_stdio_uart(rc_car_controller 0)

# Generate UF2 file for flashing
pico_add_extra_outputs(rc_car_controller)
