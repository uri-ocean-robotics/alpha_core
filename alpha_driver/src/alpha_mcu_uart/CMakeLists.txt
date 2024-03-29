# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)
#include build functions from Pico SDK

include(pico_sdk_import.cmake)
# Set name of project (as PROJECT_NAME) and C/C++ Standards
project(alpha_mcu_uart C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (NOT TARGET NMEA_PARSER)
  add_subdirectory(../nmea_parser nmea_parser_pico)
endif()

if (NOT TARGET ALPHA_COMMON)
  add_subdirectory(../alpha_common alpha_common_pico)
endif()

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()
include_directories(include)
# point out the CMake, where to find the executable source file
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/alpha/manager.cpp
  src/alpha/pwm_controller.cpp
)

# create map/bin/hex/uf2 files.
pico_add_extra_outputs(${PROJECT_NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(${PROJECT_NAME})
# Pull in our pico_stdlib which pulls in commonly used features (gpio, timer-delay etc)
target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_bootsel_via_double_reset
        hardware_pwm
        hardware_gpio
        hardware_i2c
        hardware_uart
        pico_multicore
        NMEA_PARSER
        ALPHA_COMMON
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/upload.sh ${CMAKE_BINARY_DIR}/upload.sh
)

add_custom_target(upload)

add_custom_command(TARGET upload
    COMMAND ${CMAKE_BINARY_DIR}/upload.sh
)
