cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME h723-template)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC

    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    algorithm/crc/crc.c
    algorithm/hash/hash.c
    algorithm/pid/pid.c
    algorithm/td/td.c

    application/chassis/chassis.c

    bsp/dwt/bsp_dwt.c
    bsp/fdcan/bsp_fdcan.c
    bsp/gpio/bsp_gpio.c
    bsp/rtt/SEGGER_RTT/RTT/SEGGER_RTT.c
    bsp/rtt/SEGGER_RTT/RTT/SEGGER_RTT_printf.c
    bsp/spi/bsp_spi.c
    bsp/tim/bsp_tim.c
    bsp/usart/bsp_usart.c

    device/buzzer/buzzer.c
    device/dbus/dbus.c
    device/sbus/sbus.c
    device/motor/motor.c
    device/ws2812/ws2812.c

    util/vofa/vofa.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/CMSIS/DSP/Include

    # Add user defined include paths
    algorithm/crc
    algorithm/hash
    algorithm/pid
    algorithm/td

    application/chassis

    bsp/dwt
    bsp/fdcan
    bsp/gpio
    bsp/rtt
    bsp/rtt/SEGGER_RTT/RTT
    bsp/spi
    bsp/tim
    bsp/usart

    device/buzzer
    device/dbus
    device/motor
    device/sbus
    device/ws2812

    util/log
    util/vofa
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Force include compiler_attributes.h in all C source files
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -include ${CMAKE_SOURCE_DIR}/compiler_attributes.h
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    arm_cortexM7lfdp_math
 
    # Add user defined libraries
)
