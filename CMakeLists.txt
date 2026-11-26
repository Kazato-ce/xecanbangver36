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
set(CMAKE_PROJECT_NAME XECANBANGFINAL)

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
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/Src/app.c
    App/Src/state_machine.c
    App/Src/balance.c
    App/Src/imu_mpu6050.c
    App/Src/stepper_tmc2208.c
    App/Src/radio_nrf24.c
    App/Src/tof_vl53l0x.c
    App/Src/ultrasonic_hcsr05.c
    App/Src/telemetry_uart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App/Inc
)

# Add compile options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m
)

get_filename_component(ARM_TOOLCHAIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
find_program(OBJCOPY_EXE NAMES arm-none-eabi-objcopy.exe arm-none-eabi-objcopy HINTS "${ARM_TOOLCHAIN_DIR}" "C:/Users/Thanh/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin")

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${OBJCOPY_EXE} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${OBJCOPY_EXE} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Building ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
)
