# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DEBUG
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_BOARD pimoroni_servo2040 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Servo2040 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Pull in FreeRTOS
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-Kernel)
include(lib/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Pull in servo2040
set(PIMORONI_PICO_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pimoroni-pico)
include(lib/pimoroni-pico/pimoroni_pico_import.cmake)

add_subdirectory(lib/nrf24l01)


# Find all .c files in the src directory
file(GLOB SRC_FILES src/*.c src/*.cpp)
add_executable(Servo2040 ${SRC_FILES} )

pico_set_program_name(Servo2040 "Servo2040")
pico_set_program_version(Servo2040 "0.1")

# Add the standard library to the build
target_link_libraries(Servo2040 pico_stdlib)

# Add the standard include files to the build
target_include_directories(Servo2040 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

include(libraries/servo2040/servo2040)
include(drivers/analogmux/analogmux)
include(drivers/analog/analog)
include(drivers/button/button)

# Add any user requested libraries
target_link_libraries(Servo2040
    pico_stdlib 
    FreeRTOS-Kernel 
    FreeRTOS-Kernel-Heap4
    hardware_spi
    servo2040
    analogmux
    analog
    button
    nrf24_driver
)

pico_add_extra_outputs(Servo2040)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Servo2040 0)
pico_enable_stdio_usb(Servo2040 1)

