# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
   set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(PICO_SDK_PATH ${USERHOME}/.pico-sdk/sdk/1.5.1)
set(PICO_TOOLCHAIN_PATH ${USERHOME}/.pico-sdk/toolchain/13_2_Rel1)
if(WIN32)
    set(pico-sdk-tools_DIR ${USERHOME}/.pico-sdk/tools/1.5.1)
    include(${pico-sdk-tools_DIR}/pico-sdk-tools-config.cmake)
    include(${pico-sdk-tools_DIR}/pico-sdk-tools-config-version.cmake)
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(app C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Pull in FreeRTOS
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Pull in servo2040
include($ENV{PIMORONI_PICO_PATH}/pimoroni_pico_import.cmake)

link_directories(libmicroros)

# Find all .c files in the src directory
file(GLOB SRC_FILES src/*.c src/*.cpp)

add_executable(app ${SRC_FILES})

target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# uROS
target_include_directories(app PUBLIC
    libmicroros/include
)
include(libraries/servo2040/servo2040)
include(drivers/analogmux/analogmux)
include(drivers/analog/analog)
include(drivers/button/button)
 
# pull in common dependencies
target_link_libraries(app 
    pico_stdlib 
    microros
    FreeRTOS-Kernel 
    FreeRTOS-Kernel-Heap4
    servo2040
    analogmux
    analog
    button
    )

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(app)

# enable usb output, disable uart output
pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 1)

# uROS
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)