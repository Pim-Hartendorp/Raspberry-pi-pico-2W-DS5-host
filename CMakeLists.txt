# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(HID_host_bt C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(DS5_hid_host DS5_hid_host.c main.c )

pico_set_program_name(DS5_hid_host "DS5_hid_host")
pico_set_program_version(DS5_hid_host "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(DS5_hid_host 0)
pico_enable_stdio_usb(DS5_hid_host 1)

include_directories(
        ${PICO_SDK_PATH}/lib/btstack/platform/embedded
)

# Add the standard library to the build
target_link_libraries(DS5_hid_host
        pico_stdlib
        pico_cyw43_arch_none
        pico_btstack_cyw43
        pico_btstack_classic)

# Add the standard include files to the build
target_include_directories(DS5_hid_host PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_add_extra_outputs(DS5_hid_host)
