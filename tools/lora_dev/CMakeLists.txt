cmake_minimum_required(VERSION 3.12)


# Set C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# This assumes PICO_SDK_PATH is set correctly as an environment variable
set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/external/pico-sdk)

set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
include(${PICO_SDK_INIT_CMAKE_FILE})


# Set project name
project(lora_dev C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# Add the lib subdirectory
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lora)

add_executable(lora_dev
            main.cpp
)

# pull in common dependencies
target_link_libraries(lora_dev pico_stdlib hardware_flash hardware_i2c lora)

target_include_directories(lora_dev
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# enable usb output, disable uart output
pico_enable_stdio_usb(lora_dev 1)
pico_enable_stdio_uart(lora_dev 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(lora_dev)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

