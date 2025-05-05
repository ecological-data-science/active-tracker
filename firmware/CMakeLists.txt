cmake_minimum_required(VERSION 3.12)


# Set C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# This assumes PICO_SDK_PATH is set correctly as an environment variable
set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external/pico-sdk)

set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
include(${PICO_SDK_INIT_CMAKE_FILE})

# Set project name
project(pico_tracker C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# Add the external libraries
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external)

# Add the lib subdirectory
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/gps)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/classifier)

add_executable(pico_tracker
            main.cpp
)

# pull in common dependencies
target_link_libraries(pico_tracker pico_stdlib hardware_i2c gps classifier)

target_include_directories(pico_tracker
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_tracker 1)
pico_enable_stdio_uart(pico_tracker 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(pico_tracker)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

