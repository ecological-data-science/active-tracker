cmake_minimum_required(VERSION 3.12)



# This assumes PICO_SDK_PATH is set correctly as an environment variable
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})


set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
include(${PICO_SDK_INIT_CMAKE_FILE})


# Set project name
project(storage C CXX ASM)
# Set C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()


add_executable(storage
            storage.cpp
            lib/pico_lfs_hal.c
            littlefs/lfs.c
            littlefs/lfs_util.c
)

add_subdirectory (/Users/colin.torney/workspace/pico-filesystem pfs)
# pull in common dependencies
target_link_libraries(storage pico_stdlib hardware_flash)

# enable usb output, disable uart output
pico_enable_stdio_usb(storage 1)
pico_enable_stdio_uart(storage 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(storage)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()
