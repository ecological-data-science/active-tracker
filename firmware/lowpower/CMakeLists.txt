cmake_minimum_required(VERSION 3.12)



# This assumes PICO_SDK_PATH is set correctly as an environment variable
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})




set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
include(${PICO_SDK_INIT_CMAKE_FILE})

# We also need PICO EXTRAS
include(pico_extras_import.cmake)
# Set project name
project(low_power C CXX ASM)
# Set C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()


add_executable(lowpower
            lowpower.cpp
            lib/gps_pico.cpp
            lib/sfe_bus.cpp
)

# pull in common dependencies
target_link_libraries(lowpower pico_stdlib hardware_sleep hardware_i2c hardware_uart hardware_gpio)

# enable usb output, disable uart output
pico_enable_stdio_usb(lowpower 1)
pico_enable_stdio_uart(lowpower 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(lowpower)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()
