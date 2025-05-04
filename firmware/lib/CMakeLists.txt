
add_library(i2cbus STATIC
    i2c_bus.cpp
)

# Link against dependencies
target_link_libraries(i2cbus PUBLIC 
    pico_stdlib 
    hardware_i2c
)

# Add public include directory
target_include_directories(i2cbus PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Compile options for this target only
target_compile_options(i2cbus PRIVATE
    -Wall
    -Wno-format
    -Wno-unused-function
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(i2cbus PRIVATE -Wno-maybe-uninitialized)
endif()
