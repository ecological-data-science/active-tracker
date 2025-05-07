
add_library(gps
    gps.cpp
    gps_pico.cpp
)

# Link against dependencies
target_link_libraries(gps 
    pico_stdlib 
    hardware_i2c
    hardware_rtc
    i2cbus  # Add the sfe_bus library as a dependency
)

# Add public include directory
target_include_directories(gps PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Compile options for this target only
target_compile_options(gps PRIVATE
    -Wall
    -Wno-format
    -Wno-unused-function
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(gps PRIVATE -Wno-maybe-uninitialized)
endif()
