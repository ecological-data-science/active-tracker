

add_library(classifier
    classifier.cpp
    imu.cpp
    tflite_classifier.cpp
)

# Link against dependencies
target_link_libraries(classifier 
    pico_stdlib 
    hardware_i2c
    i2cbus 
    pico-tflmicro
)

# Add public include directory
target_include_directories(classifier PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/filter
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Compile options for this target only
target_compile_options(classifier PRIVATE
    -Wall
    -Wno-format
    -Wno-unused-function
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(classifier PRIVATE -Wno-maybe-uninitialized)
endif()
