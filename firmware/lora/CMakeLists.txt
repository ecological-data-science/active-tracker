

add_library(lora
    lora.cpp
)

# Link against dependencies
target_link_libraries(lora 
    pico_stdlib 
    hardware_uart
    loramessage
    storage
)

# Add public include directory
target_include_directories(lora PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Compile options for this target only
target_compile_options(lora PRIVATE
    -Wall
    -Wno-format
    -Wno-unused-function
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(lora PRIVATE -Wno-maybe-uninitialized)
endif()

