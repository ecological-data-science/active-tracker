
add_library(lowpower
            lowpower.cpp
)

# pull in common dependencies
target_link_libraries(lowpower pico_stdlib pico_aon_timer hardware_sleep)

# Add public include directory
target_include_directories(lowpower PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    # ${CMAKE_CURRENT_SOURCE_DIR}/../external/pico-extras/src/rp2_common/pico_sleep/include
    # ${CMAKE_CURRENT_SOURCE_DIR}/../external/pico-extras/src/rp2_common/hardware_rosc/include
)

add_compile_options(lowpower PRIVATE
        -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()
