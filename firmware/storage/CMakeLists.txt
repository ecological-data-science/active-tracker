
add_library(storage
            storage.cpp
            pico_lfs_hal.c
            lfs.c
            lfs_util.c
)

# pull in common dependencies
target_link_libraries(storage pico_stdlib hardware_flash)

# Add public include directory
target_include_directories(storage PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

add_compile_options(storage PRIVATE
        -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()
