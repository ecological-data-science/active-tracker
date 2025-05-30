
# pragma once

#include "pico_lfs_hal.h"

// Defines one region of flash to use for a filesystem. The size is a multiple of
// the 4096 byte erase size. We calculate it's location working back from the end of the
// flash device, so that code flashed at the start of the device will not collide.
// Pimoroni Tiny 2040 have an 8Mb flash device, so we're looking to be less than 8Mb.

// 1536 blocks will reserve a 6Mb filsystem - 3/4 of the 8Mb device

#define FLASHFS_BLOCK_COUNT 1536
#define FLASHFS_SIZE_BYTES (PICO_ERASE_PAGE_SIZE * FLASHFS_BLOCK_COUNT)

// Flash can be addressed at several different aliased addresses. We use the base
// address to document the location, and for addresses in the UF2 file format. On the
// actual pico we will calculate a different base address which will use an alias
// that gives us the caching performance we want.
// A start location counted back from the end of the device.
#define FLASHFS_BASE_ADDR (XIP_MAIN_BASE + PICO_FLASH_SIZE_BYTES - FLASHFS_SIZE_BYTES)

