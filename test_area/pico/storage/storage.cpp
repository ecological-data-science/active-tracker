#include <stdio.h>
#include <pico/stdlib.h>

#include "littlefs/lfs.h"
#include "lib/pico_lfs_hal.h"

#include "pico_flash_fs.h"

// configuration of the filesystem is provided by this struct
struct lfs_config cfg = {
    // block device operations
    .read  = pico_read_flash_block,
    .prog  = pico_prog_flash_block,
    .erase = pico_erase_flash_block,
    .sync  = pico_sync_flash_block,

    // block device configuration

    // device is memory mapped for reading, so reading can be per byte
    .read_size = 1,
    
    .prog_size = PICO_PROG_PAGE_SIZE,
    .block_size = PICO_ERASE_PAGE_SIZE,

    // the number of blocks we use for a flash fs.
    .block_count = FLASHFS_BLOCK_COUNT,
    .block_cycles = 500,

    // cache needs to be a multiple of the programming page size.
    .cache_size = PICO_PROG_PAGE_SIZE * 1,

    .lookahead_size = 16,
};

lfs_t lfs;
lfs_file_t file;

int main()
{
    // the Pico SDK requires init at the start of main()
    stdio_init_all();

    sleep_ms(5000);
    printf("LittleFS example\n");
    // mount the filesystem
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }

    // read current count
    uint32_t boot_count = 0;
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

    // update boot count
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    lfs_file_close(&lfs, &file);

    // release any resources we were using
    lfs_unmount(&lfs);

    // print the boot count
    printf("boot_count: %d\n", boot_count);
  while (1) {
    sleep_ms(1000);
  printf( "%d bytes\n", PICO_FLASH_SIZE_BYTES);
  printf( "%d bytes\n", PICO_ERASE_PAGE_SIZE);
  }
}
