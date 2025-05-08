/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define ROOT_SIZE 0x100000
#define ROOT_OFFSET 0x100000


#include <stdio.h>
#include "pico/stdlib.h"
#include <pfs.h>
struct pfs_pfs *pfs;
struct lfs_config cfg;

ffs_pico_createcfg (&cfg, ROOT_OFFSET, ROOT_SIZE);
pfs = pfs_ffs_create (&cfg);
pfs_mount (pfs, "/");

int main() {
    stdio_init_all();
    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
