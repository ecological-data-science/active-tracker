
#pragma once

#define DEBUG 0

#include "pico/stdlib.h"
#include <cstdio>

// Helper function to format time from microseconds since boot
// Returns a pointer to a static buffer containing the time string [HH:MM:SS]
static const char* formatTimePrefix() {
    uint64_t us_since_boot = time_us_64();
    uint64_t s_since_boot = us_since_boot / 1000000ULL; // Convert microseconds to seconds

    uint32_t hours = s_since_boot / 3600;
    uint32_t minutes = (s_since_boot % 3600) / 60;
    uint32_t seconds = s_since_boot % 60;
    uint32_t milliseconds = (us_since_boot % 1000000) / 1000;

    // Use a static buffer to store the formatted string
    static char buffer[15]; // Enough for "[HH:MM:SS]\0"
    snprintf(buffer, sizeof(buffer), "[%02u:%02u:%02u.%02u]", hours, minutes, seconds, milliseconds);

    return buffer;
}

#if DEBUG
#define DEBUG_PRINT(x) \
    do { \
        printf("%s ", formatTimePrefix()); \
        printf x; \
        printf("\n"); \
    } while(0)
#else
  #define DEBUG_PRINT(x)
#endif
