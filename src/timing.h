#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"

/*
    Microsecond-precision timing built on the Pico's hardware_timer.
    Mirrors the tick_* API from the original x86 code but uses
    time_us_64() instead of RDTSC.

    Usage pattern:
        timing_preset_basetime();
        // ... do work ...
        uint64_t elapsed = timing_progress_us();

        timing_wait_us(1000);  // busy-wait until 1000us past basetime
*/

extern uint64_t timing_basetime_us;

static inline void timing_preset_basetime(void) {
    timing_basetime_us = time_us_64();
}

static inline uint64_t timing_progress_us(void) {
    return time_us_64() - timing_basetime_us;
}

void timing_wait_us(uint32_t us);

bool timing_check_pins(uint32_t mask,
                       uint32_t want,
                       uint32_t min_us,
                       uint32_t max_us);

#endif
