#include "timing.h"
#include "pico/time.h"
#include "hardware/gpio.h"

uint64_t timing_basetime_us = 0;

void timing_wait_us(uint32_t us) {
    uint64_t target = timing_basetime_us + us;
    while (time_us_64() < target) {
        tight_loop_contents();
    }
    timing_basetime_us = target;
}

bool timing_check_pins(uint32_t mask,
                       uint32_t want,
                       uint32_t min_us,
                       uint32_t max_us) {
    uint64_t deadline = timing_basetime_us + max_us;
    uint64_t earliest = timing_basetime_us + min_us;

    while (time_us_64() < deadline) {
        if ((gpio_get_all() & mask) == (want & mask)) {
            uint64_t now = time_us_64();
            if (now < earliest) {
                return false;
            }
            timing_basetime_us = now;
            return true;
        }
    }
    return false;
}
