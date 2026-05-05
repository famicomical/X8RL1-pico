/*
    CZ8RL1 protocol driver, Pico port.

    Direct GPIO replaces the original parallel-port I/O. Logic is
    bit-banged; timing comes from the hardware microsecond timer.
    Critical sections disable interrupts to avoid jitter on the
    self-clocked serial protocol.

    The CZ8RL1 outputs (BUSY / STATUS / READ_DATA) appear to be
    open-collector, so the Pico's internal pullups provide the high
    level (3.3V); no external pullups are needed.
*/

#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

#include "cz8rl1.h"
#include "pins.h"
#include "timing.h"

/* command / status protocol bit timing (microseconds) */
#define TICK_HEAD   1000
#define TICK_BIT1    750
#define TICK_BIT0    250
#define TICK_BIT     250

/* Direct GPIO helpers (faster than gpio_put / gpio_get for inner loops). */
static inline void pin_high(uint pin) { sio_hw->gpio_set = 1u << pin; }
static inline void pin_low(uint pin)  { sio_hw->gpio_clr = 1u << pin; }
static inline bool pin_read(uint pin) { return (sio_hw->gpio_in >> pin) & 1u; }

/* Idle line state: COMMAND high, STROBE low, WRITE_DATA low. */
static inline void set_idle(void) {
    pin_high(PIN_COMMAND);
    pin_low(PIN_STROBE);
    pin_low(PIN_WRITE_DATA);
}

void cz8rl1_init(void) {
    /* outputs */
    gpio_init(PIN_WRITE_DATA);
    gpio_init(PIN_STROBE);
    gpio_init(PIN_COMMAND);
    gpio_set_dir(PIN_WRITE_DATA, GPIO_OUT);
    gpio_set_dir(PIN_STROBE,     GPIO_OUT);
    gpio_set_dir(PIN_COMMAND,    GPIO_OUT);

    /* inputs with pullups (CZ8RL1 outputs are open-collector) */
    gpio_init(PIN_BUSY);
    gpio_init(PIN_READ_DATA);
    gpio_init(PIN_STATUS);
    gpio_set_dir(PIN_BUSY,      GPIO_IN);
    gpio_set_dir(PIN_READ_DATA, GPIO_IN);
    gpio_set_dir(PIN_STATUS,    GPIO_IN);
    gpio_pull_up(PIN_BUSY);
    gpio_pull_up(PIN_READ_DATA);
    gpio_pull_up(PIN_STATUS);

    set_idle();

    /* let the line settle before issuing the first command */
    sleep_ms(100);
}

/*
    Wait until pin matches `want` (true = high, false = low), within
    [min_us, max_us] from basetime. Returns true on success and updates
    basetime to the time the level was reached.
*/
static bool wait_pin(uint pin, bool want, uint32_t min_us, uint32_t max_us) {
    uint64_t deadline = timing_basetime_us + max_us;
    uint64_t earliest = timing_basetime_us + min_us;
    while (time_us_64() < deadline) {
        if (pin_read(pin) == want) {
            uint64_t now = time_us_64();
            if (now < earliest) return false;
            timing_basetime_us = now;
            return true;
        }
    }
    return false;
}

static bool is_apss(void) {
    timing_preset_basetime();
    /* APSS: BUSY low and STATUS stays high for >2 * TICK_HEAD */
    if (pin_read(PIN_BUSY) == false && pin_read(PIN_STATUS) == true) {
        return !wait_pin(PIN_STATUS, false, 0, TICK_HEAD * 2);
    }
    return false;
}

static bool apss_break(void) {
    if (!is_apss()) return false;

    /* assert STROBE to request break */
    pin_high(PIN_STROBE);
    timing_preset_basetime();
    bool ok = wait_pin(PIN_BUSY, false, 0, TICK_HEAD * 1000);
    pin_low(PIN_STROBE);
    set_idle();
    return ok;
}

/* Receive one byte from CZ8RL1 (interrupts must be disabled). */
static int cz8rl1_rx(void) {
    int value = 0;

    timing_preset_basetime();

    /* wait for BUSY to go high (within 2s) */
    if (!wait_pin(PIN_BUSY, true, 0, 2000000)) {
        return CZ8RL1_ERR_BSYACK;
    }
    timing_preset_basetime();
    timing_wait_us(1000);

    /* assert STROBE */
    pin_high(PIN_STROBE);

    /* wait for BUSY low (within 2ms) */
    if (!wait_pin(PIN_BUSY, false, 0, 2000)) {
        set_idle();
        return CZ8RL1_STS_DOAPSS;
    }

    /* wait for STATUS to go high (start of header), within 2s */
    if (!wait_pin(PIN_STATUS, true, 0, 2000000)) {
        set_idle();
        return CZ8RL1_ERR_NOHEADER;
    }
    /* header: STATUS high for ~1000us +- 25% */
    if (!wait_pin(PIN_STATUS, false,
                  (uint32_t)(TICK_HEAD * 0.75),
                  (uint32_t)(TICK_HEAD * 1.25))) {
        set_idle();
        return CZ8RL1_ERR_BADHEAD;
    }

    /* read 8 bits, MSB first */
    for (int bitmask = 0x80; bitmask; bitmask >>= 1) {
        /* wait for STATUS high (start of bit) */
        timing_preset_basetime();
        while (!pin_read(PIN_STATUS)) {
            if (timing_progress_us() > 2000) {
                set_idle();
                return CZ8RL1_ERR_BADBIT;
            }
        }
        /* measure how long STATUS stays high */
        timing_preset_basetime();
        uint64_t width;
        do {
            width = timing_progress_us();
            if (width > 2000) {
                set_idle();
                return CZ8RL1_ERR_BADBIT;
            }
        } while (pin_read(PIN_STATUS));
        /* wait same width on the LOW side, then sample */
        timing_wait_us((uint32_t)width);
        if (width > 750) {
            value |= bitmask;
        }
    }

    set_idle();
    return value;
}

/* Transmit one byte to CZ8RL1 (interrupts must be disabled). */
static void cz8rl1_tx(uint8_t value) {
    timing_preset_basetime();
    /* header: COMMAND low for 1000us */
    pin_low(PIN_COMMAND);
    timing_wait_us(TICK_HEAD);
    pin_high(PIN_COMMAND);

    for (uint8_t bitmask = 0x80; bitmask; bitmask >>= 1) {
        /* space: 750us if bit=1, 250us if bit=0 */
        timing_wait_us((value & bitmask) ? TICK_BIT1 : TICK_BIT0);
        /* mark: COMMAND low for 250us */
        pin_low(PIN_COMMAND);
        timing_wait_us(TICK_BIT);
        pin_high(PIN_COMMAND);
    }
}

int cz8rl1_send_command(uint8_t command) {
    int status;
    uint32_t saved = save_and_disable_interrupts();

    if (is_apss()) {
        if (!apss_break()) {
            restore_interrupts(saved);
            return CZ8RL1_ERR_DOAPSS;
        }
        status = cz8rl1_rx();
        restore_interrupts(saved);
        if (status < 0) return status;
        saved = save_and_disable_interrupts();
    }

    cz8rl1_tx(command);
    status = cz8rl1_rx();
    restore_interrupts(saved);

    /* small inter-command pause */
    sleep_us(1000);
    return status;
}

int cz8rl1_write_data(const uint8_t *buf,
                      uint32_t buf_size,
                      uint32_t sample_rate,
                      bool busy_break) {
    uint64_t step_us = 1000000ULL / sample_rate;
    uint32_t bit_count = buf_size * 8;
    uint32_t pos;
    uint8_t cdata = 0;

    uint32_t saved = save_and_disable_interrupts();
    timing_preset_basetime();

    for (pos = 0; pos < bit_count; pos++) {
        if (busy_break && pin_read(PIN_BUSY)) break;

        if ((pos & 7) == 0) {
            cdata = buf[pos >> 3];
        } else {
            cdata <<= 1;
        }

        timing_wait_us((uint32_t)step_us);
        if (cdata & 0x80) pin_high(PIN_WRITE_DATA);
        else              pin_low(PIN_WRITE_DATA);
    }

    set_idle();
    restore_interrupts(saved);

    return (pos < bit_count) ? CZ8RL1_STS_BREAK : CZ8RL1_STS_OK;
}
