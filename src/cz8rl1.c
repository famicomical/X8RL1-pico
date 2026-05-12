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
    /* APSS: BUSY stays HIGH and STATUS stays LOW for >2 * TICK_HEAD.
       This matches the original X8RL1/cz8rl1.c polarity (where the
       APSS condition is `BUSY=1 AND STATUS=0` in the LPT status
       register, with no XOR mask). Normal idle on this device is
       BUSY=0/STATUS=1, which would have falsely triggered APSS. */
    if (pin_read(PIN_BUSY) == true && pin_read(PIN_STATUS) == false) {
        return !wait_pin(PIN_STATUS, true, 0, TICK_HEAD * 2);
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

    /* STATUS idles HIGH on this device. Header = device pulls STATUS
       LOW for ~1000us, then releases. Bits = LOW for 750us (=1) or
       250us (=0), with a HIGH gap between. This matches the original
       LPT-port code's polarity (check_low for assertion, check_high
       for release). */

    /* wait for STATUS to go low (start of header), within 2s */
    if (!wait_pin(PIN_STATUS, false, 0, 2000000)) {
        set_idle();
        return CZ8RL1_ERR_NOHEADER;
    }
    /* header: STATUS low for ~1000us +- 25%, then back to high */
    if (!wait_pin(PIN_STATUS, true,
                  (uint32_t)(TICK_HEAD * 0.75),
                  (uint32_t)(TICK_HEAD * 1.25))) {
        set_idle();
        return CZ8RL1_ERR_BADHEAD;
    }

    /* Read 8 bits, MSB first.

       The bit value is encoded in the HIGH-space duration between
       consecutive LOW pulses (the original tx defines bit=1 as a
       750us space and bit=0 as a 250us space, then a fixed 250us
       LOW "make"). Measure the space width directly. */
    for (int bitmask = 0x80; bitmask; bitmask >>= 1) {
        /* measure SPACE: how long STATUS stays HIGH before the next
           falling edge */
        timing_preset_basetime();
        uint64_t space_us;
        do {
            space_us = timing_progress_us();
            if (space_us > 2000) {
                set_idle();
                return CZ8RL1_ERR_BADBIT;
            }
        } while (pin_read(PIN_STATUS));

        /* wait through the MAKE (LOW) pulse */
        timing_preset_basetime();
        uint64_t make_us;
        do {
            make_us = timing_progress_us();
            if (make_us > 2000) {
                set_idle();
                return CZ8RL1_ERR_BADBIT;
            }
        } while (!pin_read(PIN_STATUS));

        /* threshold midway between 250us (bit=0) and 750us (bit=1) */
        if (space_us > 500) {
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

static inline uint8_t sample_inputs(void) {
    uint32_t gpio = sio_hw->gpio_in;
    uint8_t s = 0;
    if (gpio & (1u << PIN_BUSY))      s |= 0x01;
    if (gpio & (1u << PIN_STATUS))    s |= 0x02;
    if (gpio & (1u << PIN_READ_DATA)) s |= 0x04;
    return s;
}

uint32_t cz8rl1_trace_command(uint8_t command,
                              uint32_t trace_duration_us,
                              cz8rl1_trace_event_t *events,
                              uint32_t max_events,
                              uint8_t *initial_state_out) {
    uint32_t saved = save_and_disable_interrupts();

    uint8_t initial = sample_inputs();
    if (initial_state_out) *initial_state_out = initial;

    uint64_t tx_start_us = time_us_64();
    cz8rl1_tx(command);

    uint32_t n = 0;
    uint8_t  prev = initial;
    uint64_t deadline = tx_start_us + trace_duration_us;

    /* Replicate the full rx handshake so the device actually transmits
       its response on STATUS: catch the BUSY-rise (ack), wait 1ms,
       assert STROBE. Then keep sampling so STATUS pulses are recorded. */
    bool busy_rise_seen = false;
    bool strobe_asserted = false;
    uint64_t busy_rise_us = 0;

    while (time_us_64() < deadline) {
        uint8_t cur = sample_inputs();
        uint64_t now = time_us_64();

        if (cur != prev) {
            if (n < max_events) {
                events[n].t_us     = (uint32_t)(now - tx_start_us);
                events[n].pin_state = cur;
                n++;
            }
            if (!busy_rise_seen && (cur & 0x01) && !(prev & 0x01)) {
                busy_rise_seen = true;
                busy_rise_us = now;
            }
            prev = cur;
        }

        if (busy_rise_seen && !strobe_asserted &&
                (now - busy_rise_us) >= 1000) {
            pin_high(PIN_STROBE);
            strobe_asserted = true;
        }
    }

    pin_low(PIN_STROBE);
    set_idle();
    restore_interrupts(saved);
    return n;
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
