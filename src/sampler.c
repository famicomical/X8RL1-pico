/*
    Continuous tape sampler. See sampler.h for the design overview.

    Platform layer (lines marked PLATFORM) is the only Pico-specific
    code: hardware_alarm_set_target / from_us_since_boot,
    gpio_set_irq_enabled_with_callback, time_us_64, gpio_get. Replace
    those four families to port to another MCU or to a Linux/POSIX
    timer-signal scheme. The algorithm above is platform-neutral.
*/

#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"

#include "sampler.h"
#include "cz8rl1.h"
#include "pins.h"

#define RING_SIZE  8192u
#define RING_MASK  (RING_SIZE - 1u)

static uint8_t            ring_buf[RING_SIZE];
static volatile uint32_t  ring_head;     /* writer: alarm ISR        */
static volatile uint32_t  ring_tail;     /* writer: main loop        */

/* Constants for the run, set in sampler_start, read in ISRs. */
static uint64_t step_us;
static uint64_t jit_us[2];
static bool     jit_flag[2];
static uint32_t idle_limit;
static bool     busy_break_enabled;

/* Per-run state. ISRs run at the same NVIC priority and therefore
   serialise; reads/writes of these fields don't need locks. */
static volatile uint64_t last_sample_us;
static volatile bool     edge_seen_this_period;
static volatile bool     running;
static volatile int      terminate_reason;

static volatile uint32_t total_edges;
static volatile uint32_t dropped_edges;   /* edges rejected by stale guard */
static volatile uint32_t total_bytes;
static volatile uint32_t total_samples;
static uint64_t          jitter_sum[2];

static uint8_t  byte_buf;
static uint8_t  bit_count;
static uint32_t idle_left;
static int      edge_level;

static int alarm_num = -1;          /* PLATFORM: hardware alarm slot */

/* ---- ISRs ----------------------------------------------------------- */

static void gpio_callback(uint gpio, uint32_t events) {
    (void)events;
    if (gpio != PIN_READ_DATA) return;
    if (!running) return;

    uint64_t edge_t = time_us_64();                               /* PLATFORM */
    uint64_t ticknow = edge_t - last_sample_us;
    if (ticknow > step_us) {        /* stale / outside this period */
        dropped_edges++;
        return;
    }

    int level = edge_level;
    jitter_sum[level] += ticknow;

    if (jit_flag[level]) {
        /*
            Slide the upcoming alarm so the next sample lands at
            (step - jit_us) past this edge — i.e., jit_us into the
            next bit cell. Identical effect to the DOS code's
            tickbase += ticknow - tick_jitter, but expressed as an
            absolute alarm target.
        */
        uint64_t new_target = edge_t + step_us - jit_us[level];
        hardware_alarm_set_target((uint)alarm_num,                /* PLATFORM */
                                  from_us_since_boot(new_target));
    }

    edge_level ^= 1;
    total_edges++;
    edge_seen_this_period = true;
}

static void alarm_callback(uint num) {
    (void)num;
    if (!running) return;

    uint64_t sample_t = time_us_64();                             /* PLATFORM */
    bool level_now    = gpio_get(PIN_READ_DATA);                  /* PLATFORM */

    /* Idle-break counter: any edge in this period resets it. */
    if (edge_seen_this_period) {
        edge_seen_this_period = false;
        idle_left = idle_limit;
    } else if (idle_limit > 0 && --idle_left == 0) {
        terminate_reason = CZ8RL1_STS_AUTOSTOP;
        running = false;
        return;
    }

    last_sample_us = sample_t;

    /* Pack one sample, MSB-first within each byte. */
    byte_buf = (uint8_t)((byte_buf << 1) | (level_now ? 1u : 0u));
    bit_count++;
    total_samples++;

    if (bit_count == 8) {
        uint32_t head = ring_head;
        if (head - ring_tail >= RING_SIZE) {
            terminate_reason = CZ8RL1_ERR_OVERFLOW;
            running = false;
            return;
        }
        ring_buf[head & RING_MASK] = byte_buf;
        ring_head = head + 1;
        total_bytes++;
        bit_count = 0;
        byte_buf  = 0;
    }

    /* Tape stopped → BUSY high. */
    if (busy_break_enabled && gpio_get(PIN_BUSY)) {               /* PLATFORM */
        terminate_reason = CZ8RL1_STS_BREAK;
        running = false;
        return;
    }

    /* Default schedule. The GPIO ISR may override this if an edge
       lands before this alarm fires. */
    hardware_alarm_set_target((uint)alarm_num,                    /* PLATFORM */
                              from_us_since_boot(sample_t + step_us));
}

/* ---- Public API ----------------------------------------------------- */

void sampler_init(void) {
    if (alarm_num < 0) {
        alarm_num = (int)hardware_alarm_claim_unused(true);       /* PLATFORM */
    }
    hardware_alarm_set_callback((uint)alarm_num, alarm_callback); /* PLATFORM */
}

bool sampler_start(uint32_t sample_rate,
                   uint32_t idle_break_sec,
                   bool     busy_break,
                   double   jitter_low,
                   double   jitter_high) {
    if (running) return false;
    if (sample_rate < 1000 || sample_rate > 200000) return false;

    step_us     = 1000000ULL / sample_rate;
    jit_us[0]   = (uint64_t)(jitter_low  * (double)step_us);
    jit_us[1]   = (uint64_t)(jitter_high * (double)step_us);
    jit_flag[0] = jitter_low  > 0.0;
    jit_flag[1] = jitter_high > 0.0;
    idle_limit  = idle_break_sec * sample_rate;
    busy_break_enabled = busy_break;

    ring_head = ring_tail = 0;
    byte_buf = 0;
    bit_count = 0;
    idle_left = idle_limit;
    edge_level = 0;
    jitter_sum[0] = jitter_sum[1] = 0;
    total_edges = total_bytes = total_samples = 0;
    dropped_edges = 0;
    edge_seen_this_period = false;
    terminate_reason = 0;

    uint64_t now = time_us_64();                                  /* PLATFORM */
    last_sample_us = now;
    running = true;

    gpio_set_irq_enabled_with_callback(PIN_READ_DATA,             /* PLATFORM */
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true, gpio_callback);

    hardware_alarm_set_target((uint)alarm_num,                    /* PLATFORM */
                              from_us_since_boot(now + step_us));
    return true;
}

uint32_t sampler_read(uint8_t *out, uint32_t max_bytes) {
    uint32_t head = ring_head;        /* snapshot — ISR may advance head */
    uint32_t tail = ring_tail;
    uint32_t avail = head - tail;
    if (avail == 0) return 0;
    if (avail > max_bytes) avail = max_bytes;

    uint32_t off   = tail & RING_MASK;
    uint32_t first = RING_SIZE - off;
    if (first > avail) first = avail;
    memcpy(out, &ring_buf[off], first);
    if (avail > first) {
        memcpy(out + first, &ring_buf[0], avail - first);
    }
    ring_tail = tail + avail;
    return avail;
}

bool sampler_running(void) {
    return running;
}

void sampler_stop(sampler_stats_t *stats) {
    running = false;
    hardware_alarm_cancel((uint)alarm_num);                       /* PLATFORM */
    gpio_set_irq_enabled(PIN_READ_DATA,                           /* PLATFORM */
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

    if (!stats) return;
    stats->total_bytes   = total_bytes;
    stats->total_samples = total_samples;
    stats->total_edges   = total_edges;
    stats->dropped_edges = dropped_edges;
    stats->reason        = terminate_reason ? terminate_reason
                                            : CZ8RL1_STS_BREAK;
    if (total_edges == 0 || step_us == 0) {
        stats->jitter_low = stats->jitter_high = 0.0;
    } else {
        /*
            Match the original's reporting: jitter_low is averaged
            over rise-edges (level==1 entries), jitter_high over
            fall-edges (level==0). Divide by step_us for a 0..1
            fraction.
        */
        stats->jitter_low  = (double)(jitter_sum[1] * 2 / total_edges)
                                / (double)step_us;
        stats->jitter_high = (double)(jitter_sum[0] * 2 / total_edges)
                                / (double)step_us;
    }
}
