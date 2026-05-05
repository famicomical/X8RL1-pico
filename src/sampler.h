#ifndef SAMPLER_H
#define SAMPLER_H

#include <stdint.h>
#include <stdbool.h>

/*
    Continuous tape sampler.

    A periodic hardware timer drives an ISR that, on every fire, reads
    READ_DATA, packs one bit into a byte buffer, and writes a byte to
    the ring buffer when full. A GPIO IRQ on READ_DATA records each
    edge timestamp and shifts the next alarm to keep samples mid-bit
    (the software PLL from the original DOS code, but the alarm is
    rescheduled directly so the correction takes effect on the *next*
    sample, not the one after — same convergence as the polling loop).

    Single core, single IRQ priority. Portable: any MCU with a
    one-shot hardware timer + edge-triggered GPIO interrupts can run
    this — only sampler.c contains platform calls.

    The main loop calls sampler_read() to drain the ring buffer and
    push to USB. Sampling never pauses for USB; if the host stalls
    long enough that the ring fills, the ISR aborts with
    CZ8RL1_ERR_OVERFLOW so we never silently lose tape time.
*/

typedef struct {
    uint32_t total_bytes;
    uint32_t total_samples;
    uint32_t total_edges;
    int      reason;          /* CZ8RL1_STS_* / CZ8RL1_ERR_* */
    double   jitter_low;      /* avg fall-edge offset / sample period */
    double   jitter_high;     /* avg rise-edge offset / sample period */
} sampler_stats_t;

void sampler_init(void);

/*
    Begin a sampling session. Returns false if already running or if
    sample_rate is out of range. jitter_low/high are fractions in
    [0..1] for PLL phase offsets per polarity (pass 0.5 / 0.5 for
    mid-bit sampling). Set either to 0 to disable PLL on that polarity.
*/
bool sampler_start(uint32_t sample_rate,
                   uint32_t idle_break_sec,
                   bool     busy_break,
                   double   jitter_low,
                   double   jitter_high);

/* Drain up to max_bytes from the ring buffer; returns bytes copied. */
uint32_t sampler_read(uint8_t *out, uint32_t max_bytes);

/* True while the ISR is still sampling. */
bool sampler_running(void);

/* Stop sampling and fill stats. Safe to call when already stopped. */
void sampler_stop(sampler_stats_t *stats);

#endif
