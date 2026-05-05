/*
    X8RL1-Pico: CZ8RL1 controller running on a Raspberry Pi Pico,
    streaming sampled tape data to a host PC over USB CDC.

    Wire protocol (line-oriented text with framed binary blocks):

        Host -> Pico:
            PING
            CMD <hex>
            STATUS
            SENSOR
            READ <rate_hz> <max_seconds> <idle_break_sec>
            WRITE <rate_hz> <length_bytes>
            <binary length_bytes>          (after WRITE header acknowledged)

        Pico -> Host:
            READY
            PONG
            OK
            ERR <code> <message>
            STS <hex>
            DATA <length_bytes>            (binary follows immediately)
            STATS edges=N bytes=B jl=F jh=F
            END
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"

#include "cz8rl1.h"
#include "sampler.h"
#include "usb_stream.h"
#include "pins.h"

/*
    READ uses sampler.c — a hardware timer + GPIO IRQ keep sampling
    continuously, with the original software PLL applied in the GPIO
    ISR. Main loop just drains the ring buffer to USB.

    WRITE still uses a single fixed buffer (playback is short-lived
    and bounded by the .TAP file size in practice).
*/
#define DRAIN_BUF_SIZE  512
#define WRITE_BUF_SIZE  (64 * 1024)
static uint8_t drain_buf[DRAIN_BUF_SIZE];
static uint8_t write_buf[WRITE_BUF_SIZE];

#define LINE_BUF_SIZE 128

static void send_err(const char *code, const char *msg) {
    usb_stream_printf("ERR %s %s", code, msg);
}

static void handle_cmd(const char *args) {
    unsigned hex;
    if (sscanf(args, "%x", &hex) != 1) {
        send_err("PARSE", "expected hex code");
        return;
    }
    int status = cz8rl1_send_command((uint8_t)hex);
    if (status < 0) {
        usb_stream_printf("ERR PROTO %d", status);
    } else {
        usb_stream_printf("STS %02x", status & 0xff);
    }
}

static void handle_status(void)  { usb_stream_printf("STS %02x",
                                       cz8rl1_send_command(CZ8RL1_STATUS) & 0xff); }
static void handle_sensor(void)  { usb_stream_printf("STS %02x",
                                       cz8rl1_send_command(CZ8RL1_SENSOR) & 0xff); }

static void drain_to_usb(uint64_t *bytes_sent, uint64_t max_bytes) {
    for (;;) {
        uint32_t want = DRAIN_BUF_SIZE;
        if (max_bytes > 0 && *bytes_sent + want > max_bytes) {
            want = (uint32_t)(max_bytes - *bytes_sent);
            if (want == 0) return;
        }
        uint32_t n = sampler_read(drain_buf, want);
        if (n == 0) return;
        usb_stream_printf("CHUNK %u", n);
        usb_stream_write_bytes(drain_buf, n);
        usb_stream_flush();
        *bytes_sent += n;
    }
}

static void handle_read(const char *args) {
    unsigned rate, max_sec, idle_sec;
    if (sscanf(args, "%u %u %u", &rate, &max_sec, &idle_sec) != 3) {
        send_err("PARSE", "expected: READ <rate> <max_sec> <idle_sec>");
        return;
    }

    if (!sampler_start(rate, idle_sec, /* busy_break = */ true,
                       /* jitter_low = */ 0.5, /* jitter_high = */ 0.5)) {
        send_err("START", "sampler_start failed (rate or already running)");
        return;
    }

    usb_stream_write_line("OK STREAM");

    uint64_t max_bytes = (uint64_t)rate * max_sec / 8;
    uint64_t bytes_sent = 0;

    /*
        Sampler runs autonomously in IRQ context; main loop just
        drains the ring buffer. If the host wedged, the sampler will
        eventually trip CZ8RL1_ERR_OVERFLOW and stop.
    */
    while (sampler_running()) {
        drain_to_usb(&bytes_sent, max_bytes);
        if (max_bytes > 0 && bytes_sent >= max_bytes) break;
    }

    sampler_stats_t stats;
    sampler_stop(&stats);

    /* Drain any tail still in the ring buffer after the ISR stopped. */
    drain_to_usb(&bytes_sent, max_bytes);

    usb_stream_printf("STATS edges=%u bytes=%u samples=%u jl=%.3f jh=%.3f result=%d",
                      stats.total_edges, stats.total_bytes, stats.total_samples,
                      stats.jitter_low, stats.jitter_high, stats.reason);
    usb_stream_write_line("END");
}

static void handle_write(const char *args) {
    unsigned rate, length;
    if (sscanf(args, "%u %u", &rate, &length) != 2) {
        send_err("PARSE", "expected: WRITE <rate> <length>");
        return;
    }
    if (length > WRITE_BUF_SIZE) {
        send_err("SIZE", "exceeds buffer");
        return;
    }

    /* Acknowledge so host starts streaming binary payload. */
    usb_stream_printf("OK %u", length);
    usb_stream_flush();

    /* Read exactly `length` bytes from stdin. */
    size_t got = 0;
    while (got < length) {
        int c = getchar_timeout_us(5 * 1000 * 1000);  /* 5s per byte */
        if (c == PICO_ERROR_TIMEOUT) {
            send_err("TIMEOUT", "incomplete payload");
            return;
        }
        write_buf[got++] = (uint8_t)c;
    }

    int result = cz8rl1_write_data(write_buf, length, rate, /* busy_break = */ true);
    usb_stream_printf("RESULT %d", result);
    usb_stream_write_line("END");
}

static void dispatch(char *line) {
    /* split command from args */
    char *args = strchr(line, ' ');
    if (args) {
        *args++ = '\0';
        while (*args == ' ') args++;
    } else {
        args = "";
    }

    if      (strcmp(line, "PING")   == 0) usb_stream_write_line("PONG");
    else if (strcmp(line, "CMD")    == 0) handle_cmd(args);
    else if (strcmp(line, "STATUS") == 0) handle_status();
    else if (strcmp(line, "SENSOR") == 0) handle_sensor();
    else if (strcmp(line, "READ")   == 0) handle_read(args);
    else if (strcmp(line, "WRITE")  == 0) handle_write(args);
    else                                  send_err("UNKNOWN", line);
}

int main(void) {
    usb_stream_init();
    usb_stream_wait_for_host();

    cz8rl1_init();
    sampler_init();
    usb_stream_write_line("READY");

    char line[LINE_BUF_SIZE];
    for (;;) {
        int n = usb_stream_read_line(line, sizeof line, 0);
        if (n <= 0) continue;
        dispatch(line);
    }
}
