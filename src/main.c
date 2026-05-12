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
#include "hardware/gpio.h"

#include "cz8rl1.h"
#include "sampler.h"
#include "usb_stream.h"
#include "pins.h"

#define X8RL1_PICO_VERSION "0.1"

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

/* Render `status` as "STS <hex>". Values in 0..255 are the device's raw
   reply byte (printed two digits). Values >=0x100 are internal status
   codes from cz8rl1.h (CZ8RL1_STS_*); print them at full width so the
   host can tell e.g. CZ8RL1_STS_DOAPSS (0x104) apart from the raw byte
   0x04. */
static void send_status(int status) {
    if (status < 0) {
        usb_stream_printf("ERR PROTO %d", status);
    } else if (status > 0xff) {
        usb_stream_printf("STS %x", status);
    } else {
        usb_stream_printf("STS %02x", status);
    }
}

static void handle_cmd(const char *args) {
    unsigned hex;
    if (sscanf(args, "%x", &hex) != 1) {
        send_err("PARSE", "expected hex code");
        return;
    }
    send_status(cz8rl1_send_command((uint8_t)hex));
}

static void handle_status(void) { send_status(cz8rl1_send_command(CZ8RL1_STATUS)); }
static void handle_sensor(void) { send_status(cz8rl1_send_command(CZ8RL1_SENSOR)); }

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

/*
    Diagnostic: sample the three input pins for `duration_ms` milliseconds
    in a tight loop, report instantaneous level + how many samples were
    HIGH for each. If `high_count` is near 0 or near `total`, the line is
    stuck; intermediate values mean it's toggling (real protocol activity
    or noise).
*/
static void handle_pins(const char *args) {
    unsigned duration_ms = 10;
    if (*args) sscanf(args, "%u", &duration_ms);
    if (duration_ms < 1) duration_ms = 1;
    if (duration_ms > 1000) duration_ms = 1000;

    uint32_t busy_hi = 0, status_hi = 0, rdata_hi = 0, total = 0;
    absolute_time_t end = make_timeout_time_ms(duration_ms);
    while (!time_reached(end)) {
        if (gpio_get(PIN_BUSY))      busy_hi++;
        if (gpio_get(PIN_STATUS))    status_hi++;
        if (gpio_get(PIN_READ_DATA)) rdata_hi++;
        total++;
    }

    /* Also report the current output drive levels so you can confirm
       the firmware is leaving outputs in the idle state we expect. */
    bool wr  = gpio_get_out_level(PIN_WRITE_DATA);
    bool stb = gpio_get_out_level(PIN_STROBE);
    bool cmd = gpio_get_out_level(PIN_COMMAND);

    usb_stream_printf(
        "PINS busy=%u/%u status=%u/%u rdata=%u/%u "
        "out: wdata=%d strobe=%d cmd=%d (samples=%u, dur=%ums)",
        busy_hi, total, status_hi, total, rdata_hi, total,
        wr ? 1 : 0, stb ? 1 : 0, cmd ? 1 : 0,
        total, duration_ms);
}

/*
    Diagnostic: instantaneous one-shot read of the three input pins.
*/
static void handle_level(void) {
    usb_stream_printf("LEVEL busy=%d status=%d rdata=%d",
                      gpio_get(PIN_BUSY) ? 1 : 0,
                      gpio_get(PIN_STATUS) ? 1 : 0,
                      gpio_get(PIN_READ_DATA) ? 1 : 0);
}

/*
    Diagnostic: drive a single output pin to a given level so it can be
    probed with a multimeter. Syntax: POKE <wdata|strobe|cmd> <0|1>
    Does NOT call set_idle() afterwards; the pin stays at the chosen
    level until another POKE or until cz8rl1_init runs again.
*/
static void handle_poke(const char *args) {
    char name[16] = {0};
    unsigned level = 0;
    if (sscanf(args, "%15s %u", name, &level) != 2) {
        send_err("PARSE", "expected: POKE <wdata|strobe|cmd> <0|1>");
        return;
    }
    uint pin;
    if      (strcmp(name, "wdata")  == 0) pin = PIN_WRITE_DATA;
    else if (strcmp(name, "strobe") == 0) pin = PIN_STROBE;
    else if (strcmp(name, "cmd")    == 0) pin = PIN_COMMAND;
    else { send_err("PIN", "unknown pin name"); return; }

    gpio_put(pin, level ? 1 : 0);
    usb_stream_printf("POKE %s=%u (gpio %u)", name, level ? 1 : 0, pin);
}

/*
    Diagnostic: send a command byte, then capture every transition on
    BUSY/STATUS/READ_DATA for the given duration (microseconds). Reports
    initial state and every observed edge with timestamps relative to
    the start of tx. One line, suitable for parsing on the host.
*/
#define TRACE_MAX_EVENTS 512
static cz8rl1_trace_event_t trace_events[TRACE_MAX_EVENTS];

static void handle_trace(const char *args) {
    unsigned hex;
    unsigned duration_us = 100000;  /* 100 ms default */
    int n = sscanf(args, "%x %u", &hex, &duration_us);
    if (n < 1) {
        send_err("PARSE", "expected: TRACE <hex_cmd> [duration_us]");
        return;
    }
    if (duration_us < 1000)     duration_us = 1000;
    if (duration_us > 2000000)  duration_us = 2000000;

    uint8_t initial = 0;
    uint32_t count = cz8rl1_trace_command((uint8_t)hex, duration_us,
                                          trace_events, TRACE_MAX_EVENTS,
                                          &initial);

    /* Header line: initial state + count + duration. */
    usb_stream_printf("TRACE cmd=%02x init=%u dur=%u count=%u",
                      hex, initial, duration_us, count);

    /* One line per event: "EVT <t_us> <pin_state>" — keeps lines short
       and easy to parse. pin_state bit 0=BUSY, 1=STATUS, 2=READ_DATA. */
    for (uint32_t i = 0; i < count; i++) {
        usb_stream_printf("EVT %u %u",
                          trace_events[i].t_us,
                          trace_events[i].pin_state);
    }
    usb_stream_write_line("TRACE_END");
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

    if      (strcmp(line, "PING")   == 0) usb_stream_printf(
                                              "PONG x8rl1-pico %s",
                                              X8RL1_PICO_VERSION);
    else if (strcmp(line, "CMD")    == 0) handle_cmd(args);
    else if (strcmp(line, "STATUS") == 0) handle_status();
    else if (strcmp(line, "SENSOR") == 0) handle_sensor();
    else if (strcmp(line, "READ")   == 0) handle_read(args);
    else if (strcmp(line, "WRITE")  == 0) handle_write(args);
    else if (strcmp(line, "PINS")   == 0) handle_pins(args);
    else if (strcmp(line, "LEVEL")  == 0) handle_level();
    else if (strcmp(line, "POKE")   == 0) handle_poke(args);
    else if (strcmp(line, "TRACE")  == 0) handle_trace(args);
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
