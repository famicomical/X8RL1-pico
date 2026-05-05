#include <stdio.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include "usb_stream.h"

void usb_stream_init(void) {
    stdio_init_all();
}

void usb_stream_wait_for_host(void) {
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
}

int usb_stream_read_line(char *buf, size_t buf_size, uint32_t timeout_ms) {
    size_t pos = 0;
    absolute_time_t deadline = (timeout_ms > 0)
        ? make_timeout_time_ms(timeout_ms)
        : at_the_end_of_time;

    while (pos + 1 < buf_size) {
        int64_t remaining_us = absolute_time_diff_us(get_absolute_time(), deadline);
        if (remaining_us <= 0 && timeout_ms > 0) {
            return -1;
        }
        uint32_t poll_us = (timeout_ms > 0 && remaining_us < 10000)
            ? (uint32_t)remaining_us
            : 10000;

        int c = getchar_timeout_us(poll_us);
        if (c == PICO_ERROR_TIMEOUT) continue;

        if (c == '\n' || c == '\r') {
            if (pos == 0) continue;  /* skip blank lines / lone CR */
            break;
        }
        buf[pos++] = (char)c;
    }
    buf[pos] = '\0';
    return (int)pos;
}

void usb_stream_write_line(const char *line) {
    fputs(line, stdout);
    fputc('\n', stdout);
    fflush(stdout);
}

void usb_stream_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fputc('\n', stdout);
    fflush(stdout);
}

void usb_stream_write_bytes(const uint8_t *buf, size_t len) {
    /*
        fwrite uses the stdio_usb path which buffers and flushes in
        chunks; this is fine for streaming sample data.
    */
    fwrite(buf, 1, len, stdout);
}

void usb_stream_flush(void) {
    fflush(stdout);
}
