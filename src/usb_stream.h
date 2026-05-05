#ifndef USB_STREAM_H
#define USB_STREAM_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*
    USB CDC framing helpers.

    Protocol (text + binary, line-oriented):
      Host -> Pico: "<CMD> <args...>\n"
      Pico -> Host: text status lines, binary blocks framed by length.
*/

void usb_stream_init(void);

/* Block until host opens the CDC port (DTR asserted). */
void usb_stream_wait_for_host(void);

/*
    Read a line into buf. Stops on '\n', '\r', or when buf is full.
    Returns length (excluding terminator), or -1 on timeout.
    timeout_ms = 0 means wait forever.
*/
int usb_stream_read_line(char *buf, size_t buf_size, uint32_t timeout_ms);

/* Send a text line (appends '\n'). */
void usb_stream_write_line(const char *line);

/* printf-style line, automatically terminated. */
void usb_stream_printf(const char *fmt, ...);

/* Write raw bytes (no escaping, no terminator). */
void usb_stream_write_bytes(const uint8_t *buf, size_t len);

/* Flush USB CDC output. */
void usb_stream_flush(void);

#endif
