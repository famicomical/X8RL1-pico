#ifndef CZ8RL1_H
#define CZ8RL1_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* CZ8RL1 commands */
#define CZ8RL1_EJECT  0x00
#define CZ8RL1_STOP   0x01
#define CZ8RL1_PLAY   0x02
#define CZ8RL1_FF     0x03
#define CZ8RL1_REW    0x04
#define CZ8RL1_AFF    0x05
#define CZ8RL1_AREW   0x06
#define CZ8RL1_REC    0x0A
#define CZ8RL1_STATUS 0x80
#define CZ8RL1_SENSOR 0x81
#define CZ8RL1_BREAK  0xff

/* status / error codes (return values from cz8rl1_send_command) */
#define CZ8RL1_STS_OK          0x100
#define CZ8RL1_STS_AUTOSTOP    0x101
#define CZ8RL1_STS_BUFFULL     0x102
#define CZ8RL1_STS_BREAK       0x103
#define CZ8RL1_STS_DOAPSS      0x104

#define CZ8RL1_ERR_NOTFOUND   -1
#define CZ8RL1_ERR_BSYACK     -2
#define CZ8RL1_ERR_DOAPSS     -3
#define CZ8RL1_ERR_NOHEADER   -4
#define CZ8RL1_ERR_BADHEAD    -5
#define CZ8RL1_ERR_BADBIT     -6
#define CZ8RL1_ERR_SPEED1    -10
#define CZ8RL1_ERR_SPEED2    -11
#define CZ8RL1_ERR_OVERFLOW  -12   /* ring buffer overrun (host too slow) */

void cz8rl1_init(void);

/* Send a command to the CZ8RL1, return status (0..255) or negative error. */
int cz8rl1_send_command(uint8_t command);

/*
    Play buf out the WRITE_DATA line at the given rate, MSB-first per byte.
    busy_break: if true, stops when BUSY goes high.
*/
int cz8rl1_write_data(const uint8_t *buf,
                      uint32_t buf_size,
                      uint32_t sample_rate,
                      bool busy_break);

#endif
