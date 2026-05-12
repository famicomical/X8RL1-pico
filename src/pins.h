#ifndef PINS_H
#define PINS_H

/*
    GPIO pin assignments for CZ8RL1 interface.

    CZ8RL1 DIN-7 connector:
      1. WRITE_DATA   (Pico output)
      2. STROBE       (Pico output)
      3. BUSY         (Pico input,  pullup)
      4. READ_DATA    (Pico input,  pullup)
      5. STATUS_DATA  (Pico input,  pullup)
      6. GND
      7. COMMAND_DATA (Pico output)
*/

#define PIN_WRITE_DATA    9
#define PIN_STROBE       10
#define PIN_COMMAND      11

#define PIN_BUSY          8
#define PIN_READ_DATA     6
#define PIN_STATUS        7

#define PIN_OUTPUT_MASK  ((1u << PIN_WRITE_DATA) | (1u << PIN_STROBE) | (1u << PIN_COMMAND))
#define PIN_INPUT_MASK   ((1u << PIN_BUSY) | (1u << PIN_READ_DATA) | (1u << PIN_STATUS))

#endif
