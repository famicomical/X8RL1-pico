# x8rl1-pico

Raspberry Pi Pico port of [X8RL1](../X8RL1), a controller for the
Sharp CZ-8RL1 cassette data recorder used with the X1 turbo. The Pico
takes the place of the PC parallel port: it bit-bangs the CZ-8RL1
serial protocol on its GPIOs and streams sampled tape data to a host
PC over USB CDC. The host writes `.TAP` image files.

## Wiring

CZ-8RL1 DIN-7  →  Pico GPIO (truth is `src/pins.h`)

| DIN-7 | Signal       | Pico GPIO | Header pin | Direction       |
|-------|--------------|-----------|------------|-----------------|
| 1     | WRITE_DATA   | GPIO 9    | pin 12     | Pico → CZ-8RL1  |
| 2     | STROBE       | GPIO 10   | pin 14     | Pico → CZ-8RL1  |
| 3     | BUSY         | GPIO 8    | pin 11     | CZ-8RL1 → Pico  |
| 4     | READ_DATA    | GPIO 6    | pin 9      | CZ-8RL1 → Pico  |
| 5     | STATUS_DATA  | GPIO 7    | pin 10     | CZ-8RL1 → Pico  |
| 6     | GND          | GND       | pin 3/8/…  | —               |
| 7     | COMMAND_DATA | GPIO 11   | pin 15     | Pico → CZ-8RL1  |

The Pico's internal pull-ups are enabled on the three input pins. The
CZ-8RL1's outputs appear to be open-collector, so the line is naturally
clamped to 3.3 V — no level shifter is needed on the inputs. The three
output pins drive 3.3 V; if the CZ-8RL1 turns out to need 5 V logic
HIGH on its inputs, level shifters will be needed there.

## Build

```sh
export PICO_SDK_PATH=~/pico/pico-sdk
mkdir build && cd build
cmake ..
make -j
```

The result is `build/x8rl1_pico.uf2`. Hold BOOTSEL while plugging in
the Pico, then drop the `.uf2` onto the mounted RPI-RP2 volume.

## Host usage

```sh
pip install pyserial
python host/x8rl1-pico.py --port /dev/cu.usbmodem1234 status
python host/x8rl1-pico.py --port /dev/cu.usbmodem1234 read tape.tap
python host/x8rl1-pico.py --port /dev/cu.usbmodem1234 write tape.tap
python host/x8rl1-pico.py --port /dev/cu.usbmodem1234 cmd 02   # PLAY
```

## Limitations vs. the original

- Single-shot sampling: the on-board buffer is 64 KB, so a single
  `read` captures up to 64 s at 8 kHz. Long recordings will require
  chunked streaming (not yet implemented).
- `.TAP` reader/writer is X1EMU format only. XMillenium T-Tune
  headers are not handled host-side yet.
