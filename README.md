# X8RL1-pico

Raspberry Pi Pico port of [X8RL1](http://x1center.org/test.html), a controller for the
Sharp CZ-8RL1 cassette data recorder used with the X1 turbo. The Pico
takes the place of the PC parallel port: it bit-bangs the CZ-8RL1
serial protocol on its GPIOs and streams sampled tape data to a host
PC over USB CDC. The host writes `.TAP` image files.

## Wiring

CZ-8RL1 DIN-7  →  Pico GPIO (truth is `src/pins.h`)
```
| DIN-7 | Signal       | Pico GPIO | Direction       |
|-------|--------------|-----------|-----------------|
| 1     | WRITE_DATA   | GPIO 9    | pico → CZ-8RL1  |
| 2     | STROBE       | GPIO 10   | pico → CZ-8RL1  |
| 3     | BUSY         | GPIO 8    | CZ-8RL1 → pico  |
| 4     | READ_DATA    | GPIO 6    | CZ-8RL1 → pico  |
| 5     | STATUS_DATA  | GPIO 7    | CZ-8RL1 → pico  |
| 6     | GND          | GND       | —               |
| 7     | COMMAND_DATA | GPIO 11   | pico → CZ-8RL1  |
```
The CZ-8RL1's outputs require a basic MOS level shifter to connect to the pico's 3.3V bus. No level shifting is required for the pico's output pins, since the CZ-8RL1's inputs are TTL.

## Build & Flash

```sh
export PICO_SDK_PATH=~/pico/pico-sdk
mkdir build && cd build
cmake ..
make -j
```

The result is `build/x8rl1_pico.uf2`. Hold BOOTSEL while plugging in
the Pico, then drop the `.uf2` onto the mounted RPI-RP2 volume.

Or install picotool and run `picotool load -f x8rl1_pico.uf2 -x`

## Host usage

```sh
pip install pyserial
python host/x8rl1-pico.py status
python host/x8rl1-pico.py read tape.tap  # Dump a tape 
python host/x8rl1-pico.py write tape.tap # Write out a .tap
python host/x8rl1-pico.py cmd 02         # PLAY
```

