# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PoC for sending steering and power commands from an NVIDIA Orin to a Raspberry Pi Pico over I2C. Both devices will be on-board an RC car.

## Hardware Setup

- **NVIDIA Orin Nano**: I2C master (bus 7), runs Python client
- **Raspberry Pi Pico**: I2C slave (address `0x42`), runs C firmware using Pico SDK
- **Power**: Pico powered via USB from Orin; Orin powered via USB-C PD from battery bank
- **Drive system**: 7.4V 2S LiPo (separate from Orin power)

### Pinout

| Orin Nano (40-pin) | Pico | Function |
|--------------------|------|----------|
| Pin 3 | GP0 (Pin 1) | SDA (I2C Bus 7) |
| Pin 5 | GP1 (Pin 2) | SCL (I2C Bus 7) |
| Pin 6 | GND (Pin 3) | Ground |
| USB-A | USB | Power |

### Servo Output

| Pico GPIO | Function | Calibration |
|-----------|----------|-------------|
| GP2 (Pin 4) | Steering servo | 1470µs center, ±200µs |

## I2C Register Map

| Register | Address | R/W | Description | Value Range |
|----------|---------|-----|-------------|-------------|
| STEERING | `0x00` | R/W | Steering angle | 0-255 (128 = center) |
| POWER | `0x01` | R/W | Motor power | 0-255 (128 = stop, <128 = reverse, >128 = forward) |
| STATUS | `0x02` | R | Device status | Bit flags (0x01 = ready) |
| WHO_AM_I | `0x0F` | R | Device ID | Fixed `0x42` |

## Project Structure

```
pico/
  main.c                  # C firmware source
  CMakeLists.txt          # Build configuration
  pico_sdk_import.cmake   # Pico SDK import helper
orin/
  client.py               # Python I2C master client
```

## Development

### Pico Build (requires Pico SDK)

```bash
# Set SDK path (or use PICO_SDK_FETCH_FROM_GIT)
export PICO_SDK_PATH=/path/to/pico-sdk

# Build
cd pico
mkdir build && cd build
cmake ..
make

# Flash: copy build/rc_car_controller.uf2 to Pico in BOOTSEL mode
```

### Orin Setup

```bash
pip install smbus2
python orin/client.py
```

## TODO

- Add motor/ESC PWM control
- Add error handling and reconnection logic
- Add safety timeout (stop if no commands received)
