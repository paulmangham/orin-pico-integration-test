# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PoC for sending steering and power commands from an NVIDIA Orin to a Raspberry Pi Pico over I2C. Both devices will be on-board an RC car.

## Hardware Setup

- **NVIDIA Orin**: I2C master, runs Python client
- **Raspberry Pi Pico**: I2C slave (address `0x42`), runs C firmware using Pico SDK
- **Power**: Pico powered via USB from Orin; Orin powered via USB-C PD from battery bank
- **Drive system**: 7.4V 2S LiPo (separate from Orin power)

### I2C Wiring

| Orin | Pico |
|------|------|
| SDA | GP0 |
| SCL | GP1 |
| GND | GND |

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

- Add actual servo/motor PWM control in Pico firmware
- Add error handling and reconnection logic
