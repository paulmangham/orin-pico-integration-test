# Orin-Pico RC Car Controller

PoC for sending steering and power commands from an NVIDIA Orin to a Raspberry Pi Pico over I2C.

## Hardware Setup

### Orin Nano Pinout (40-pin header)

| Pin | Function |
|-----|----------|
| 3 | SDA (I2C Bus 7) |
| 5 | SCL (I2C Bus 7) |
| 6 | GND |

### Pico Pinout

| GPIO | Pin | Function |
|------|-----|----------|
| GP0 | 1 | SDA (I2C) |
| GP1 | 2 | SCL (I2C) |
| GND | 3 | Ground |
| GP2 | 4 | Steering servo PWM |

### Wiring Summary

| Orin (40-pin) | Pico | Notes |
|---------------|------|-------|
| Pin 3 (SDA) | GP0 (Pin 1) | I2C data |
| Pin 5 (SCL) | GP1 (Pin 2) | I2C clock |
| Pin 6 (GND) | GND (Pin 3) | Common ground |
| USB-A | USB | Power to Pico |

### Servo Connections

| Servo Wire | Connection |
|------------|------------|
| Signal | Pico GP2 (Pin 4) |
| Power | External 5-6V supply |
| Ground | Common ground with Pico |

### Power

- Pico: Powered via USB from Orin
- Orin: USB-C PD from battery bank
- Servos/ESC: 7.4V 2S LiPo (separate)

### Steering Servo Calibration

- Center: 1470µs
- Range: ±200µs (1270-1670µs)

## Building the Pico Firmware

### Prerequisites

- CMake 3.13+
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- Pico SDK (auto-downloaded or set `PICO_SDK_PATH`)

### Build

```bash
cd pico
mkdir build && cd build
cmake -DPICO_SDK_FETCH_FROM_GIT=ON ..
make
```

### Flash

1. Hold **BOOTSEL** on the Pico
2. Plug in USB (Pico mounts as `RPI-RP2`)
3. Copy `build/rc_car_controller.uf2` to the drive
4. Pico reboots automatically

### Debug Console

The Pico outputs debug messages over USB serial:

```bash
screen /dev/ttyACM0 115200
```

Exit screen: `Ctrl+A` then `K` then `Y`

## Orin Setup

```bash
cd orin
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python client.py
```

## I2C Register Map

| Register | Address | R/W | Description |
|----------|---------|-----|-------------|
| STEERING | 0x00 | R/W | 0-255 (128 = center) |
| POWER | 0x01 | R/W | 0-255 (128 = stop) |
| STATUS | 0x02 | R | Status flags |
| WHO_AM_I | 0x0F | R | Device ID (0x42) |
