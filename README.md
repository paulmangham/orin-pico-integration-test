# Orin-Pico RC Car Controller

PoC for sending steering and power commands from an NVIDIA Orin to a Raspberry Pi Pico over I2C.

## Hardware Setup

### Wiring

| Orin | Pico |
|------|------|
| SDA | GP0 |
| SCL | GP1 |
| GND | GND |
| USB-A | USB (power) |

### Power

- Pico: Powered via USB from Orin
- Orin: USB-C PD from battery bank
- Motors: 7.4V 2S LiPo (separate)

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
