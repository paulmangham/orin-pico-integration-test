"""
Orin I2C Client for RC Car Control

Communicates with Raspberry Pi Pico over I2C to control steering and power.
"""

import smbus2
import time

# I2C Configuration
I2C_BUS = 7  # /dev/i2c-7 on Jetson Orin Nano (pins 3/5)
PICO_ADDR = 0x42

# Register addresses
REG_STEERING = 0x00
REG_POWER = 0x01
REG_STATUS = 0x02
REG_WHO_AM_I = 0x0F

# Expected device ID
EXPECTED_WHO_AM_I = 0x42


class PicoController:
    """I2C client for controlling the Pico-based RC car controller."""

    def __init__(self, bus_num=I2C_BUS, addr=PICO_ADDR):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr

    def close(self):
        """Close the I2C bus."""
        self.bus.close()

    def verify_connection(self):
        """Verify connection by reading WHO_AM_I register.

        Returns:
            bool: True if device responds with correct ID
        """
        try:
            who = self.bus.read_byte_data(self.addr, REG_WHO_AM_I)
            return who == EXPECTED_WHO_AM_I
        except OSError:
            return False

    def get_status(self):
        """Read device status register.

        Returns:
            int: Status byte
        """
        return self.bus.read_byte_data(self.addr, REG_STATUS)

    def set_steering(self, value):
        """Set steering position.

        Args:
            value: 0-255, where 128 is center, 0 is full left, 255 is full right
        """
        value = max(0, min(255, int(value)))
        self.bus.write_byte_data(self.addr, REG_STEERING, value)

    def get_steering(self):
        """Read current steering value.

        Returns:
            int: Current steering value (0-255)
        """
        return self.bus.read_byte_data(self.addr, REG_STEERING)

    def set_power(self, value):
        """Set motor power.

        Args:
            value: 0-255, where 128 is stop, <128 is reverse, >128 is forward
        """
        value = max(0, min(255, int(value)))
        self.bus.write_byte_data(self.addr, REG_POWER, value)

    def get_power(self):
        """Read current power value.

        Returns:
            int: Current power value (0-255)
        """
        return self.bus.read_byte_data(self.addr, REG_POWER)

    def stop(self):
        """Stop the car (steering center, power off)."""
        self.set_steering(128)
        self.set_power(128)

    def drive(self, steering, power):
        """Set both steering and power in one call.

        Args:
            steering: 0-255 (128 = center)
            power: 0-255 (128 = stop)
        """
        self.set_steering(steering)
        self.set_power(power)


def main():
    """Demo/test routine."""
    print("Initializing Pico Controller...")
    controller = PicoController()

    # Verify connection
    if not controller.verify_connection():
        print("ERROR: Could not connect to Pico!")
        print(f"Expected device at address 0x{PICO_ADDR:02X}")
        return

    print("Connected to Pico!")
    print(f"Status: 0x{controller.get_status():02X}")

    try:
        # Demo sequence
        print("\nRunning demo sequence...")

        # Center steering, stop
        print("Centering steering, stopping...")
        controller.stop()
        time.sleep(1)

        # Turn left
        print("Turning left...")
        controller.set_steering(64)  # Left
        time.sleep(1)

        # Turn right
        print("Turning right...")
        controller.set_steering(192)  # Right
        time.sleep(1)

        # Center and forward
        print("Forward...")
        controller.set_steering(128)
        controller.set_power(180)  # Forward ~40%
        time.sleep(1)

        # Reverse
        print("Reverse...")
        controller.set_power(76)  # Reverse ~40%
        time.sleep(1)

        # Stop
        print("Stopping...")
        controller.stop()

        print("\nDemo complete!")

    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        controller.stop()
        controller.close()


if __name__ == "__main__":
    main()
