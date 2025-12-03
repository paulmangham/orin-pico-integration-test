/**
 * RC Car I2C Slave Controller
 *
 * Raspberry Pi Pico firmware that receives steering and power commands
 * from an NVIDIA Orin over I2C.
 *
 * Registers:
 *   0x00 - STEERING (R/W): 0-255, 128 = center
 *   0x01 - POWER (R/W): 0-255, 128 = stop, <128 = reverse, >128 = forward
 *   0x02 - STATUS (R): Status flags (0x01 = ready)
 *   0x0F - WHO_AM_I (R): Device ID (0x42)
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/i2c_slave.h"

// LED
#define LED_PIN 25

// I2C Configuration
#define I2C_SLAVE_ADDRESS 0x42
#define I2C_BAUDRATE      100000  // 100 kHz
#define I2C_SLAVE_SDA_PIN 0
#define I2C_SLAVE_SCL_PIN 1

// Register addresses
#define REG_STEERING  0x00
#define REG_POWER     0x01
#define REG_STATUS    0x02
#define REG_WHO_AM_I  0x0F
#define REG_COUNT     16

// Register storage
static struct {
    uint8_t regs[REG_COUNT];
    uint8_t reg_address;
    bool reg_address_written;
} context;

// I2C activity flag (set in interrupt, cleared in main loop)
static volatile bool i2c_activity = false;

// Read-only register mask (1 = read-only)
static const uint8_t read_only_mask[REG_COUNT] = {
    [REG_STATUS]   = 1,
    [REG_WHO_AM_I] = 1,
};

// Forward declarations
static void apply_steering(uint8_t value);
static void apply_power(uint8_t value);

/**
 * I2C slave event handler - called from interrupt context
 */
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE:  // Master has written data
            if (!context.reg_address_written) {
                // First byte is the register address
                context.reg_address = i2c_read_byte_raw(i2c);
                context.reg_address_written = true;
            } else {
                // Subsequent bytes are data to write
                uint8_t data = i2c_read_byte_raw(i2c);
                if (context.reg_address < REG_COUNT &&
                    !read_only_mask[context.reg_address]) {
                    context.regs[context.reg_address] = data;
                }
                context.reg_address++;
            }
            break;

        case I2C_SLAVE_REQUEST:  // Master is requesting data
            if (context.reg_address < REG_COUNT) {
                i2c_write_byte_raw(i2c, context.regs[context.reg_address]);
            } else {
                i2c_write_byte_raw(i2c, 0xFF);  // Invalid register
            }
            context.reg_address++;
            break;

        case I2C_SLAVE_FINISH:  // Transaction complete
            context.reg_address_written = false;
            i2c_activity = true;
            break;

        default:
            break;
    }
}

/**
 * Apply steering value to servo
 *
 * @param value 0-255, where 128 is center
 */
static void apply_steering(uint8_t value) {
    // Convert 0-255 to servo pulse width
    // Typical servo: 1000-2000us pulse, 128 = 1500us (center)
    // TODO: Implement actual PWM output to servo

    int angle = (int)value - 128;  // -128 to +127
    printf("Steering: %d (raw: %d)\n", angle, value);
}

/**
 * Apply power value to motor
 *
 * @param value 0-255, where 128 = stop, <128 = reverse, >128 = forward
 */
static void apply_power(uint8_t value) {
    // TODO: Implement actual motor control (ESC PWM or H-bridge)

    if (value == 128) {
        printf("Power: STOP\n");
    } else if (value > 128) {
        int power_pct = (value - 128) * 100 / 127;
        printf("Power: FORWARD %d%%\n", power_pct);
    } else {
        int power_pct = (128 - value) * 100 / 128;
        printf("Power: REVERSE %d%%\n", power_pct);
    }
}

/**
 * Initialize registers with default values
 */
static void init_registers(void) {
    memset(&context, 0, sizeof(context));

    context.regs[REG_STEERING] = 128;  // Center
    context.regs[REG_POWER]    = 128;  // Stop
    context.regs[REG_STATUS]   = 0x01; // Ready
    context.regs[REG_WHO_AM_I] = 0x42; // Device ID
}

/**
 * Setup I2C slave
 */
static void setup_i2c_slave(void) {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

int main(void) {
    // Initialize LED first for immediate visual feedback
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Rapid blink to show we're starting
    for (int i = 0; i < 6; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }

    stdio_init_all();

    // Wait for USB serial (optional, for debugging)
    sleep_ms(2000);

    printf("RC Car I2C Slave Controller\n");
    printf("I2C Address: 0x%02X\n", I2C_SLAVE_ADDRESS);
    printf("SDA: GP%d, SCL: GP%d\n", I2C_SLAVE_SDA_PIN, I2C_SLAVE_SCL_PIN);

    init_registers();
    setup_i2c_slave();

    printf("Ready.\n");

    // LED off = idle, blinks on I2C activity
    gpio_put(LED_PIN, 0);

    // Track previous values to detect changes
    uint8_t prev_steering = context.regs[REG_STEERING];
    uint8_t prev_power = context.regs[REG_POWER];

    while (true) {
        // Check for register changes and apply them
        uint8_t steering = context.regs[REG_STEERING];
        uint8_t power = context.regs[REG_POWER];

        if (steering != prev_steering) {
            apply_steering(steering);
            prev_steering = steering;
        }

        if (power != prev_power) {
            apply_power(power);
            prev_power = power;
        }

        // Blink LED on I2C activity
        if (i2c_activity) {
            i2c_activity = false;
            gpio_put(LED_PIN, 1);
            sleep_ms(20);
            gpio_put(LED_PIN, 0);
        }

        sleep_ms(10);
    }

    return 0;
}
