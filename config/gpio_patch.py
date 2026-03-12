#!/usr/bin/env python3
"""
Klipper Linux-MCU Simulation Patch for Docker/WSL2

Patches all hardware-access source files so that klipper_mcu
can run without /dev/gpiochip, /sys/bus/iio, /dev/spidev, etc.

All function signatures match src/linux/gpio.h exactly.

Usage (inside container):
    docker exec klipper python3 /home/klippy/printer_data/config/gpio_patch.py
"""

import os

BASE = '/home/klippy/klipper/src/linux'

patches = {}

# ── gpio.c ──────────────────────────────────────────────────────
patches['gpio.c'] = r'''// Simulated GPIO for Docker/WSL2 (no real /dev/gpiochip)
#include <string.h>
#include "command.h"
#include "gpio.h"
#include "internal.h"
#include "sched.h"

DECL_ENUMERATION_RANGE("pin", "gpio0", GPIO(0, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip0/gpio0", GPIO(0, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip1/gpio0", GPIO(1, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip2/gpio0", GPIO(2, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip3/gpio0", GPIO(3, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip4/gpio0", GPIO(4, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip5/gpio0", GPIO(5, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip6/gpio0", GPIO(6, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip7/gpio0", GPIO(7, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip8/gpio0", GPIO(8, 0), MAX_GPIO_LINES);

struct gpio_line { int chipid; int offset; int fd; int state; };
static struct gpio_line gpio_lines[9 * MAX_GPIO_LINES];

struct gpio_out gpio_out_setup(uint32_t pin, uint8_t val) {
    if (pin >= ARRAY_SIZE(gpio_lines)) shutdown("Not an output pin");
    struct gpio_line *line = &gpio_lines[pin];
    line->offset = GPIO2PIN(pin);
    line->chipid = GPIO2PORT(pin);
    line->state = !!val;
    line->fd = 1;
    struct gpio_out g = { .line = line };
    return g;
}
void gpio_out_reset(struct gpio_out g, uint8_t val) { g.line->state = !!val; }
void gpio_out_write(struct gpio_out g, uint8_t val) { g.line->state = !!val; }
void gpio_out_toggle_noirq(struct gpio_out g) { gpio_out_write(g, !g.line->state); }
void gpio_out_toggle(struct gpio_out g) { gpio_out_toggle_noirq(g); }

struct gpio_in gpio_in_setup(uint32_t pin, int8_t pull_up) {
    if (pin >= ARRAY_SIZE(gpio_lines)) shutdown("Not an input pin");
    struct gpio_line *line = &gpio_lines[pin];
    line->offset = GPIO2PIN(pin);
    line->chipid = GPIO2PORT(pin);
    line->fd = 1;
    struct gpio_in g = { .line = line };
    return g;
}
void gpio_in_reset(struct gpio_in g, int8_t pull_up) {}
uint8_t gpio_in_read(struct gpio_in g) { return 0; }
'''

# ── analog.c (ADC) ─────────────────────────────────────────────
# gpio_adc: struct gpio_adc { int fd; };
patches['analog.c'] = r'''// Simulated ADC for Docker/WSL2
#include "command.h"
#include "gpio.h"
#include "internal.h"
#include "sched.h"

DECL_CONSTANT("ADC_MAX", 4095);

#define ANALOG_START (1<<12)
DECL_ENUMERATION_RANGE("pin", "analog0", ANALOG_START, 8);

struct gpio_adc gpio_adc_setup(uint32_t pin) {
    return (struct gpio_adc){ .fd = 1 };
}
uint32_t gpio_adc_sample(struct gpio_adc g) { return 0; }
uint16_t gpio_adc_read(struct gpio_adc g) { return 2048; }
void gpio_adc_cancel_sample(struct gpio_adc g) {}
'''

# ── spidev.c ────────────────────────────────────────────────────
# spi_config: struct spi_config { int fd; int rate; };
patches['spidev.c'] = r'''// Simulated SPI for Docker/WSL2
#include "command.h"
#include "gpio.h"
#include "internal.h"
#include "sched.h"

struct spi_config spi_setup(uint32_t bus, uint8_t mode, uint32_t rate) {
    return (struct spi_config){ .fd = 1, .rate = (int)rate };
}
void spi_prepare(struct spi_config config) {}
void spi_transfer(struct spi_config config, uint8_t receive_data,
                  uint8_t len, uint8_t *data) {}
'''

# ── i2c.c ───────────────────────────────────────────────────────
# i2c_config: struct i2c_config { int fd; uint8_t addr; };
# Returns int, not void!
patches['i2c.c'] = r'''// Simulated I2C for Docker/WSL2
#include "command.h"
#include "gpio.h"
#include "internal.h"
#include "sched.h"

struct i2c_config i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr) {
    return (struct i2c_config){ .fd = 1, .addr = addr };
}
int i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write) {
    return 0;
}
int i2c_read(struct i2c_config config, uint8_t reg_len, uint8_t *reg,
             uint8_t read_len, uint8_t *read) {
    return 0;
}
'''

# ── hard_pwm.c ──────────────────────────────────────────────────
# gpio_pwm: struct gpio_pwm { int duty_fd, enable_fd; uint32_t period; };
# Signature: gpio_pwm_setup(uint32_t pin, uint32_t cycle_time, uint16_t val)
#            gpio_pwm_write(struct gpio_pwm g, uint16_t val)
patches['hard_pwm.c'] = r'''// Simulated PWM for Docker/WSL2
#include "command.h"
#include "gpio.h"
#include "internal.h"
#include "sched.h"

DECL_CONSTANT("PWM_MAX", 32768);

struct gpio_pwm gpio_pwm_setup(uint32_t pin, uint32_t cycle_time, uint16_t val) {
    return (struct gpio_pwm){ .duty_fd = 1, .enable_fd = 1, .period = cycle_time };
}
void gpio_pwm_write(struct gpio_pwm g, uint16_t val) {}
'''

# ── pca9685.c ───────────────────────────────────────────────────
# Same struct gpio_pwm, uses pca9685_setup / pca9685_write
patches['pca9685.c'] = r'''// Simulated PCA9685 for Docker/WSL2
#include "command.h"
#include "gpio.h"
#include "internal.h"
#include "sched.h"

DECL_CONSTANT("PCA9685_MAX", 4096);

struct gpio_pwm pca9685_setup(uint8_t bus, uint8_t addr, uint8_t channel,
                              uint16_t cycle_ticks, uint16_t val) {
    return (struct gpio_pwm){ .duty_fd = 1, .enable_fd = 1, .period = cycle_ticks };
}
void pca9685_write(struct gpio_pwm g, uint16_t val) {}
'''

# ── sensor_ds18b20.c ───────────────────────────────────────────
patches['sensor_ds18b20.c'] = r'''// Simulated DS18B20 for Docker/WSL2
// No w1 bus in Docker — placeholder only
'''

# ── Apply all patches ──────────────────────────────────────────

patched = 0
for filename, content in patches.items():
    path = os.path.join(BASE, filename)
    if os.path.exists(path):
        with open(path, 'w') as f:
            f.write(content)
        print(f'  OK: {filename} ({len(content)} bytes)')
        patched += 1
    else:
        print(f'  SKIP: {filename} (not found)')

# ── Targeted patch: stepper.c (timing tolerance for Docker/WSL2) ──
# The "Stepper too far in past" shutdown is hardcoded with a ~1ms tolerance.
# In Docker/WSL2 scheduling jitter easily exceeds this. Since this is a
# simulation (the bridge handles real timing independently), we replace
# the shutdown with a no-op so Klipper keeps running instead of crashing.
STEPPER_C = '/home/klippy/klipper/src/stepper.c'
if os.path.exists(STEPPER_C):
    with open(STEPPER_C, 'r') as f:
        content = f.read()

    original = content

    # Klipper versions use slightly different formulations — patch all known variants
    replacements = [
        # Variant A (common in recent Klipper)
        ('shutdown("Stepper too far in past");',
         '/* shutdown("Stepper too far in past"); */ /* Docker/WSL2 sim patch */'),
        # Variant B (older formulation)
        ('shutdown("Stepper too far in past")',
         '/* shutdown("Stepper too far in past") */ (void)0'),
    ]

    for old, new in replacements:
        content = content.replace(old, new)

    if content != original:
        with open(STEPPER_C, 'w') as f:
            f.write(content)
        print(f'  OK: stepper.c (timing shutdown disabled)')
    else:
        print(f'  WARN: stepper.c found but shutdown string not matched — check Klipper version')
        # Fallback: print the lines containing 'too far' so the user can inspect
        for i, line in enumerate(original.splitlines(), 1):
            if 'too far' in line or 'far in past' in line.lower():
                print(f'       Line {i}: {line.strip()}')
else:
    print(f'  SKIP: stepper.c (not found at {STEPPER_C})')

print(f'\nPatched {patched}/{len(patches)} files.')
print('Now run: make clean && make -j4')
