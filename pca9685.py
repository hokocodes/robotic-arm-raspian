# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Outputs a 50% duty cycle PWM single on the 0th channel.
# Connect an LED and resistor in series to the pin
# to visualize duty cycle changes and its impact on brightness.

import board
from adafruit_pca9685 import PCA9685
import time

# Create the I2C bus interface.
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = busio.I2C(board.GP1, board.GP0)    # Pi Pico RP2040

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)

# Set the PWM frequency to 60hz.
pca.frequency = 60

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
pca.channels[0].duty_cycle = 0x7FFF
time.sleep(1)  # Wait for 1 second

# To make a servo rotate, you need to set the duty cycle to appropriate values for your servo.
# Typically, 1ms pulse width corresponds to 0 degrees, and 2ms pulse width corresponds to 180 degrees.
# For a 60Hz frequency, 1ms pulse width is approximately 2730 (0x0AAA) and 2ms pulse width is approximately 5460 (0x1554).

# Continuously rotate the servo back and forth between 0 and 180 degrees

pca.channels[0].duty_cycle = 0xffff
time.sleep(1)  # Wait for 1 second

# Rotate to 180 degrees
pca.channels[0].duty_cycle = 0x7FFF
time.sleep(1)  # Wait for 1 second

