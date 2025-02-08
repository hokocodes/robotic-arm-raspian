# SPDX-FileCopyrightText: 2018 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time
import board
import pwmio
from adafruit_motor import servo

# This line creates a PWMOut object on Pin D5 with a 50% duty cycle and a frequency of 50Hz.
pwm = pwmio.PWMOut(board.D0, duty_cycle=2 ** 15, frequency=50)

# Create a servo object.
servo = servo.Servo(pwm)

while True:
    for angle in range(0, 180, 5):  # 0 - 180 degrees, 5 degrees at a time.
        servo.angle = angle
        print("Sleeping for 0.05 seconds")
        time.sleep(0.15)
    for angle in range(180, 0, -5): # 180 - 0 degrees, 5 degrees at a time.
        servo.angle = angle
        time.sleep(0.15)
