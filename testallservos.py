# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=8)


# for i in range(6):
#     x = input("Enter servo number to set ({}), or anything else to exit: ")
#     if x == str(i):
#         kit.continuous_servo[i].throttle = 1
#         print("Servo {}, Throttle = 1".format(i))
#         time.sleep(1)
#         kit.continuous_servo[i].throttle = -1
#         print("Servo {}, Throttle = -1".format(i))
#         time.sleep(1)
r = input()
while True:

    servo_number = r  # Change this to test other servos (0-5)
    
    if servo_number == "0":
        kit.continuous_servo[0].throttle = 1
        print("Servo 0, Throttle = 1")
        time.sleep(1)
        kit.continuous_servo[0].throttle = -1
        print("Servo 0, Throttle = -1")
        time.sleep(1)
        servo_number = input()
    elif servo_number == "1":
        kit.continuous_servo[1].throttle = 1
        print("Servo 1, Throttle = 1")
        time.sleep(1)
        kit.continuous_servo[1].throttle = -1
        print("Servo 1, Throttle = -1")
        time.sleep(1)
        servo_number = input()
    elif servo_number == "2":
        kit.continuous_servo[2].throttle = 1
        print("Servo 2, Throttle = 1")
        time.sleep(1)
        kit.continuous_servo[2].throttle = -1
        print("Servo 2, Throttle = -1")
        time.sleep(1)
        servo_number = input()
    elif servo_number == "3":
        kit.continuous_servo[3].throttle = 1
        print("Servo 3, Throttle = 1")
        time.sleep(1)
        kit.continuous_servo[3].throttle = -1
        print("Servo 3, Throttle = -1")
        time.sleep(1)
        servo_number = input()
    elif servo_number == "4":
        kit.continuous_servo[4].throttle = 1
        print("Servo 4, Throttle = 1")
        time.sleep(1)
        kit.continuous_servo[4].throttle = -1
        print("Servo 4, Throttle = -1")
        time.sleep(1)
        servo_number = input()
    elif servo_number == "5":
        kit.continuous_servo[5].throttle = 1
        print("Servo 5, Throttle = 1")
        time.sleep(1)
        kit.continuous_servo[5].throttle = -1
        print("Servo 5, Throttle = -1")
        time.sleep(1)
        servo_number = input()

