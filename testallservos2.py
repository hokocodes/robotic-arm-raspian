from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(1000, 2000)

kit.servo[0].angle = 180
print("Servo 0, Angle = 180")
time.sleep(5)
kit.servo[0].angle = 0
print("Servo 0, Angle = 0")
time.sleep(1)

