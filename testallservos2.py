from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(1000, 2000)

kit.servo[0].angle = 180
print("Servo 0, Angle = 180")
time.sleep(5)
kit.servo[0].set_pulse_width_range(1000, 2000)
kit.servo[0].angle = 140
print("Servo 0, Angle = 140")
time.sleep(5)
kit.servo[0].angle = 120
print("Servo 0, Angle = 120")
time.sleep(5)

kit.servo[0].angle = 100
print("Servo 0, Angle = 100")
time.sleep(5)

kit.servo[0].angle = 80
print("Servo 0, Angle = 80")
time.sleep(5)

kit.servo[0].angle = 60
print("Servo 0, Angle = 60")
time.sleep(5)

kit.servo[0].angle = 40
print("Servo 0, Angle = 40")
time.sleep(5)

kit.servo[0].angle = 20
print("Servo 0, Angle = 20")
time.sleep(5)

kit.servo[0].angle = 0
print("Servo 0, Angle = 0")
time.sleep(5)

