from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(1000, 2000)

kit.servo[0].angle = 180
time.sleep(1)
kit.servo[0].angle = 0
time.sleep(1)

