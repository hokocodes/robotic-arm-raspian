from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(1000, 2000)
kit.servo[1].set_pulse_width_range(1000, 2000)

kit.servo[2].set_pulse_width_range(1000, 2000)
kit.servo[3].set_pulse_width_range(1000, 2000)
kit.servo[4].set_pulse_width_range(1000, 2000)
kit.servo[5].set_pulse_width_range(1000, 2000)
kit.servo[1].angle = 180
print("Servo 1, Angle = 180")
time.sleep(5)

while True:
    r = input()
    if r == "0":
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
    elif r == "1":
        kit.servo[1].angle = 180
        print("Servo 1, Angle = 180")
        time.sleep(5)
        kit.servo[1].angle = 0
        print("Servo 1, Angle = 0")
        time.sleep(5)
    elif r == "2":
        kit.servo[2].angle = 180
        print("Servo 2, Angle = 180")
        time.sleep(5)
        kit.servo[2].angle = 0
        print("Servo 2, Angle = 0")
        time.sleep(5)
    elif r == "3":
        kit.servo[3].angle = 180
        print("Servo 3, Angle = 180")
        time.sleep(5)
        kit.servo[3].angle = 0
        print("Servo 3, Angle = 0")
        time.sleep(5)
    elif r == "4":
        kit.servo[4].angle = 180
        print("Servo 4, Angle = 180")
        time.sleep(5)
        kit.servo[4].angle = 0
        print("Servo 4, Angle = 0")
        time.sleep(5)
    elif r == "5":
        kit.servo[5].angle = 180
        print("Servo 5, Angle = 180")
        time.sleep(5)
        kit.servo[5].angle = 0
        print("Servo 5, Angle = 0")
        time.sleep(5)
    elif r == "6":
        kit.servo[6].angle = 180
        print("Servo 6, Angle = 180")
        time.sleep(5)
        kit.servo[6].angle = 0
        print("Servo 6, Angle = 0")
        time.sleep(5)
    elif r == "7":
        kit.servo[7].angle = 180
        print("Servo 7, Angle = 180")
        time.sleep(5)
        kit.servo[7].angle = 0
        print("Servo 7, Angle = 0")
        time.sleep(5)
    elif r == "8":
        kit.servo[8].angle = 180
        print("Servo 8, Angle = 180")
        time.sleep(5)
        kit.servo[8].angle = 0
        print("Servo 8, Angle = 0")
        time.sleep(5)
    elif r == "9":
        kit.servo[9].angle = 180
        print("Servo 9, Angle = 180")
        time.sleep(5)
        kit.servo[9].angle = 0
        print("Servo 9, Angle = 0")
        time.sleep(5)
    elif r == "10":
        kit.servo[10].angle = 180
        print("Servo 10, Angle = 180")
        time.sleep(5)
        kit.servo[10].angle = 0
        print("Servo 10, Angle = 0")
        time.sleep(5)
    elif r == "11":
        kit.servo[11].angle = 180
        print("Servo 11, Angle = 180")
        time.sleep(5)
        kit.servo[11].angle = 0
        print("Servo 11, Angle = 0")
        time.sleep(5)
