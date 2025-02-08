from gpiozero import Servo
import sleep

servo = Servo(0)

servo.value = 0.5
sleep(1)
servo.value = 0
sleep(1)
servo.value = 1
sleep(1)