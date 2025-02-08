from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()
servo = Servo(0, pin_factory=factory)