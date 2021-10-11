import RPi.GPIO as GPIO
from  gpiozero import Servo
import time

servo_gpio_pin = 17
servo = Servo(servo_gpio_pin)

while True:
	servo.value = -1
	time.sleep(2)
	servo.value = 0
	time.sleep(2)
	servo.value = 1
	time.sleep(2)


