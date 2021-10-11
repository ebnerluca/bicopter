import RPi.GPIO as GPIO
import time

gpio_pin = 17

GPIO.setmode(GPIO.BCM)  # numbering according to GPIO pins, not Raspberry pins
GPIO.setup(gpio_pin, GPIO.OUT) 

# blink an led 
while True:

	GPIO.output(17, GPIO.HIGH)
	time.sleep(1.0)

	GPIO.output(17, GPIO.LOW)
	time.sleep(1.0)

