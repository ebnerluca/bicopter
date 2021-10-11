import RPi.GPIO as GPIO
import time

pin = 17  # GPIO 17
Hz = 50  # PWM period [Hz]

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

p = GPIO.PWM(pin, Hz)
p.start(50.0)

while True:
	p.ChangeDutyCycle(25)
	time.sleep(0.5)
	p.ChangeDutyCycle(50)
	time.sleep(0.5)
	p.ChangeDutyCycle(75)
	time.sleep(0.5)
	p.ChangeDutyCycle(50)
	time.sleep(0.5)
