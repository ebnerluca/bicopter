from gpiozero.pins.pigpio import PiGPIOFactory
from  gpiozero import Servo, Device, AngularServo
import time

Device.pin_factory = PiGPIOFactory()

servo_gpio_pin = 17
servo = AngularServo(servo_gpio_pin, min_pulse_width=0.5/1000.0, max_pulse_width=2.5/1000.0)

while True:
	servo.min()
	time.sleep(2)
	servo.max()
	time.sleep(2)
	servo.mid()
	time.sleep(2)


