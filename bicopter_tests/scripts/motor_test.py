from gpiozero import Servo, Device
from gpiozero.pins.pigpio import PiGPIOFactory
import time

motor_left_gpio_pin = 17
motor_left_min_signal = 0.001
motor_left_max_signal = 0.0015  # max 50% speed
motor_arm_signal = 0.0011
initial_val = -1


motor_arm_value = ((motor_arm_signal - motor_left_min_signal) * 2.0) / (motor_left_max_signal - motor_left_min_signal) - 1.0  # find servo value between [-1,1] that corresponds to arm signal
print("motor arm value: " + str(motor_arm_value))
time.sleep(2)

print("Initializing servo with initial value: " + str(initial_val))

Device.pin_factory = PiGPIOFactory()  # reduced servo jitter

motor_left = Servo(pin=motor_left_gpio_pin,
                   initial_value=None,
                   min_pulse_width=motor_left_min_signal,
                   max_pulse_width=motor_left_max_signal)

print("Current pulse width: " + str(motor_left.pulse_width))

key=True
while(key):
    try:
        print(str(motor_left.pulse_width))
    except KeyboardInterrupt:
        key=False

motor_left.value = -1
print("Current pulse width: " + str(motor_left.pulse_width))
key=True
while(key):
    try:
        print(str(motor_left.pulse_width))
    except KeyboardInterrupt:
        key=False

motor_left.value = motor_arm_value
print("Current pulse width: " + str(motor_left.pulse_width))
key=True
while(key):
    try:
        print(str(motor_left.pulse_width))
    except KeyboardInterrupt:
        key=False


print("Cleanup ...")
motor_left.value=None
