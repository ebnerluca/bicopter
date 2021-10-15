from gpiozero import Servo
import time

motor_left_gpio_pin = 5
motor_left_min_signal = 1. / 1000.
motor_left_max_signal = 1.5 / 1000.  # max 50% speed
motor_arm_signal = 1.15 / 1000.
motor_arm_value = ((motor_arm_signal - motor_left_min_signal) * 2.0) / (motor_left_max_signal - motor_left_min_signal) - 1.0  # find servo value between [-1,1] that corresponds to arm signal

print("Initializing servo with initial value: None ...")
time.sleep(2)
motor_left = Servo(pin=motor_left_gpio_pin,
                   initial_value=None,
                   min_pulse_width=motor_left_min_signal,
                   max_pulse_width=motor_left_max_signal)
print("Done.")

print("Waiting 5 sec before arming...")
time.sleep(5)
motor_left.value = motor_arm_value
print("ARMED.")




