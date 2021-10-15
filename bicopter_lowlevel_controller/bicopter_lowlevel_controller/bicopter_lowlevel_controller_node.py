# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger

# import RPi.GPIO as GPIO
from gpiozero import Servo, AngularServo


class BicopterLowlevelController(Node):

    def __init__(self):
        super().__init__('bicopter_lowlevel_controller')

        # TODO read from config
        self.i = 0
        # pins
        self.servo_left_gpio_pin = 17
        self.servo_right_gpio_pin = 27
        self.motor_left_gpio_pin = 5
        self.motor_right_gpio_pin = 6

        # safety limits
        self.servo_left_min_angle = -45.0
        self.servo_left_max_angle = 45.0  # 45deg
        self.servo_right_min_angle = -45.0
        self.servo_right_max_angle = 45.0
        self.motor_left_min_signal = 1. / 1000.
        self.motor_left_max_signal = 1.5 / 1000.  # max 50% speed
        self.motor_right_min_signal = 1. / 1000.
        self.motor_right_max_signal = 1.5 / 1000.  # max 50% speed
        self.motor_arm_signal = 1.15 / 1000.
        self.motor_arm_value = ((self.motor_arm_signal - self.motor_left_min_signal) * 2.0) / (self.motor_left_max_signal - self.motor_left_min_signal) - 1.0

        # calibration
        self.servo_left_calibrated_min_angle = -90.0  # deg, minimum possible angle when servo receives min() command
        self.servo_left_calibrated_max_angle = 90.0  # deg, maximum possible angle when servo receives max() command
        self.servo_right_calibrated_min_angle = -90.0  # deg, minimum possible angle when servo receives min() command
        self.servo_right_calibrated_max_angle = 90.0  # deg, maximum possible angle when servo receives max() command

        # servos, motors (motor ESC signal is same as for servos: pwm, 50Hz, 1-2ms duty)
        self.servo_left = AngularServo(pin=self.servo_left_gpio_pin,
                                       initial_angle=None,
                                       min_angle=self.servo_left_min_angle,
                                       max_angle=self.servo_left_max_angle)
        self.servo_right = AngularServo(pin=self.servo_right_gpio_pin,
                                        initial_angle=None,
                                        min_angle=self.servo_right_min_angle,
                                        max_angle=self.servo_right_max_angle)
        self.motor_left = Servo(pin=self.motor_left_gpio_pin,
                                initial_value=None,
                                min_pulse_width=self.motor_left_min_signal,
                                max_pulse_width=self.motor_left_max_signal)
        self.motor_right = Servo(pin=self.motor_right_gpio_pin,
                                 initial_value=None,
                                 min_pulse_width=self.motor_right_min_signal,
                                 max_pulse_width=self.motor_right_max_signal)

        # motor controller
        self.motor_controller_timer = self.create_timer(1./50., self.apply_commands)  # apply new commands with 200Hz
        self.servo_left_command = 0.0  # measured in degrees
        self.servo_right_command = 0.0  # measured in degrees
        self.motor_left_command = -1.0  # measured from -1.0 (zero power) to 1.0 (max power)
        self.motor_right_command = -1.0

        # services
        self.is_armed = False
        self.arm_srv = self.create_service(Trigger, 'arm', self.arm_srv_callback)
        self.disarm_srv = self.create_service(Trigger, 'disarm', self.disarm_srv_callback)

        # publishers
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # subscribers
        self.commands_sub = self.create_subscription(String, 'commands', self.update_commands_callback, 10)

    def apply_commands(self):

        # clip commands to ensure boundaries
        # TODO

        if self.is_armed:
            self.servo_left.angle = self.servo_left_command
            self.servo_right.angle = self.servo_right_command
            self.motor_left.value = self.motor_left_command
            self.motor_right.value = self.motor_right_command

        self.i = self.i + 1
        msg = String()
        msg.data = str(self.i)
        print(str(self.i))
        self.publisher_.publish(msg)

    def arm_srv_callback(self, request, response):

        if self.is_armed:
            response.success = True
            response.message = "Bicopter already armed."
            self.get_logger().warn("Arming not possible, Bicopter already armed.")
        elif (self.motor_left_command > self.motor_arm_signal) or (self.motor_right_command > self.motor_arm_signal):
            response.success = False
            response.message = "Arming not possible, motor commands are too high."
            self.get_logger().warn("Arming not possible, motor commands are too high.")
        else:
            self.is_armed = True
            self.motor_left_command = self.motor_arm_value
            self.motor_right_command = self.motor_arm_value
            self.servo_left_command = 0.0
            self.servo_right_command = 0.0
            response.success = True
            response.message = "Bicopter armed."

        return response

    def disarm_srv_callback(self, request, response):

        if not self.is_armed:
            response.success = True
            response.message = "Bicopter already disarmed."
            self.get_logger().warn("Disarming not possible, Bicopter already disarmed.")
        else:
            self.servo_left.detach()
            self.servo_right.detach()
            self.motor_left.detach()
            self.motor_right.detach()
            self.is_armed = False
            response.success = True
            response.message = "Bicopter disarmed."

        return response

    def update_commands_callback(self, commands):

        pass


def main(args=None):
    rclpy.init(args=args)

    node = BicopterLowlevelController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
