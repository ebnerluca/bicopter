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
from rclpy.duration import Duration
from rclpy.time import Time

from bicopter_msgs.msg import ActuatorCommands
from std_srvs.srv import Trigger

from gpiozero import Servo, AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory

import numpy as np


class BicopterLowlevelController(Node):

    def __init__(self):
        super().__init__('bicopter_lowlevel_controller_node')

        self.get_logger().info("Reading parameters ...")
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('servo_left_gpio_pin', None),
                                    ('servo_right_gpio_pin', None),
                                    ('motor_left_gpio_pin', None),
                                    ('motor_right_gpio_pin', None),

                                    ('motor_pwm_min_signal', None),
                                    ('motor_pwm_max_signal', None),
                                    ('motor_power_limit', None),

                                    ('servo_pwm_min_pulse_width', None),
                                    ('servo_pwm_max_pulse_width', None),

                                    ('servo_limit_min_angle', None),
                                    ('servo_limit_max_angle', None),
                                    ('motor_arm_signal', None),

                                    ('servo_left_calibrated_min_angle', None),
                                    ('servo_left_calibrated_max_angle', None),
                                    ('servo_right_calibrated_min_angle', None),
                                    ('servo_right_calibrated_max_angle', None),

                                    ('actuator_commands_topic', None)
        ])

        # GPIO pins
        self.servo_left_gpio_pin = self.get_parameter('servo_left_gpio_pin').value
        self.servo_right_gpio_pin = self.get_parameter('servo_right_gpio_pin').value
        self.motor_left_gpio_pin = self.get_parameter('motor_left_gpio_pin').value
        self.motor_right_gpio_pin = self.get_parameter('motor_right_gpio_pin').value

        # Motor ESC protocol
        self.motor_pwm_min_signal = self.get_parameter('motor_pwm_min_signal').value
        self.motor_pwm_max_signal = self.get_parameter('motor_pwm_max_signal').value
        self.motor_arm_signal = self.get_parameter('motor_arm_signal').value

        # Servo PWM
        self.servo_pwm_min_pulse_width = self.get_parameter('servo_pwm_min_pulse_width').value
        self.servo_pwm_max_pulse_width = self.get_parameter('servo_pwm_max_pulse_width').value

        # Safety Limits
        self.servo_limit_min_angle = self.get_parameter('servo_limit_min_angle').value
        self.servo_limit_max_angle = self.get_parameter('servo_limit_max_angle').value
        self.motor_power_limit = self.get_parameter('motor_power_limit').value
        self.motor_limited_max_signal = self.motor_pwm_min_signal + self.motor_power_limit * \
                                        (self.motor_pwm_max_signal - self.motor_pwm_min_signal)

        self.motor_arm_value = ((self.motor_arm_signal - self.motor_pwm_min_signal) * 2.0) / (
                self.motor_limited_max_signal - self.motor_pwm_min_signal) - 1.0

        # calibration
        self.servo_left_calibrated_min_angle = self.get_parameter('servo_left_calibrated_min_angle').value
        self.servo_left_calibrated_max_angle = self.get_parameter('servo_left_calibrated_max_angle').value
        self.servo_right_calibrated_min_angle = self.get_parameter('servo_right_calibrated_min_angle').value
        self.servo_right_calibrated_max_angle = self.get_parameter('servo_right_calibrated_max_angle').value

        # topics
        self.actuator_commands_topic = self.get_parameter('actuator_commands_topic').value

        self.get_logger().info("Motor power limit: " + str(self.motor_power_limit))
        self.get_logger().info("Motor limited max signal: " + str(self.motor_limited_max_signal))
        self.get_logger().info("Motor arm value: " + str(self.motor_arm_value) + " [-1,1]")

        # servos, motors (motor ESC signal is same as for servos: pwm, 50Hz, 1-2ms duty cycle)
        self.get_logger().info("Initializing motors and servos ...")
        Device.pin_factory = PiGPIOFactory()  # reduced servo jitter
        self.servo_left = AngularServo(pin=self.servo_left_gpio_pin,
                                       initial_angle=None,
                                       min_angle=self.servo_left_calibrated_min_angle,
                                       max_angle=self.servo_left_calibrated_max_angle,
                                       min_pulse_width=self.servo_pwm_min_pulse_width,
                                       max_pulse_width=self.servo_pwm_max_pulse_width)
        self.servo_right = AngularServo(pin=self.servo_right_gpio_pin,
                                        initial_angle=None,
                                        min_angle=self.servo_right_calibrated_min_angle,
                                        max_angle=self.servo_right_calibrated_max_angle,
                                        min_pulse_width=self.servo_pwm_min_pulse_width,
                                        max_pulse_width=self.servo_pwm_max_pulse_width)
        self.motor_left = Servo(pin=self.motor_left_gpio_pin,
                                initial_value=None,
                                min_pulse_width=self.motor_pwm_min_signal,
                                max_pulse_width=self.motor_limited_max_signal)
        self.motor_right = Servo(pin=self.motor_right_gpio_pin,
                                 initial_value=None,
                                 min_pulse_width=self.motor_pwm_min_signal,
                                 max_pulse_width=self.motor_limited_max_signal)

        # motor controller
        self.motor_controller_timer = self.create_timer(1. / 50., self.apply_commands)  # apply new commands with 200Hz
        self.servo_left_command = 0.0  # measured in degrees
        self.servo_right_command = 0.0  # measured in degrees
        self.motor_left_command = -1.0  # measured from -1.0 (zero power) to 1.0 (max power)
        self.motor_right_command = -1.0
        self.command_last_stamp = self.get_clock().now()
        self.command_current_stamp = self.get_clock().now()

        # services
        self.is_armed = False
        self.arm_srv = self.create_service(Trigger, 'arm', self.arm_srv_callback)
        self.disarm_srv = self.create_service(Trigger, 'disarm', self.disarm_srv_callback)

        # publishers
        # self.publisher_ = self.create_publisher(String, 'topic', 10)

        # subscribers
        self.commands_sub = self.create_subscription(ActuatorCommands, self.actuator_commands_topic, self.update_commands_callback, 10)

    def apply_commands(self):

        if self.is_armed:

            # check stamp
            """duration = self.get_clock().now() - self.command_current_stamp
            if duration > Duration(seconds=1.0):
                self.get_logger().warn("command is too old!")
                self.disarm()"""

            self.servo_left.angle = self.servo_left_command
            self.servo_right.angle = self.servo_right_command
            self.motor_left.value = self.motor_left_command
            self.motor_right.value = self.motor_right_command

        """msg = String()
        msg.data = "hello"
        self.publisher_.publish(msg)"""

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
            self.get_logger().warn("Bicopter ARMED")
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
            self.disarm()
            response.success = True
            response.message = "Bicopter disarmed."

        return response

    def disarm(self):
        self.servo_left.detach()
        self.servo_right.detach()
        self.motor_left.detach()
        self.motor_right.detach()
        self.is_armed = False
        self.get_logger().warn("Bicopter disarmed")


    def update_commands_callback(self, commands):

        # clip commands to ensure boundaries
        self.servo_left_command = np.clip(a=commands.b1 * 180.0 / np.pi,
                                          a_min=self.servo_limit_min_angle,
                                          a_max=self.servo_limit_max_angle)
        self.servo_right_command = np.clip(a=commands.b2 * 180.0 / np.pi,
                                          a_min=self.servo_limit_min_angle,
                                          a_max=self.servo_limit_max_angle)
        self.motor_left_command = np.clip(a=commands.r1/1000.0 - 1.0, a_min=-1.0, a_max=1.0)
        self.motor_right_command = np.clip(a=commands.r2/1000.0 - 1.0, a_min=-1.0, a_max=1.0)
        self.command_current_stamp = Time.from_msg(commands.header.stamp)

    def on_shutdown(self):
        
        self.get_logger().info("Shutting down ...")
        self.disarm()


def main(args=None):
    rclpy.init(args=args)

    node = BicopterLowlevelController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.on_shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
