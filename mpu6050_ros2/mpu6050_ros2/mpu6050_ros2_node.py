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

from bicopter_msgs.msg import SensorReadings

import smbus2
import numpy as np
from tf_transformations import quaternion_from_euler


class MPU6050(Node):

    def __init__(self):
        super().__init__('mpu6050_node')

        self.get_logger().info("Reading parameters ...")
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('i2c_adress', None),
                                    ('reg_powermgmt', None),
                                    ('reg_gyro_x', None),
                                    ('reg_gyro_y', None),
                                    ('reg_gyro_z', None),
                                    ('reg_accel_x', None),
                                    ('reg_accel_y', None),
                                    ('reg_accel_z', None),
                                    ('gyro_scale_factor', None),
                                    ('accel_scale_factor', None),
                                    ('frequency', None),
                                    ('imu_readings_topic', None)
        ])

        # read parameters
        self.i2c_adress = self.get_parameter('i2c_adress').value
        self.reg_powermgmt = self.get_parameter('reg_powermgmt').value
        self.reg_gyro_x = self.get_parameter('reg_gyro_x').value
        self.reg_gyro_y = self.get_parameter('reg_gyro_y').value
        self.reg_gyro_z = self.get_parameter('reg_gyro_z').value
        self.reg_accel_x = self.get_parameter('reg_accel_x').value
        self.reg_accel_y = self.get_parameter('reg_accel_y').value
        self.reg_accel_z = self.get_parameter('reg_accel_z').value
        self.gyro_scale_factor = self.get_parameter('gyro_scale_factor').value
        self.accel_scale_factor = self.get_parameter('accel_scale_factor').value
        self.frequency = self.get_parameter('frequency').value
        self.imu_readings_topic = self.get_parameter('imu_readings_topic').value

        self.roll = 0.0  # TODO initialize from gyro
        self.pitch = 0.0
        self.readings_msg = SensorReadings()

        self.get_logger().info("Waking up MPU6050 ...")
        self.bus = smbus2.SMBus(1)
        self.bus.write_byte_data(self.i2c_adress, self.reg_powermgmt, 0)  # wake up

        # timer
        self.readings_timer = self.create_timer(1. / self.frequency, self.update_readings)

        # publishers
        self.publisher = self.create_publisher(SensorReadings, self.imu_readings_topic, 10)

        self.get_logger().info("Running.")


    def update_readings(self):

        # get gyro readings
        gyro_x_rate = (self.read_word_2c(self.reg_gyro_x) / self.gyro_scale_factor) * np.pi / 180.0  # gyro roll rate
        gyro_y_rate = (self.read_word_2c(self.reg_gyro_y) / self.gyro_scale_factor) * np.pi / 180.0  # gyro pitch rate
        gyro_z_rate = self.read_word_2c(self.reg_gyro_z) / self.gyro_scale_factor * np.pi / 180.0  # gyro yaw rate

        # get accelerometer readings
        accel_x = self.read_word_2c(self.reg_accel_x) / self.accel_scale_factor
        accel_y = self.read_word_2c(self.reg_accel_y) / self.accel_scale_factor
        accel_z = self.read_word_2c(self.reg_accel_z) / self.accel_scale_factor

        # calculate roll, pitch angle from accelerometer data
        accel_pitch = np.arctan2(accel_x, np.sqrt(accel_y*accel_y + accel_z * accel_z))  # * 180.0 / np.pi
        accel_roll = np.arctan2(accel_y, np.sqrt(accel_x*accel_x + accel_z * accel_z))  # * 180.0 / np.pi

        self.roll = 0.98 * (self.roll + (1.0 / self.frequency) * gyro_x_rate) + 0.02 * accel_roll
        self.pitch = 0.98 * (self.pitch + (1.0 / self.frequency) * gyro_y_rate) + 0.02 * accel_pitch

        q = quaternion_from_euler(self.roll, self.pitch, 0.0)

        # update message
        self.readings_msg.q_x = q[0]
        self.readings_msg.q_y = q[1]
        self.readings_msg.q_z = q[2]
        self.readings_msg.q_w = q[3]
        self.readings_msg.w_x = gyro_x_rate
        self.readings_msg.w_y = gyro_y_rate
        self.readings_msg.w_z = gyro_z_rate
        self.readings_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.readings_msg)

    def read_word_2c(self, reg):
        """Reads word from a register reg which is given as 16 bit two complement."""
        high = self.bus.read_byte_data(self.i2c_adress, reg)
        low = self.bus.read_byte_data(self.i2c_adress, reg+1)
        value = (high << 8) + low

        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value


def main(args=None):
    rclpy.init(args=args)

    node = MPU6050()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
