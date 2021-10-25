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
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class BicopterAnalytics(Node):

    def __init__(self):
        super().__init__('bicopter_analytics_node')

        self.get_logger().info("Reading parameters ...")
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('imu_readings_topic', None),
                                ])

        # read parameters
        self.imu_readings_topic = self.get_parameter('imu_readings_topic').value

        # publishers
        self.broadcaster = TransformBroadcaster(self)

        self.subscriber = self.create_subscription(SensorReadings, self.imu_readings_topic, self.imu_readings_callback, 1)

        self.get_logger().info("Running.")

    def imu_readings_callback(self, readings):

        transform = TransformStamped()
        transform.header.stamp = readings.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "base"

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = readings.q_x
        transform.transform.rotation.y = readings.q_y
        transform.transform.rotation.z = readings.q_z
        transform.transform.rotation.w = readings.q_w

        self.broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)

    node = BicopterAnalytics()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
