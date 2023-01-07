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

from bicopter_msgs.msg import ActuatorCommands, SensorReadings
from std_srvs.srv import Trigger

import pybullet as p
import pybullet_data

import numpy as np

from ament_index_python import get_package_share_directory
from os.path import join


class BicopterLowlevelController(Node):

    def __init__(self):
        super().__init__('bicopter_lowlevel_controller_node')

        self.get_logger().info("Reading parameters ...")
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('motor_power_limit', None),

                                    ('servo_limit_min_angle', None),
                                    ('servo_limit_max_angle', None),

                                    ('actuator_commands_topic', None),
                                    ('imu_readings_topic', None),

                                    ('model.package', None),
                                    ('model.file', None),

                                    ('prop_c1', None)
        ])

        # Safety Limits
        self.servo_limit_min_angle = self.get_parameter('servo_limit_min_angle').value
        self.servo_limit_max_angle = self.get_parameter('servo_limit_max_angle').value
        self.motor_power_limit = self.get_parameter('motor_power_limit').value
        self.motor_arm_value = 0.0
        # topics
        self.actuator_commands_topic = self.get_parameter('actuator_commands_topic').value
        self.imu_readings_topic = self.get_parameter('imu_readings_topic').value
        # URDF
        model_package = get_package_share_directory(self.get_parameter('model.package').value)
        model_file = self.get_parameter('model.file').value
        self.urdf_path = join(model_package, model_file)
        # prop c1 constant
        self.prop_c1 = self.get_parameter('prop_c1').value

        # pybullet
        physics_client_id = p.connect(p.GUI)
        self.get_logger().info("PyBullet client connected with ID: " + str(physics_client_id))

        # Config
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(False)

        # Spawn ground plane
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.loadURDF("plane.urdf", basePosition=[0.0, 0.0, 0.0])
        self.get_logger().info("Loading Bicopter URDF from: " + self.urdf_path)
        self.bicopter_id = p.loadURDF(self.urdf_path)

        # pybullet controller
        self.pybullet_sim_timer = self.create_timer(1. / 240., self.pybullet_step)
        self.servo_left_command = 0.0  # measured in degrees
        self.servo_right_command = 0.0  # measured in degrees
        self.motor_left_command = 0.0
        self.motor_right_command = 0.0
        self.command_current_stamp = self.get_clock().now()

        # services
        self.is_armed = False
        self.arm_srv = self.create_service(Trigger, 'arm', self.arm_srv_callback)
        self.disarm_srv = self.create_service(Trigger, 'disarm', self.disarm_srv_callback)

        # imu publisher
        self.imu_publisher = self.create_publisher(SensorReadings, self.imu_readings_topic, 5)
        self.imu_readings = SensorReadings()

        # subscribers
        self.commands_sub = self.create_subscription(ActuatorCommands, self.actuator_commands_topic, self.update_commands_callback, 10)

        self.get_logger().info("Running.")

    def pybullet_step(self):
        """Servo commands should be given in degrees, Motor commands should be given in interval [-1,1]."""

        # simulation step
        p.stepSimulation()

        if self.is_armed:

            # check stamp
            duration = self.get_clock().now() - self.command_current_stamp
            if duration > Duration(seconds=1.0):
                self.get_logger().warn("Command is too old! Disarming ...")
                self.disarm()
            else:
                # set servos
                p.setJointMotorControlArray(bodyUniqueId=self.bicopter_id, jointIndices=[0, 1],
                                            controlMode=p.POSITION_CONTROL,
                                            targetPositions=[self.servo_left_command,
                                                             self.servo_right_command],
                                            targetVelocities=[0.0, 0.0],
                                            positionGains=[1.0, 1.0],
                                            velocityGains=[0.5, 0.5])

                # apply thrust
                p.applyExternalForce(objectUniqueId=self.bicopter_id, linkIndex=0,
                                     forceObj=[0.0, 0.0, self.prop_c1 * self.motor_left_command * self.motor_left_command],
                                     posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)
                p.applyExternalForce(objectUniqueId=self.bicopter_id, linkIndex=1,
                                     forceObj=[0.0, 0.0, self.prop_c1 * self.motor_right_command * self.motor_right_command],
                                     posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)

        # get readings
        pos, orientation = p.getBasePositionAndOrientation(self.bicopter_id)
        # rpy = np.around(np.array(p.getEulerFromQuaternion(orientation)), 2)
        self.imu_readings.z = pos[2]
        self.imu_readings.q_x = orientation[0]
        self.imu_readings.q_y = orientation[1]
        self.imu_readings.q_z = orientation[2]
        self.imu_readings.q_w = orientation[3]

        vel, angular_vel = p.getBaseVelocity(self.bicopter_id)
        self.imu_readings.v_z = vel[2]
        rotmat = np.array(p.getMatrixFromQuaternion(orientation)).reshape((3, 3), order="F")
        drpy = rotmat.dot(np.array(angular_vel))

        self.imu_readings.w_x = drpy[0]
        self.imu_readings.w_y = drpy[1]
        self.imu_readings.w_z = drpy[2]

        # publish readings
        self.imu_publisher.publish(self.imu_readings)

    def arm_srv_callback(self, request, response):

        if self.is_armed:
            response.success = True
            response.message = "Bicopter already armed."
            self.get_logger().warn("Arming not possible, Bicopter already armed.")
        # check if motor commands too high
        # elif (self.motor_left_command > self.motor_arm_value) or (self.motor_right_command > self.motor_arm_value):
        #     response.success = False
        #     response.message = "Arming not possible, motor commands are too high."
        #     self.get_logger().warn("Arming not possible, motor commands are too high.")
        else:
            self.is_armed = True
            self.get_logger().warn("Bicopter ARMED")
            self.motor_left_command = self.motor_arm_value
            self.motor_right_command = self.motor_arm_value
            # self.servo_left_command = 0.0
            # self.servo_right_command = 0.0
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
        self.is_armed = False
        self.get_logger().warn("Bicopter disarmed")

    def update_commands_callback(self, commands):

        # clip commands to ensure boundaries
        self.servo_left_command = np.clip(a=commands.b1,
                                          a_min=self.servo_limit_min_angle,
                                          a_max=self.servo_limit_max_angle)
        self.servo_right_command = np.clip(a=commands.b2,
                                          a_min=self.servo_limit_min_angle,
                                          a_max=self.servo_limit_max_angle)
        self.motor_left_command = np.clip(a=commands.r1, a_min=0.0, a_max=self.motor_power_limit)
        self.motor_right_command = np.clip(a=commands.r2, a_min=0.0, a_max=self.motor_power_limit)
        # self.command_current_stamp = Time.from_msg(commands.header.stamp)
        self.command_current_stamp = self.get_clock().now()

    def on_shutdown(self):

        self.get_logger().info("Shutting down ...")
        self.disarm()
        p.disconnect()
        self.get_logger().info("Pybullet client disconnected.")


def main(args=None):
    rclpy.init(args=args)

    node = BicopterLowlevelController()

    # adding shutdown method to context shutdown
    rclpy.get_default_context().on_shutdown(node.on_shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
