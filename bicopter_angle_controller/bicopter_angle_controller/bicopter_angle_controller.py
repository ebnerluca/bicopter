#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from os import path
from yaml import load as yaml_load

import rclpy
from rclpy.node import Node
import numpy as np
from tf_transformations import euler_from_quaternion

from bicopter_msgs.msg import AngleCommands, ActuatorCommands, SensorReadings


class BicopterAngleController(Node):

    # Constructor
    def __init__(self):
        super().__init__('bicopter_angle_controller')
        self.log("__init__")
        
        # declare parameters
        self.declare_parameter("controller_rate")
        self.declare_parameter("angle_commands_topic")
        self.declare_parameter("actuator_commands_topic")
        self.declare_parameter("imu_readings_topic")
        self.declare_parameter("gains.roll.kp")
        self.declare_parameter("gains.roll.kd")
        self.declare_parameter("gains.pitch.kp")
        self.declare_parameter("gains.pitch.kd")
        self.declare_parameter("gains.yaw.kp")
        self.declare_parameter("gains.yaw.kd")
        self.declare_parameter("gains.height.kp")
        self.declare_parameter("gains.height.kd")
        self.declare_parameter("model_properties.package")
        self.declare_parameter("model_properties.file")

        # get parameters
        self.controllerRate = self.get_parameter("controller_rate").value
        self.angleCommandsTopic = self.get_parameter("angle_commands_topic").value
        self.actuatorCommandsTopic = self.get_parameter("actuator_commands_topic").value
        self.IMUreadingsTopic = self.get_parameter("imu_readings_topic").value
        self.r_kp = self.get_parameter("gains.roll.kp").value
        self.r_kd = self.get_parameter("gains.roll.kd").value
        self.p_kp = self.get_parameter("gains.pitch.kp").value
        self.p_kd = self.get_parameter("gains.pitch.kd").value
        self.y_kp = self.get_parameter("gains.yaw.kp").value
        self.y_kd = self.get_parameter("gains.yaw.kd").value
        self.z_kp = self.get_parameter("gains.height.kp").value
        self.z_kd = self.get_parameter("gains.height.kd").value
        model_properties_package = self.get_parameter("model_properties.package").value
        model_properties_file = self.get_parameter("model_properties.file").value

        # load model properties
        model_properties_dict = self.load_model_properties(model_properties_package, model_properties_file)

        self.bicopter_arm_l1 = model_properties_dict["bicopter_arm_l1"]
        self.bicopter_arm_l2 = model_properties_dict["bicopter_arm_l2"]
        self.bicopter_Ixx = model_properties_dict["bicopter_Ixx"]
        self.bicopter_Iyy = model_properties_dict["bicopter_Iyy"]
        self.bicopter_Izz = model_properties_dict["bicopter_Izz"]
        self.bicopter_mass = model_properties_dict["bicopter_mass"]
        self.bicopter_prop_c1 = model_properties_dict["bicopter_prop_lift_coeff"]


        # class variables
        self.jacobian = self.get_jacobian()

        # Publishers
        self.actuatorCommandPublisher = self.create_publisher(ActuatorCommands, self.actuatorCommandsTopic, 1)
        self.actuatorCommands = ActuatorCommands()  # [speed_left, speed_right, servo_left, servo_right]
        self.actuatorCommands.r1 = 0.0
        self.actuatorCommands.r2 = 0.0
        self.actuatorCommands.b1 = 0.0
        self.actuatorCommands.b2 = 0.0

        # Subscribers
        self.create_subscription(SensorReadings, self.IMUreadingsTopic, self.imu_readings_callback, 1)
        self.IMUreadings = None  # [q_x, q_y, q_z, q_w, w_x, w_y, w_z]
        self.create_subscription(AngleCommands, self.angleCommandsTopic, self.angle_commands_callback, 1)
        self.angleCommands = AngleCommands()
        # Main loop for publishing commands with fixed freq
        self.create_timer(1.0/self.controllerRate, self.publish_commands)

    def load_model_properties(self, package, file):
        print(f"loading package {package} and file {file}")
        package_path = get_package_share_directory(package)
        
        with open(path.join(package_path, file), "r") as f:
            return yaml_load(f)

    def angle_commands_callback(self, commands):
        self.angleCommands = commands

    def imu_readings_callback(self, readings):

        q = [readings.q_x, readings.q_y, readings.q_z, readings.q_w]
        r, p, y = euler_from_quaternion(q)

        # PD controller
        ddr = self.r_kp * (self.angleCommands.r - r) + self.r_kd * (self.angleCommands.w_x - readings.w_x)
        ddp = self.p_kp * (self.angleCommands.p - p) + self.p_kd * (self.angleCommands.w_y - readings.w_y)
        ddy = self.y_kp * (self.angleCommands.y - y) + self.y_kd * (self.angleCommands.w_z - readings.w_z)
        throttle = self.z_kp * (self.angleCommands.z - readings.z) + self.z_kd * (self.angleCommands.v_z - readings.v_z)

        # update actuator commands
        # self.jacobian = self.get_jacobian()  # if dynamic jacobian is used
        jinv = np.linalg.pinv(self.jacobian)
        e = np.array([ddr, ddp, ddy, throttle])
        u = jinv.dot(e)  # computed actuator commands

        # self.actuatorCommands.r1 = np.clip(u[0], 0.5, np.sqrt(4))
        # self.actuatorCommands.r2 = np.clip(u[1], 0.5, np.sqrt(4))
        self.actuatorCommands.r1 = np.max([u[0], 0.0])  # props can only spin in one direction
        self.actuatorCommands.r2 = np.max([u[1], 0.0])  # props can only spin in one direction
        self.actuatorCommands.b1 = u[2]
        self.actuatorCommands.b2 = u[3]

    def get_jacobian(self):

        k1 = self.bicopter_arm_l1 * self.bicopter_prop_c1 / self.bicopter_Ixx
        k2 = self.bicopter_arm_l2 * self.bicopter_prop_c1 / self.bicopter_Iyy
        k3 = self.bicopter_arm_l1 * self.bicopter_prop_c1 / self.bicopter_Izz

        # linearized jacobian around current state
        # jacobian = np.array([
        #     [k1 * np.cos(self.b1) * self.r1, -k1 * np.cos(self.b2) * self.r2, -k1 * np.sin(self.b1) * self.r1 * self.r1, -k1 * np.sin(self.b1) * self.r2 * self.r2],
        #     [k2 * np.sin(self.b1) * self.r1, k2 * np.sin(self.b2) * self.r2, k2 * np.cos(self.b1) * self.r1 * self.r1, k2 * np.cos(self.b1) * self.r2 * self.r2],
        #     [-k3 * np.sin(self.b1) * self.r1, k3 * np.sin(self.b2) * self.r2, -k3 * np.cos(self.b1) * self.r1 * self.r1, k3 * np.cos(self.b1) * self.r2 * self.r2],
        #     [self.bicopter_prop_c1 * self.r1 / self.bicopter_mass, self.bicopter_prop_c1 + self.r2 / self.bicopter_mass, 0.0, 0.0]
        # ])

        # linearized jacobian around equilibrium
        r_eq = self.bicopter_mass * 9.81 / (2.0 * self.bicopter_prop_c1)
        jacobian = np.array([
            [k1 * r_eq, -k1 * r_eq, 0.0, 0.0],
            [0.0, 0.0, k2 * r_eq * r_eq, k2 * r_eq * r_eq],
            [0.0, 0.0, -k3 * r_eq * r_eq, k3 * r_eq * r_eq],
            [self.bicopter_prop_c1 / self.bicopter_mass, self.bicopter_prop_c1 / self.bicopter_mass, 0.0, 0.0]
        ])

        return jacobian

    def log(self, stuff):
        self.get_logger().info("[BicopterAngleController]: " + stuff)

    def publish_commands(self):
        self.actuatorCommandPublisher.publish(self.actuatorCommands)

    # On ROS shutdown
    def on_shutdown(self):
        self.log("Shutting down...")
        return


def main(args=None):
    rclpy.init(args=args)

    node = BicopterAngleController()

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
