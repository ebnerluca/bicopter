#!/usr/bin/env python3

# import geometry_msgs.msg
import rclpy
from rclpy.node import Node

import numpy as np

# from std_srvs.srv import Trigger
# from std_msgs.msg import String
# from tf.transformations import euler_from_quaternion, quaternion_matrix
from tf_transformations import euler_from_quaternion

from bicopter_msgs.msg import ActuatorCommands, SensorReadings
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import TransformStamped

from ament_index_python.packages import get_package_share_directory
from os import path
import yaml


class BicopterAngleController(Node):

    # Constructor
    def __init__(self):
        super().__init__('bicopter_angle_controller')
        self.log("__init__")

        # parameters
        # self.controllerRate = rospy.get_param("controller_rate")
        # self.actuatorCommandsTopic = rospy.get_param("actuator_commands_topic")
        # self.IMUreadingsTopic = rospy.get_param("imu_readings_topic")
        # self.r_kp = rospy.get_param("gains/roll/kp")
        # self.r_kd = rospy.get_param("gains/roll/kd")
        # self.p_kp = rospy.get_param("gains/pitch/kp")
        # self.p_kd = rospy.get_param("gains/pitch/kd")
        # self.y_kp = rospy.get_param("gains/yaw/kp")
        # self.y_kd = rospy.get_param("gains/yaw/kd")
        # self.bicopter_arm_l1 = rospy.get_param("/bicopter_model/bicopter_arm_l1")
        # self.bicopter_arm_l2 = rospy.get_param("/bicopter_model/bicopter_arm_l2")
        # self.bicopter_Ixx = rospy.get_param("/bicopter_model/bicopter_Ixx")
        # self.bicopter_Iyy = rospy.get_param("/bicopter_model/bicopter_Iyy")
        # self.bicopter_Izz = rospy.get_param("/bicopter_model/bicopter_Izz")
        # self.bicopter_mass = rospy.get_param("/bicopter_model/bicopter_mass")
        # self.bicopter_prop_c1 = rospy.get_param("/bicopter_model/bicopter_prop_lift_coeff")
        
        # declare parameters
        self.declare_parameter("controller_rate")
        self.declare_parameter("actuator_commands_topic")
        self.declare_parameter("imu_readings_topic")
        self.declare_parameter("gains.roll.kp")
        self.declare_parameter("gains.roll.kd")
        self.declare_parameter("gains.pitch.kp")
        self.declare_parameter("gains.pitch.kd")
        self.declare_parameter("gains.yaw.kp")
        self.declare_parameter("gains.yaw.kd")
        self.declare_parameter("model_properties.package")
        self.declare_parameter("model_properties.file")

        # self.declare_parameter("/bicopter_model/bicopter_arm_l1", None)
        # self.declare_parameter("/bicopter_model/bicopter_arm_l2", None)
        # self.declare_parameter("/bicopter_model/bicopter_Ixx", None)
        # self.declare_parameter("/bicopter_model/bicopter_Iyy", None)
        # self.declare_parameter("/bicopter_model/bicopter_Izz", None)
        # self.declare_parameter("/bicopter_model/bicopter_mass", None)
        # self.declare_parameter("/bicopter_model/bicopter_prop_lift_coeff", None)

        # get parameters
        self.controllerRate = self.get_parameter("controller_rate").value
        self.actuatorCommandsTopic = self.get_parameter("actuator_commands_topic").value
        self.IMUreadingsTopic = self.get_parameter("imu_readings_topic").value
        self.r_kp = self.get_parameter("gains.roll.kp").value
        self.r_kd = self.get_parameter("gains.roll.kd").value
        self.p_kp = self.get_parameter("gains.pitch.kp").value
        self.p_kd = self.get_parameter("gains.pitch.kd").value
        self.y_kp = self.get_parameter("gains.yaw.kp").value
        self.y_kd = self.get_parameter("gains.yaw.kd").value
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
        # self.actuatorCommandPublisher = rospy.Publisher(self.actuatorCommandsTopic, ActuatorCommands, queue_size=10)
        self.actuatorCommandPublisher = self.create_publisher(ActuatorCommands, self.actuatorCommandsTopic, 1)
        self.actuatorCommands = ActuatorCommands()  # [speed_left, speed_right, servo_left, servo_right]
        # self.actuatorCommands.header.stamp = rospy.time.now()
        self.actuatorCommands.r1 = 0.0
        self.actuatorCommands.r2 = 0.0
        self.actuatorCommands.b1 = 0.0
        self.actuatorCommands.b2 = 0.0
        """self.r1 = 0.0  # speed left
        self.r2 = 0.0  # speed right
        self.b1 = 0.0  # servo_left
        self.b2 = 0.0  # servo_right"""

        # Subscribers
        # rospy.Subscriber(self.IMUreadingsTopic, SensorReadings, self.imu_readings_callback)
        self.create_subscription(SensorReadings, self.IMUreadingsTopic, self.imu_readings_callback, 1)
        self.IMUreadings = None  # [q_x, q_y, q_z, q_w, w_x, w_y, w_z]

        # Services
        # rospy.Service("call_service", Trigger, self.service_callback)

        # publish commands
        self.create_timer(1.0/self.controllerRate, self.run)

    def load_model_properties(self, package, file):
        print(f"loading package {package} and file {file}")
        package_path = get_package_share_directory(package)
        
        with open(path.join(package_path, file), "r") as f:
            return yaml.load(f)

    def imu_readings_callback(self, readings):

        q = [readings.q_x, readings.q_y, readings.q_z, readings.q_w]
        r, p, y = euler_from_quaternion(q)

        # PD controller
        ddr = self.r_kp * (0.0 - r) + self.r_kd * (0.0 - readings.w_x)
        ddp = self.p_kp * (0.0 - p) + self.p_kd * (0.0 - readings.w_y)
        ddy = self.y_kp * (0.0 - y) + self.y_kd * (0.0 - readings.w_z)
        throttle = 8.0 * (1.0 - readings.z) + 1.0 * (0.0 - readings.v_z)

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
        """jacobian = np.array([
            [k1 * np.cos(self.b1) * self.r1, -k1 * np.cos(self.b2) * self.r2, -k1 * np.sin(self.b1) * self.r1 * self.r1, -k1 * np.sin(self.b1) * self.r2 * self.r2],
            [k2 * np.sin(self.b1) * self.r1, k2 * np.sin(self.b2) * self.r2, k2 * np.cos(self.b1) * self.r1 * self.r1, k2 * np.cos(self.b1) * self.r2 * self.r2],
            [-k3 * np.sin(self.b1) * self.r1, k3 * np.sin(self.b2) * self.r2, -k3 * np.cos(self.b1) * self.r1 * self.r1, k3 * np.cos(self.b1) * self.r2 * self.r2],
            [self.bicopter_prop_c1 * self.r1 / self.bicopter_mass, self.bicopter_prop_c1 + self.r2 / self.bicopter_mass, 0.0, 0.0]
        ])"""

        # linearized jacobian around equilibrium
        r_eq = self.bicopter_mass * 9.81 / (2.0 * self.bicopter_prop_c1)
        jacobian = np.array([
            [k1 * r_eq, -k1 * r_eq, 0.0, 0.0],
            [0.0, 0.0, k2 * r_eq * r_eq, k2 * r_eq * r_eq],
            [0.0, 0.0, -k3 * r_eq * r_eq, k3 * r_eq * r_eq],
            [self.bicopter_prop_c1 / self.bicopter_mass, self.bicopter_prop_c1 / self.bicopter_mass, 0.0, 0.0]
        ])

        return jacobian

    def service_callback(self, request):
        response = TriggerResponse()
        response.success = True
        response.message = "Placeholder."

        self.log("Placeholder.")
        return response

    def log(self, stuff):
        self.get_logger().info("[BicopterAngleController]: " + stuff)

    # Main Loop
    def run(self):

        # rate = rospy.Rate(self.controllerRate)

        # while not rospy.is_shutdown():

        #     # publish actuator commands
        #     self.actuatorCommandPublisher.publish(self.actuatorCommands)

        #     # update state
        #     """self.r1 = self.actuatorCommands.r1
        #     self.r2 = self.actuatorCommands.r2
        #     self.b1 = self.actuatorCommands.b1
        #     self.b2 = self.actuatorCommands.b2"""

        #     # wait
        #     rate.sleep()

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

    # # Initialize node and name it.
    # rospy.init_node('bicopter_angle_controller')

    # try:
    #     node = BicopterAngleController()
    #     node.log("Node is running.")
    #     node.run()

    #     rospy.spin()

    #     rospy.on_shutdown(node.on_shutdown)

    # except rospy.ROSInterruptException:
    #     pass
