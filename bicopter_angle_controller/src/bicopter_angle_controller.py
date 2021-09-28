#!/usr/bin/env python

# import geometry_msgs.msg
import rospy

import numpy as np

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_matrix

from bicopter_msgs.msg import ActuatorCommands, SensorReadings
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import TransformStamped


class BicopterAngleController():

    # Constructor
    def __init__(self):
        self.log("__init__")

        # parameters
        self.controllerRate = rospy.get_param("controller_rate")
        self.actuatorCommandsTopic = rospy.get_param("actuator_commands_topic")
        self.IMUreadingsTopic = rospy.get_param("imu_readings_topic")
        self.r_kp = rospy.get_param("gains/roll/kp")
        self.r_kd = rospy.get_param("gains/roll/kd")
        self.p_kp = rospy.get_param("gains/pitch/kp")
        self.p_kd = rospy.get_param("gains/pitch/kd")
        self.y_kp = rospy.get_param("gains/yaw/kp")
        self.y_kd = rospy.get_param("gains/yaw/kd")
        self.bicopter_prop_c1 = rospy.get_param("/bicopter_pybullet_controller/prop_c1")
        self.bicopter_arm_l1 = 0.110
        self.bicopter_arm_l2 = 0.120
        self.bicopter_Ixx = 0.003
        self.bicopter_Iyy = 0.001
        self.bicopter_Izz = 0.003
        self.bicopter_mass = 0.6

        # class variables

        # Publishers
        self.actuatorCommandPublisher = rospy.Publisher(self.actuatorCommandsTopic, ActuatorCommands, queue_size=10)
        self.actuatorCommands = ActuatorCommands()  # [speed_left, speed_right, servo_left, servo_right]
        # self.actuatorCommands.header.stamp = rospy.time.now()
        self.actuatorCommands.r1 = 1.0
        self.actuatorCommands.r2 = 1.0
        self.actuatorCommands.b1 = 0.0
        self.actuatorCommands.b2 = 0.0
        """self.r1 = 0.0  # speed left
        self.r2 = 0.0  # speed right
        self.b1 = 0.0  # servo_left
        self.b2 = 0.0  # servo_right"""

        # Subscribers
        rospy.Subscriber(self.IMUreadingsTopic, SensorReadings, self.imu_readings_callback)
        self.IMUreadings = None  # [q_x, q_y, q_z, q_w, w_x, w_y, w_z]

        # Services
        # rospy.Service("call_service", Trigger, self.service_callback)

    def imu_readings_callback(self, readings):

        rospy.loginfo_throttle(5, "Received IMU readings.")
        q = [readings.q_x, readings.q_y, readings.q_z, readings.q_w]
        r, p, y = euler_from_quaternion(q)
        R = quaternion_matrix(q)[0:3, 0:3]
        w = R.dot(np.array([readings.w_x, readings.w_y, readings.w_z]))
        #rospy.loginfo("rpy: " + str(np.around(np.array([r, p, y]), 2)) + ", drpy: " + str(np.around(w, 2)))
        ddr = self.r_kp * (0.0 - r) + self.r_kd * (0.0 - w[0])
        ddp = self.p_kp * (0.0 - p) + self.p_kd * (0.0 - w[1])
        ddy = self.y_kp * (0.0 - y) + self.y_kd * (0.0 - w[2])
        throttle = 5.50

        # update actuator commands
        jacobian = self.get_jacobian()
        # rospy.loginfo("j: \n" + str(jacobian))
        jinv = np.linalg.pinv(jacobian)
        rospy.loginfo("jinv: \n" + str(jinv))
        e = np.array([ddr, ddp, ddy, throttle])
        rospy.loginfo("e: \n" + str(e))
        u = jinv.dot(e)

        self.actuatorCommands.r1 = np.clip(u[0], 0.0, np.sqrt(3.1))
        self.actuatorCommands.r2 = np.clip(u[1], 0.0, np.sqrt(3.1))
        self.actuatorCommands.b1 = u[2]
        self.actuatorCommands.b2 = u[3]

    def get_jacobian(self):

        k1 = self.bicopter_arm_l1 * self.bicopter_prop_c1 / self.bicopter_Ixx
        k2 = self.bicopter_arm_l2 * self.bicopter_prop_c1 / self.bicopter_Iyy
        k3 = self.bicopter_arm_l1 * self.bicopter_prop_c1 / self.bicopter_Izz

        """jacobian = np.array([
            [k1 * np.cos(self.b1) * self.r1, -k1 * np.cos(self.b2) * self.r2, -k1 * np.sin(self.b1) * self.r1 * self.r1, -k1 * np.sin(self.b1) * self.r2 * self.r2],
            [k2 * np.sin(self.b1) * self.r1, k2 * np.sin(self.b2) * self.r2, k2 * np.cos(self.b1) * self.r1 * self.r1, k2 * np.cos(self.b1) * self.r2 * self.r2],
            [-k3 * np.sin(self.b1) * self.r1, k3 * np.sin(self.b2) * self.r2, -k3 * np.cos(self.b1) * self.r1 * self.r1, k3 * np.cos(self.b1) * self.r2 * self.r2],
            [self.bicopter_prop_c1 * self.r1 / self.bicopter_mass, self.bicopter_prop_c1 + self.r2 / self.bicopter_mass, 0.0, 0.0]
        ])"""

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
        rospy.loginfo("[BicopterPybulletController]: " + stuff)

    # Main Loop
    def run(self):

        rate = rospy.Rate(self.controllerRate)

        while not rospy.is_shutdown():

            # publish actuator commands
            self.actuatorCommandPublisher.publish(self.actuatorCommands)

            # update state
            """self.r1 = self.actuatorCommands.r1
            self.r2 = self.actuatorCommands.r2
            self.b1 = self.actuatorCommands.b1
            self.b2 = self.actuatorCommands.b2"""

            # wait
            rate.sleep()

    # On ROS shutdown
    def on_shutdown(self):
        self.log("Shutting down...")
        return


if __name__ == '__main__':

    # Initialize node and name it.
    rospy.init_node('bicopter_angle_controller')

    try:
        node = BicopterAngleController()
        rospy.loginfo("[BicopterPybulletController]: Node is running.")
        node.run()

        rospy.spin()

        rospy.on_shutdown(node.on_shutdown)

    except rospy.ROSInterruptException:
        pass
