#!/usr/bin/env python

# import geometry_msgs.msg
import rospy

import pybullet as p
import pybullet_data

import numpy as np

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

from bicopter_msgs.msg import ActuatorCommands, SensorReadings
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import TransformStamped


class BicopterPybulletController():

    # Constructor
    def __init__(self):
        self.log("__init__")

        # parameters
        self.controllerRate = rospy.get_param("controller_rate")
        self.actuatorCommandsTopic = rospy.get_param("actuator_commands_topic")
        self.IMUreadingsTopic = rospy.get_param("imu_readings_topic")
        self.URDFpath = rospy.get_param("urdf_path")
        self.bicopter_prop_c1 = rospy.get_param("/bicopter_model/bicopter_prop_lift_coeff")

        self.actuatorCommands = [0.0, 0.0, 0.0, 0.0]  # [speed_left, speed_right, servo_left, servo_right]
        self.newestCommandStamp = None

        # Publishers
        self.IMUpublisher = rospy.Publisher(self.IMUreadingsTopic, SensorReadings, queue_size=10)
        self.IMUreadings = SensorReadings()

        # Subscribers
        rospy.Subscriber(self.actuatorCommandsTopic, ActuatorCommands, self.actuator_commands_callback)

        # Services
        # rospy.Service("call_service", Trigger, self.service_callback)

        # PyBullet
        self.pybullet_is_realtime = False
        self.biCopterId = self.initialize_pybullet()

    def initialize_pybullet(self):

        # Connecting
        physics_client_id = p.connect(p.GUI)
        self.log("PyBullet client connected with ID: " + str(physics_client_id))

        # Config
        p.setGravity(0, 0, -10)
        p.setRealTimeSimulation(False)

        # Spawn ground plane
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.loadURDF("plane.urdf", basePosition=[0.0, 0.0, 0.0])
        return p.loadURDF(self.URDFpath)

    def actuator_commands_callback(self, commands):

        # update actuator commands
        self.actuatorCommands[0] = commands.r1
        self.actuatorCommands[1] = commands.r2
        self.actuatorCommands[2] = commands.b1
        self.actuatorCommands[3] = commands.b2

        self.newestCommandStamp = commands.header.stamp

    def service_callback(self, request):
        response = TriggerResponse()
        response.success = True
        response.message = "Placeholder."

        self.log("Placeholder.")
        return response

    def log(self, stuff):
        rospy.loginfo("[BicopterPybulletController]: " + stuff)

    # Main Loop
    def simulate(self):

        rate = rospy.Rate(self.controllerRate)

        while not rospy.is_shutdown():

            # set servos
            p.setJointMotorControlArray(bodyUniqueId=self.biCopterId, jointIndices=[0, 1],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=[self.actuatorCommands[2], self.actuatorCommands[3]],
                                        targetVelocities=[0.0, 0.0],
                                        positionGains=[1.0, 1.0],
                                        velocityGains=[0.5, 0.5])

            # apply thrust
            p.applyExternalForce(objectUniqueId=self.biCopterId, linkIndex=0,
                                 forceObj=[0.0, 0.0, self.bicopter_prop_c1 * self.actuatorCommands[0]*self.actuatorCommands[0]],
                                 posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)
            p.applyExternalForce(objectUniqueId=self.biCopterId, linkIndex=1,
                                 forceObj=[0.0, 0.0, self.bicopter_prop_c1 * self.actuatorCommands[1]*self.actuatorCommands[1]],
                                 posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)

            # simulation step
            p.stepSimulation()

            # get readings
            pos, orientation = p.getBasePositionAndOrientation(self.biCopterId)
            # rpy = np.around(np.array(p.getEulerFromQuaternion(orientation)), 2)
            self.IMUreadings.q_x = orientation[0]
            self.IMUreadings.q_y = orientation[1]
            self.IMUreadings.q_z = orientation[2]
            self.IMUreadings.q_w = orientation[3]

            vel, angular_vel = p.getBaseVelocity(self.biCopterId)
            rotmat = np.array(p.getMatrixFromQuaternion(orientation)).reshape((3, 3), order="F")
            drpy = rotmat.dot(np.array(angular_vel))

            self.IMUreadings.w_x = drpy[0]
            self.IMUreadings.w_y = drpy[1]
            self.IMUreadings.w_z = drpy[2]

            self.IMUreadings.height = pos[2]
            self.IMUreadings.v_z = vel[2]

            # publish readings
            self.IMUpublisher.publish(self.IMUreadings)

            # wait
            rate.sleep()

    # On ROS shutdown
    def on_shutdown(self):
        self.log("Shutting down...")
        p.disconnect()
        self.log("PyBullet client disconnected.")


if __name__ == '__main__':

    # Initialize node and name it.
    rospy.init_node('bicopter_pybullet_controller')

    try:
        node = BicopterPybulletController()
        node.log("Node is running.")
        node.simulate()

        rospy.spin()

        rospy.on_shutdown(node.on_shutdown)

    except rospy.ROSInterruptException:
        pass
