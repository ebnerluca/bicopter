#!/usr/bin/env python

# import geometry_msgs.msg
import rospy

import pybullet as p
import pybullet_data

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

from bicopter_msgs.msg import ActuatorCommands
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import TransformStamped


class BicopterPybulletController():

    # Constructor
    def __init__(self):
        self.log("__init__")

        # parameters
        self.controllerRate = rospy.get_param("controller_rate")
        self.actuatorCommandsTopic = rospy.get_param("actuator_commands_topic")
        self.URDFpath = rospy.get_param("urdf_path")

        self.actuatorCommands = [0.0, 0.0, 0.0, 0.0]  # [speed_left, speed_right, servo_left, servo_right]
        self.newestCommandStamp = None

        # Publishers
        # self.RGBImagePublisher = rospy.Publisher(self.cameraRGBImageTopic, Image, queue_size=10

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
        self.actuatorCommands[0] = commands.speed_left
        self.actuatorCommands[1] = commands.speed_right
        self.actuatorCommands[2] = commands.servo_left
        self.actuatorCommands[3] = commands.servo_right

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
                                        velocityGains=[0.1, 0.1])

            # apply thrust
            p.applyExternalForce(objectUniqueId=self.biCopterId, linkIndex=0, forceObj=[0.0, 0.0, self.actuatorCommands[0]],
                                 posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)
            p.applyExternalForce(objectUniqueId=self.biCopterId, linkIndex=1, forceObj=[0.0, 0.0, self.actuatorCommands[1]],
                                 posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)

            p.stepSimulation()
            rate.sleep()

    # On ROS shutdown
    def on_shutdown(self):
        self.log("Shutting down...")
        p.disconnect()
        self.log("PyBullet client disconnected.")


if __name__ == '__main__':

    # Initialize node and name it.
    rospy.init_node('data_generation')

    try:
        node = BicopterPybulletController()
        rospy.loginfo("[BicopterPybulletController]: Node is running.")
        node.simulate()

        rospy.spin()

        rospy.on_shutdown(node.on_shutdown)

    except rospy.ROSInterruptException:
        pass
