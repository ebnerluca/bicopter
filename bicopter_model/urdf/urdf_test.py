import pybullet as p
import pybullet_data
import math
import time

physics_client_id = p.connect(p.GUI)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(True)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", basePosition=[0.0, 0.0, 0.0])
bicopterId = p.loadURDF("/home/ebnerl/catkin_ws/src/bicopter/bicopter_pybullet_controller/urdf/bicopter_urdf.urdf")

index = 0.0
index_step = 0.00001
rate = 200  # Hz

kp = [1, 1]
kd = [0.1, 0.1]

while True:
    servo_left = 1.2 * math.sin(index)
    servo_right = 1.2 * -math.sin(index)
    index = index + index_step
    p.setJointMotorControlArray(bodyUniqueId=bicopterId, jointIndices=[0, 1], controlMode=p.POSITION_CONTROL,
                                targetPositions=[servo_left, servo_right],
                                targetVelocities=[0.0, 0.0],
                                positionGains=kp,
                                velocityGains=kd)

    speed_left = 10.0
    speed_right = 10.0
    p.applyExternalForce(objectUniqueId=bicopterId, linkIndex=0, forceObj=[0.0, 0.0, speed_left], posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)
    p.applyExternalForce(objectUniqueId=bicopterId, linkIndex=1, forceObj=[0.0, 0.0, speed_right], posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)

    time.sleep(1/rate)

