import pybullet as p
import pybullet_data
import math
import time

physics_client_id = p.connect(p.GUI)

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(False)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", basePosition=[0.0, 0.0, 0.0])
bicopterId = p.loadURDF("/home/ebnerl/dev_ws/src/bicopter/bicopter_model/urdf/bicopter_urdf.urdf")

# sim frequency
rate = 240.0  # Hz

# pybullet pos controller gains
kp = [1, 1]
kd = [0.1, 0.1]

# rotor speeds
speed_left = 1.0
speed_right = 1.0

t = 0.0
while True:

    # step
    t = t + 1.0/rate
    p.stepSimulation()

    # set servo positions
    servo_left = 0.78 * math.sin(t)
    servo_right = 0.78 * -math.sin(t)
    p.setJointMotorControlArray(bodyUniqueId=bicopterId, jointIndices=[0, 1], controlMode=p.POSITION_CONTROL,
                                targetPositions=[servo_left, servo_right],
                                targetVelocities=[0.0, 0.0],
                                positionGains=kp,
                                velocityGains=kd)

    p.applyExternalForce(objectUniqueId=bicopterId, linkIndex=0, forceObj=[0.0, 0.0, speed_left], posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)
    p.applyExternalForce(objectUniqueId=bicopterId, linkIndex=1, forceObj=[0.0, 0.0, speed_right], posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME)

    time.sleep(1.0/rate)

