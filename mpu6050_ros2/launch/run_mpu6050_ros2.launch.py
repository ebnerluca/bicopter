from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()
    parameters = os.path.join(get_package_share_directory('mpu6050_ros2'), 'config', 'default.yaml')

    node = Node(package='mpu6050_ros2',
                namespace='mpu6050',
                executable='mpu6050_node',
                name='mpu6050_node',
                parameters=[parameters],
                emulate_tty=True
                )

    ld.add_action(node)
    return ld
