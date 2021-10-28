from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()
    parameters = os.path.join(get_package_share_directory('bicopter_pybullet_controller'), 'config', 'default.yaml')

    node = Node(package='bicopter_pybullet_controller',
                namespace='bicopter_pybullet_controller',
                executable='bicopter_pybullet_controller_node',
                name='bicopter_pybullet_controller_node',
                parameters=[parameters],
                emulate_tty=True
                )
    ld.add_action(node)
    return ld
