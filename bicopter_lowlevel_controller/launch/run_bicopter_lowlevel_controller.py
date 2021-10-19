from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()
    parameters = os.path.join(get_package_share_directory('bicopter_lowlevel_controller'), 'config', 'default.yaml')

    node = Node(package='bicopter_lowlevel_controller',
                namespace='bicopter',
                executable='bicopter_lowlevel_controller_node',
                name='bicopter_lowlevel_controller',
                parameters=[parameters]
                )
    ld.add_action(node)
    return ld
