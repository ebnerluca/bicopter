from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bicopter_lowlevel_controller',
            namespace='bicopter',
            executable='bicopter_lowlevel_controller_node',
            name='bicopter_lowlevel_controller'
        ),

    ])
