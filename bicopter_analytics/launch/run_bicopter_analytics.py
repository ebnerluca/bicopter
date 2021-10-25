from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()
    parameters = os.path.join(get_package_share_directory('bicopter_analytics'), 'config', 'default.yaml')
    rviz2_config = os.path.join(get_package_share_directory('bicopter_analytics'), 'config', 'rviz2_config.rviz')

    node = Node(package='bicopter_analytics',
                namespace='bicopter_analytics',
                executable='bicopter_analytics_node',
                name='bicopter_analytics_node',
                parameters=[parameters],
                emulate_tty=True
                )
    rviz = Node(package='rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments= ['-d', str(rviz2_config)],
                output='screen',
                emulate_tty=True
                )

    ld.add_action(node)
    ld.add_action(rviz)
    return ld
