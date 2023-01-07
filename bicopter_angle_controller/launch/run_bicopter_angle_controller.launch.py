from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    controller_params = PathJoinSubstitution(
        [
            FindPackageShare("bicopter_angle_controller"),
            "config",
            "default.yaml"
        ]
    )

    node = Node(package='bicopter_angle_controller',
                executable='bicopter_angle_controller',
                name='bicopter_angle_controller',
                parameters=[controller_params],#, model_params],
                output="screen",
                emulate_tty=True
                )

    ld.add_action(node)

    return ld