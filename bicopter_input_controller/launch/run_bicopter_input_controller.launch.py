from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # controller_params = PathJoinSubstitution(
    #     [
    #         FindPackageShare("bicopter_input_controller"),
    #         "config",
    #         "default.yaml"
    #     ]
    # )

    input_controller = Node(package='bicopter_input_controller',
                executable='bicopter_input_controller',
                name='bicopter_input_controller',
                # parameters=[controller_params],#, model_params],
                output="screen",
                # emulate_tty=True
                )
    
    joy_node = Node(package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                )

    ld.add_action(input_controller)
    ld.add_action(joy_node)

    return ld