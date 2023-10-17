from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    ld = LaunchDescription()

    # simulator
    simulator = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("bicopter_pybullet_controller"),
                "launch",
                "run_bicopter_pybullet_controller.launch.py"
            ]
        )
    )

    # angle controller
    angle_controller = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("bicopter_angle_controller"),
                "launch",
                "run_bicopter_angle_controller.launch.py"
            ]
        )
    )

    # input controller
    input_controller = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("bicopter_input_controller"),
                "launch",
                "run_bicopter_input_controller.launch.py"
            ]
        )
    )

    ld.add_action(input_controller)
    ld.add_action(angle_controller)
    ld.add_action(simulator)

    return ld