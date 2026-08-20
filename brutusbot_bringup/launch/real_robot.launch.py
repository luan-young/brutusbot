import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("brutusbot_firmware"),
                "launch",
                "hardware_interface.launch.py",
            )
        )
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("brutusbot_controller"),
                "launch",
                "controller.launch.py",
            )
        )
    )

    joystick_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("brutusbot_controller"),
                "launch",
                "joystick_teleop.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
    )

    return LaunchDescription(
        [
            hardware_interface_launch,
            controller_launch,
            joystick_teleop_launch,
        ]
    )
