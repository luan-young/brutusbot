import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("brutusbot_description"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("brutusbot_controller"), "launch", "controller.launch.py")
        )
    )

    joystick_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("brutusbot_controller"), "launch", "joystick_teleop.launch.py")
        ),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    return LaunchDescription([
        gazebo_launch,
        controller_launch,
        joystick_teleop_launch
    ])