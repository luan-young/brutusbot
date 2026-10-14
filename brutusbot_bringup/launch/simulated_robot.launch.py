import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty",
        description="World in brutusbot_description/worlds: empty, textured_room or calibration",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("brutusbot_description"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"use_sim_time": "True", "world": LaunchConfiguration("world")}.items()
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
        world_arg,
        gazebo_launch,
        controller_launch,
        joystick_teleop_launch
    ])