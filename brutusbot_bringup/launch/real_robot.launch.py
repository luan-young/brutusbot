import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="true",
        description="Start the v4l2_camera node (/image_raw, /camera_info)",
    )

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

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        parameters=[
            os.path.join(get_package_share_directory("brutusbot_bringup"), "config", "v4l2_camera.yaml"),
        ],
        condition=IfCondition(LaunchConfiguration("camera")),
        output="screen",
    )

    return LaunchDescription(
        [
            camera_arg,
            hardware_interface_launch,
            controller_launch,
            joystick_teleop_launch,
            camera_node,
        ]
    )
