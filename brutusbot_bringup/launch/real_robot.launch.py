import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


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

    camera_config = os.path.join(get_package_share_directory("brutusbot_bringup"), "config", "v4l2_camera.yaml")
    with open(camera_config) as f:
        video_device = yaml.safe_load(f)["v4l2_camera"]["ros__parameters"]["video_device"]

    # v4l2_camera applies exposure_time_absolute while the camera may still be in auto exposure, where the control is
    # inactive and the value is silently skipped. Switching to manual first lets the node apply the configured value.
    camera_manual_exposure = ExecuteProcess(
        cmd=["v4l2-ctl", "-d", video_device, "-c", "auto_exposure=1"],
        output="screen",
    )

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        parameters=[camera_config],
        output="screen",
    )

    camera = GroupAction(
        [
            camera_manual_exposure,
            RegisterEventHandler(OnProcessExit(target_action=camera_manual_exposure, on_exit=[camera_node])),
        ],
        condition=IfCondition(LaunchConfiguration("camera")),
    )

    return LaunchDescription(
        [
            camera_arg,
            hardware_interface_launch,
            controller_launch,
            joystick_teleop_launch,
            camera,
        ]
    )
