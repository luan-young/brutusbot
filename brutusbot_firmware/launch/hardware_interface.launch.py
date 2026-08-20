import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    description_pkg_dir = get_package_share_directory("brutusbot_description")
    controller_pkg_dir = get_package_share_directory("brutusbot_controller")

    urdf_file = os.path.join(description_pkg_dir, "urdf", "brutusbot.xacro")
    controllers_file = os.path.join(controller_pkg_dir, "config", "brutusbot_controllers.yaml")

    robot_description = xacro.process_file(
        urdf_file,
        mappings={"is_sim": "false"},
    ).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            }
        ],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_file,
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            },
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager,
        ]
    )
