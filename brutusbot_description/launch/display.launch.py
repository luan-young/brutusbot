from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro


def generate_launch_description():
    description_pkg_dir = get_package_share_directory("brutusbot_description")
    urdf_file = os.path.join(description_pkg_dir, "urdf", "brutusbot.xacro")
    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher",
        output="screen",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(description_pkg_dir, "rviz", "display.rviz")],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz2_node,
    ])
