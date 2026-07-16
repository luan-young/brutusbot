from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from pathlib import Path
import xacro


def generate_launch_description():
    description_pkg_dir = get_package_share_directory("brutusbot_description")
    urdf_file = os.path.join(description_pkg_dir, "urdf", "brutusbot.xacro")
    robot_description = xacro.process_file(urdf_file).toxml()

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(description_pkg_dir).parent.resolve())
            ]
        )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ]),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

    gz_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "brutusbot",
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity_node,
        gz_ros2_bridge,
    ])