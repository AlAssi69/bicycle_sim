import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


# Specify the name of the package
pkg_name = "bicycle_sim"


def generate_launch_description():
    # Use xacro to process the file
    urdf_path = "urdf/bicycle.urdf.xacro"
    xacro_file = os.path.join(get_package_share_directory(pkg_name), urdf_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the nodes
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw}],
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
    )

    # Spawn service
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "bicycle"],
        output="screen",
    )

    # Run the nodes
    return LaunchDescription([gazebo, node_robot_state_publisher, spawn_entity])
