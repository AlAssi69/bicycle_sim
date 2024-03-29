import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


# Specify the name of the package
pkg_name = "bicycle_sim"


def generate_launch_description():

    # Use xacro to process the file
    urdf_path = "urdf/bicycle.urdf.xacro"
    xacro_file = os.path.join(get_package_share_directory(pkg_name), urdf_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # RViz2 config
    config_path = "rviz/rviz_config.rviz"
    rvizconfig = os.path.join(get_package_share_directory(pkg_name), config_path)

    # Configure the nodes
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw}],
    )

    node_joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    node_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d" + rvizconfig],
    )

    nodes = [node_robot_state_publisher, node_rviz2, node_joint_state_publisher]

    # Run the nodes
    return LaunchDescription(nodes)
