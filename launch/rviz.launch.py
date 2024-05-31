import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


# Specify the name of the package
pkg_name = "bicycle_sim"
pkg_share_directory = get_package_share_directory(pkg_name)


def generate_launch_description():
    # Use xacro to process the file
    urdf_path = "urdf/bicycle.urdf.xacro"
    xacro_file = os.path.join(pkg_share_directory, urdf_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # RViz2 config
    config_path = "rviz/rviz_config.rviz"
    rviz_config = os.path.join(pkg_share_directory, config_path)

    # rsp
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw}],
    )

    # jsp
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        arguments=[xacro_file],
    )

    # rviz2
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d" + rviz_config],
    )

    # Run the nodes
    return LaunchDescription(
        [
            rsp_node,
            jsp_gui_node,
            rviz2_node,
        ]
    )
