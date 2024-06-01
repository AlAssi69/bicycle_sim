import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    gazebo_param_file = os.path.join(pkg_share_directory, urdf_path)
    # rsp
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_raw},
            {"use_sim_time": True},
        ],
    )

    # gazebo
    param_file_path = "config/gazebo_param.yaml"
    gazebo_param_file = os.path.join(pkg_share_directory, param_file_path)
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ),
            ]
        ),
        launch_arguments={
            "extra_gazebo_args": "--ros-args --param-file " + gazebo_param_file
        }.items(),
    )

    # Spawn URDF (the bicycle) service
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "bicycle"],
        output="screen",
    )

    # joint_broad controller
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # bicycle_steering_cont controller
    bicycle_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bicycle_steering_cont"],
    )

    # Run the nodes
    return LaunchDescription(
        [
            rsp_node,
            gazebo_node,
            spawn_entity,
            joint_broad_spawner,
            bicycle_steering_controller_spawner,
        ]
    )
