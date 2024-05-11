from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    # Use xacro to process the file
    urdf_path = "urdf/bicycle.urdf.xacro"
    xacro_file = os.path.join(pkg_share_directory, urdf_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the nodes
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_raw},
            {"use_sim_time": True},
        ],
    )

    # Set the path to the world file
    world_file_name = "obstacles.world"
    world_path = os.path.join(pkg_share_directory, "worlds", world_file_name)

    # Spawn service
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "bicycle",
            "-allow_renaming",
            "true",
        ],
    )

    # Controller run
    run_control_manager = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "ros2_control_node",
        ],
        output="screen",
    )
    # Controller loader
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_broad",
        ],
        output="screen",
    )

    load_bicycle_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "bicycle_controller",
        ],
        output="screen",
    )

    # Run the nodes
    return LaunchDescription(
        [
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[("gz_args", [" -r -v 4 empty.sdf"])],
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[run_control_manager],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_bicycle_controller],
                )
            ),
            node_robot_state_publisher,
            gz_spawn_entity,
            # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
        ]
    )
