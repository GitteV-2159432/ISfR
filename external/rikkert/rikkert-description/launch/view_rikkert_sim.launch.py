import os
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("rikkert_description"),
            "urdf",
            "rikkert.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    rviz_config_file = PathJoinSubstitution([FindPackageShare("rikkert_description"), "rviz", "config.rviz"])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
        # arguments=["-d"],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        rviz_node
        # joint_state_publisher_gui_node
    ]

    return LaunchDescription(nodes_to_start)