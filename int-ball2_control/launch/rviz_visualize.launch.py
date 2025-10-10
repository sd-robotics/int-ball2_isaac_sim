import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import xacro

from launch_ros.actions import Node

def generate_launch_description():

    ####### DATA INPUT ##########
    package_description = "ib2_control"

    # URDF Configuration
    urdf_file_path = os.path.join(
        get_package_share_directory(package_description),
        'description',
        'ib2.urdf'
    )
    print(f"URDF file path: {urdf_file_path}")
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description = {'robot_description': robot_description_content}

    # RVIZ Configuration
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_description), 
        'rviz', 
        'int-ball2_vis.rviz'
    )

    # Add robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Add RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz',
        parameters=[
            {'use_sim_time': True},
            robot_description
        ],
        arguments=['-d', rviz_config_dir]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            robot_state_publisher_node,  # Add this node
            rviz_node,
        ]
    )