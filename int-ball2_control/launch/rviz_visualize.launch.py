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
    package_description = "int-ball2_control"

    # URDF Configuration
    ib2_urdf = os.path.join(
        get_package_share_directory(package_description),
        'description',
        'ib2.urdf'
    )
    iss_urdf = os.path.join(
        get_package_share_directory(package_description),
        'description',
        'kibou.urdf'
    )

    with open(ib2_urdf, 'r') as urdf_file:
        ib2_description_content = urdf_file.read()

    with open(iss_urdf, 'r') as urdf_file:
        iss_description_content = urdf_file.read()

    ib2_description = {'robot_description': ib2_description_content}
    iss_description = {'robot_description': iss_description_content}

    # RVIZ Configuration
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_description), 
        'rviz', 
        'int-ball2_vis.rviz'
    )

    # Add robot_state_publisher node
    ib2_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="ib2_state_publisher",
        output='screen',
        parameters=[ib2_description],
        remappings=[("robot_description", "ib2_description")]
    )

    iss_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="iss_state_publisher",
        output="screen",
        parameters=[iss_description],
        remappings=[("robot_description", "iss_description")]
    )

    # Add RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz',
        parameters=[
            {'use_sim_time': True},
            ib2_description
        ],
        arguments=['-d', rviz_config_dir]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            ib2_state_publisher_node,
            iss_state_publisher_node,
            rviz_node,
        ]
    )