import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Paths to URDF models
    description_pkg = "int-ball2_description"  # Change this if your package name is different

    ib2_urdf = os.path.join(
        get_package_share_directory(description_pkg), "model", "ib2", "ib2.urdf"
    )

    custom_object_urdf = os.path.join(
        get_package_share_directory(description_pkg), "model", "custom_object_01", "custom_object_01.urdf"
    )

    iss_urdf = os.path.join(
        get_package_share_directory(description_pkg), "urdf", "iss.urdf"
    )

    # Nodes for state publishers
    ib2_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ib2_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["cat ", ib2_urdf])}],
        remappings=[("robot_description", "ib2_description")]
    )

    custom_object_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="custom_object_01_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["cat ", custom_object_urdf])}],
        remappings=[("robot_description", "custom_obj_description")]
    )

    iss_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="iss_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["cat ", iss_urdf])}],
        remappings=[("robot_description", "iss_description")]
    )

    # RViz Node
    rviz_config_path = os.path.join(
        get_package_share_directory(description_pkg), "rviz", "urdf.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    # Transformation Nodes
    trans_ib2 = Node(
        package="trans",
        executable="trans",
        name="trans_ib2",
        output="screen",
        parameters=[
            {"model_name": "ib2"},
            {"world_frame": "base"},
            {"base_frame": "body"},
            {"updateFreqHz": 50},
        ],
    )

    trans_iss = Node(
        package="trans",
        executable="trans",
        name="trans_iss",
        output="screen",
        parameters=[
            {"model_name": "iss"},
            {"world_frame": "base"},
            {"base_frame": "iss_body"},
            {"updateFreqHz": 50},
        ],
    )

    trans_custom_object = Node(
        package="trans",
        executable="trans",
        name="trans_custom_object_01",
        output="screen",
        parameters=[
            {"model_name": "custom_object_01"},
            {"world_frame": "base"},
            {"base_frame": "custom_object_01_body"},
            {"updateFreqHz": 50},
        ],
    )

    # Launch Description
    return LaunchDescription([
        ib2_state_publisher,
        custom_object_state_publisher,
        iss_state_publisher,
        rviz_node,
        # trans_ib2,
        # trans_iss,
        # trans_custom_object,
    ])
