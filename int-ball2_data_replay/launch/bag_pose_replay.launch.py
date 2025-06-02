from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ros_bag_args = [
        DeclareLaunchArgument('bag_file', description='Path to the ROS 2 bag file to play'),
        DeclareLaunchArgument('rate', default_value='10.0', description='Playback rate'),
        DeclareLaunchArgument('remap', default_value='', description='Remap rules, e.g. /old:=/new'),
        DeclareLaunchArgument('start_offset', default_value='550.0', description='Delay in seconds before starting playback'),
        DeclareLaunchArgument('topics', default_value='', description='Comma-separated list of topics to play (empty for all)'),
    ]
    
    bag_pose_pub_node = Node(
        package='ib2_data_replay',
        executable='bag_pose_replay_node',
        name='bag_pose_replay_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )
    
    run_ros_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_file'),
            '--start-offset', LaunchConfiguration('start_offset'),
            '--rate', LaunchConfiguration('rate'),
            # You can manually pass topics/remaps here or use launch arguments later
        ],
        output='screen'
    )

    return LaunchDescription(
        ros_bag_args + 
        [
            bag_pose_pub_node,
            run_ros_bag
        ]
    )
