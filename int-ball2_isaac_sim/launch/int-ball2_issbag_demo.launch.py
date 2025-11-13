from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    main_args = [
        DeclareLaunchArgument(
            'issbag',
            default_value='True',
            description='If true, also launch bag_pose_replay.launch.py'
        ),
        DeclareLaunchArgument(
            'bag_file',
            description='Path to the ROS 2 bag file to play'
        ),
    ]

    kibou_demo_usd = PathJoinSubstitution([
        FindPackageShare('ib2_isaac_sim'),
        'assets',
        'KIBOU_ISS_coordinate.usd'
    ])

    int_ball2_isaac_sim_launch_file = PathJoinSubstitution([
        FindPackageShare('ib2_isaac_sim'),
        'launch',
        'int-ball2_isaac_sim.launch.py'
    ])

    bag_pose_replay_launch_file = PathJoinSubstitution([
        FindPackageShare('ib2_data_replay'),
        'launch',
        'bag_pose_replay.launch.py'
    ])

    isaacsim_main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(int_ball2_isaac_sim_launch_file),
        launch_arguments={
            'isaac_sim_version': '4.5.0',
            'usd_file': kibou_demo_usd,
            'play_on_start': 'True',
        }.items()
    )

    bag_pose_replay_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bag_pose_replay_launch_file),
        launch_arguments={
            'bag_file': LaunchConfiguration('bag_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('issbag'))
    )

    return LaunchDescription(main_args + [isaacsim_main_launch, bag_pose_replay_launch])
