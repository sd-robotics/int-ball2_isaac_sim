from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    main_args = [
        DeclareLaunchArgument(
            'isaac_path',
            default_value='',
            description='Isaac Sim installation root folder if not default'
        ),
        DeclareLaunchArgument(
            'issbag',
            default_value='True',
            description='If true, also launch bag_pose_replay.launch.py'
        ),
        # Optionally, add more DeclareLaunchArgument for bag_pose_replay if you want to pass them through
        DeclareLaunchArgument(
            'bag_file',
            default_value='',
            description='Bag file for replay'
        ),
        DeclareLaunchArgument(
            'start_offset',
            default_value='520.0',
            description='Bag start offset'
        ),
        DeclareLaunchArgument(
            'rate', 
            default_value='10.0',
            description='Bag playback rate'
        ),
    ]

    kibou_demo_usd = PathJoinSubstitution([
        FindPackageShare('int-ball2_isaac_sim'),
        'assets',
        'KIBOU_ISS_coordinate.usd'
    ])

    isaacsim_launch_file = PathJoinSubstitution([
        FindPackageShare('isaacsim'),
        'launch',
        'run_isaacsim.launch.py'
    ])

    isaacsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(isaacsim_launch_file),
        launch_arguments={
            'install_path': LaunchConfiguration('isaac_path'),
            'gui': kibou_demo_usd,
            'play_sim_on_start': 'True',
        }.items()
    )

    bag_pose_replay_launch_file = PathJoinSubstitution([
        FindPackageShare('ib2_data_replay'),
        'launch',
        'bag_pose_replay.launch.py'
    ])

    bag_pose_replay_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bag_pose_replay_launch_file),
        launch_arguments={
            'bag_file': LaunchConfiguration('bag_file'),
            'start_offset': LaunchConfiguration('start_offset'),
            'rate': LaunchConfiguration('rate'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('issbag'))
    )

    return LaunchDescription(main_args + [isaacsim_launch, bag_pose_replay_launch])