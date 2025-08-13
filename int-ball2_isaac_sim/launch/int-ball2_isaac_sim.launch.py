from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    main_args = [
        DeclareLaunchArgument(
            'isaac_sim_version',
            default_value='4.5.0',
            description='Isaac Sim version to use, e.g., 4.5.0'
        ),
        DeclareLaunchArgument(
            'isaac_path',
            default_value='',
            description='Isaac Sim installation root folder if not default'
        ),
        DeclareLaunchArgument(
            'usd_file',
            default_value=TextSubstitution(text='KIBOU.usd'),
            description='USD file name inside assets directory'
        ),
        DeclareLaunchArgument(
            'play_on_start',
            default_value='false',
            description='Play Isaac Sim after the scene is loaded'
        ),
        # Forwarded args to upstream run_isaacsim.launch.py to ensure correct ROS setup
        DeclareLaunchArgument(
            'use_internal_libs',
            default_value='false',
            description='Use ROS libraries shipped with Isaac Sim to avoid host ROS conflicts'
        ),
        DeclareLaunchArgument(
            'dds_type',
            default_value='fastdds',
            description='DDS implementation for Isaac Sim ROS bridge (fastdds or cyclonedds)'
        ),
        DeclareLaunchArgument(
            'ros_distro',
            default_value='humble',
            description='ROS distribution to use inside Isaac Sim'
        ),
        DeclareLaunchArgument(
            'exclude_install_path',
            default_value='',
            description='Comma-separated install paths to exclude from Isaac Sim environment (e.g., /opt/ros/humble,~/ws/install)'
        ),
    ]

    asset_path = PathJoinSubstitution([
        FindPackageShare('ib2_isaac_sim'),
        'assets',
        LaunchConfiguration('usd_file')
    ])

    isaacsim_launch_file = PathJoinSubstitution([
        FindPackageShare('ib2_isaac_sim'),
        'launch',
        'run_isaacsim.launch.py'
    ])

    isaacsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(isaacsim_launch_file),
        launch_arguments={
            'version': LaunchConfiguration('isaac_sim_version'),
            'install_path': LaunchConfiguration('isaac_path'),
            # 'gui': asset_path,
            'play_sim_on_start': LaunchConfiguration('play_on_start'),
            'use_internal_libs': LaunchConfiguration('use_internal_libs'),
            'dds_type': LaunchConfiguration('dds_type'),
            'ros_distro': LaunchConfiguration('ros_distro'),
        }.items()
    )

    return LaunchDescription(main_args + [isaacsim_launch])
