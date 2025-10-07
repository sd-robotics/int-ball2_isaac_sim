from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    ]

    asset_path = PathJoinSubstitution([
        FindPackageShare('ib2_isaac_sim'),
        'assets',
        LaunchConfiguration('usd_file')
    ])

    isaacsim_launch_file = PathJoinSubstitution([
        FindPackageShare('isaacsim'),
        'launch',
        'run_isaacsim.launch.py'
    ])

    isaacsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(isaacsim_launch_file),
        launch_arguments={
            'version': LaunchConfiguration('isaac_sim_version'),
            'install_path': LaunchConfiguration('isaac_path'),
            'gui': asset_path,
            'play_sim_on_start': LaunchConfiguration('play_on_start'),
        }.items()
    )

    return LaunchDescription(main_args + [isaacsim_launch])
