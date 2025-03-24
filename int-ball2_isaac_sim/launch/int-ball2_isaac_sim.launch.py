from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    main_args = [
        DeclareLaunchArgument('isaac_path', default_value='', description='Isaac Sim installation root folder if not default'),
        DeclareLaunchArgument('usd_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('int-ball2_isaac_sim'),
                'assets',
                'KIBOU.usd'
            ]), description='USD file path'),
        DeclareLaunchArgument('play_on_start', default_value='false', description='Play Isaac Sim after the scene is loaded'),
    ]

    isaacsim_launch_file = PathJoinSubstitution([
        FindPackageShare('isaacsim'),
        'launch',
        'run_isaacsim.launch.py'
    ])

    isaacsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(isaacsim_launch_file),
        launch_arguments={
            'install_path': LaunchConfiguration('isaac_path'),
            'gui': LaunchConfiguration('usd_file'),
            'play_sim_on_start': LaunchConfiguration('play_on_start'),
        }.items()
    )

    return LaunchDescription(main_args + [isaacsim_launch])
