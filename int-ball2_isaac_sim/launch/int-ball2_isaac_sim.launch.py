from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


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
            default_value=TextSubstitution(text=''),
            description='USD file name inside assets directory'
        ),
        DeclareLaunchArgument(
            'play_sim_on_start',
            default_value='false',
            description='Play Isaac Sim after the scene is loaded'
        ),
        # Publish an initial transform twist once at startup
        DeclareLaunchArgument(
            'publish_initial_twist',
            default_value='true',
            description='Publish a one-shot initial Twist on /transform_twist at startup'
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

    isaacsim_launch_file = PathJoinSubstitution([
        FindPackageShare('ib2_isaac_sim'),
        'launch',
        'run_isaacsim.launch.py'
    ])

    def launch_setup(context):
        usd_file_value = LaunchConfiguration('usd_file').perform(context)
        # Build asset path only if user provided a file name; else pass empty string
        if usd_file_value:
            assets_dir = os.path.join(get_package_share_directory('ib2_isaac_sim'), 'assets')
            gui_arg = os.path.join(assets_dir, usd_file_value)
        else:
            gui_arg = ''

        isaacsim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(isaacsim_launch_file),
            launch_arguments={
                'version': LaunchConfiguration('isaac_sim_version'),
                'install_path': LaunchConfiguration('isaac_path'),
                'gui': TextSubstitution(text=gui_arg),
                'play_sim_on_start': LaunchConfiguration('play_sim_on_start'),
                'use_internal_libs': LaunchConfiguration('use_internal_libs'),
                'dds_type': LaunchConfiguration('dds_type'),
                'ros_distro': LaunchConfiguration('ros_distro'),
            }.items()
        )
        return [isaacsim_launch]

    # One-shot initial Twist publish (docking position)
    # Note: Use a small delay.
    initial_twist_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/transform_twist',
            'geometry_msgs/msg/Twist',
            '{linear: {x: 10.88492, y: -3.53022, z: 4.07888}, '
            ' angular: {x: 180.0, y: 0.32, z: -90.0}}'
        ],
        shell=False,
        output='log',
        condition=IfCondition(LaunchConfiguration('publish_initial_twist')),
    )

    publish_initial_twist_after_delay = TimerAction(
        period=13.0,  # seconds
        actions=[initial_twist_cmd],
        condition=IfCondition(LaunchConfiguration('publish_initial_twist')),
    )

    return LaunchDescription(main_args + [OpaqueFunction(function=launch_setup), publish_initial_twist_after_delay])
