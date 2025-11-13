import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  # Get the share directory of the ctl_only and prop packages
  nav_share_dir = get_package_share_directory('ib2_nav')

  # Construct the full path to the YAML configuration files
  nav_yaml = os.path.join(nav_share_dir, 'config', 'sim.yaml')

  nav_node = Node(
      package='ib2_nav',
      executable='ib2_nav_node',
      name='nav',
      parameters=[nav_yaml,
        {
          'use_sim_time': True
        }
      ]
    )

  return LaunchDescription([nav_node])
