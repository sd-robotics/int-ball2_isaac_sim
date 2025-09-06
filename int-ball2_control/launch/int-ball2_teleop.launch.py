from launch import LaunchDescription 
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('int-ball2_control'), 'config', 'joystick.yaml')

    joy_node = Node(
            package = 'joy',
            executable = 'joy_node',
            parameters = [joy_params],
            output = 'screen'
    )

    intball2_vel_joy_node = Node(
            package = 'int-ball2_control',
            executable = 'direct_velocity_control.py',
            name = 'ib2_vel_ctrl',
            output = 'screen',
            parameters=[{
                'use_sim_time': True
            }]
    )
    
    intball2_effort_joy_node = Node(
            package = 'int-ball2_control',
            executable = 'direct_effort_control.py',
            name = 'ib2_eff_ctrl',
            output = 'screen',
            parameters=[{
                'use_sim_time': True
            }]
    )

    return LaunchDescription([
        joy_node,
        # intball2_vel_joy_node,
        intball2_effort_joy_node,
    ])
