from launch import LaunchDescription 
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('intball_isaac'), 'config', 'joystick.yaml')

    joy_node = Node(
            package = 'joy',
            executable = 'joy_node',
            parameters = [joy_params],
            output = 'screen'
    )

    intball_vel_joy_node = Node(
            package = 'intball_isaac',
            executable = 'direct_velocity_control.py',
            name = 'intball_velocity_controller_node',
            output = 'screen'
    )
    
    intball_force_joy_node = Node(
            package = 'intball_isaac',
            executable = 'direct_force_control.py',
            name = 'intball_force_controller_node',
            output = 'screen'
    )
    
#     intball_feedbacks_joy_node = Node(
#             package = 'intball_isaac',
#             executable = 'feedbacks_control.py',
#             name = 'intball_feedbacks_controller_node',
#             output = 'screen'
#     )


    return LaunchDescription([
        joy_node,
        # intball_vel_joy_node,
        intball_force_joy_node,
    ])