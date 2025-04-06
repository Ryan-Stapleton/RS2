import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([  
        # Start the test_trackit_jacobian node and set robot_description parameter
        Node(
            package='trackit_jacobian',
            executable='trackit_jacobian',
            name='trackit_jacobian',
            output='screen',
        ),

        # Start the trackit_jacobian node and set robot_description parameter
        Node(
            package='trackit_jacobian',
            executable='test_trackit_jacobian',
            name='test_trackit_jacobian',
            output='screen',
        ),
    ])