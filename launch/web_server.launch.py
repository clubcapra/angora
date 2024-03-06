from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='angora',
            executable='client_angora',
            name='client_angora',
            output='screen',
        ),
    ])
