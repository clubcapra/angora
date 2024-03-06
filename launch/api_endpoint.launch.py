from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_dir = get_package_share_directory('angora')

    endpoint_script_path = os.path.join(package_dir, 'src', 'api_endpoint.py')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', endpoint_script_path],
            output='screen',
            shell=True
        ),
    ])