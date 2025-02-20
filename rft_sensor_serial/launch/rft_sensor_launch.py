import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the directory of the current package
    package_dir = get_package_share_directory('rft_sensor_serial')
    
    # Full path to the YAML file
    param_file_path = os.path.join(package_dir, 'config', 'params.yaml')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['setserial /dev/ttyUSB0 low_latency'],
            shell=True
        ),
        # Declare the path to the YAML file as a launch argument
        DeclareLaunchArgument(
            'params_file',
            default_value=param_file_path,
            description='Path to the ROS2 parameters yaml file to use.'
        ),
        
        # Node to launch the RFTSensorSerial node
        Node(
            package='rft_sensor_serial',
            executable='rft_sensor_serial',
            name='rft_sensor_serial',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
