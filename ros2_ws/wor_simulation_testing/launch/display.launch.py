import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    urdf_file_name = LaunchConfiguration('urdf_file_name', default_value='lynxmotion_arm.urdf')
    
    # Get the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('your_package_name'), 'your_urdf_folder', urdf_file_name)
    
    return LaunchDescription([
        # Launch the robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file_path}],
        ),
    ])
