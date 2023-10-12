from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    simple_sim_movement_path = FindPackageShare('simple_sim_movement')
    urdf_file_name = 'urdf/lynxmotion_arm.urdf'
    default_rviz_config_path = PathJoinSubstitution(
        [simple_sim_movement_path, 'rviz', 'urdf.rviz'])
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(
        get_package_share_directory('simple_sim_movement'),
        urdf_file_name)

    config_file_name = 'rviz/urdf.rviz'

    rviz_config = os.path.join(
        get_package_share_directory('simple_sim_movement'),
        config_file_name)

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        Node(
            package="simple_sim_movement",
            executable="testing_node",
            name="custom_arm_node",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]),
    ])

    return ld
