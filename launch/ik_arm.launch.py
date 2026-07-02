import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the IK Arm node with configurable parameters."""
    pkg_share = get_package_share_directory('ik_arm')
    default_config = os.path.join(pkg_share, 'config', 'arm_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to YAML config file with arm parameters'
        ),
        DeclareLaunchArgument(
            'joint_move_speed',
            default_value='0.08',
            description='Max joint speed (rad/s)'
        ),
        DeclareLaunchArgument(
            'control_rate',
            default_value='100.0',
            description='Control loop rate (Hz)'
        ),
        DeclareLaunchArgument(
            'position_tolerance',
            default_value='0.001',
            description='IK position tolerance (m)'
        ),
        DeclareLaunchArgument(
            'orientation_tolerance',
            default_value='0.01',
            description='IK orientation tolerance (rad)'
        ),
        Node(
            package='ik_arm',
            executable='ik_arm_node',
            name='ik_arm_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'joint_move_speed': LaunchConfiguration('joint_move_speed'),
                    'control_rate': LaunchConfiguration('control_rate'),
                    'position_tolerance': LaunchConfiguration('position_tolerance'),
                    'orientation_tolerance': LaunchConfiguration('orientation_tolerance'),
                },
            ],
        ),
    ])
