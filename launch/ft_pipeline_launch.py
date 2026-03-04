"""
Launch file for the FT Sensor Pipeline.

Usage:
    # Full pipeline with real sensor:
    ros2 launch ft_sensor_pipeline ft_pipeline_launch.py

    # With mock sensor (for testing without hardware):
    ros2 launch ft_sensor_pipeline ft_pipeline_launch.py use_mock:=true

    # With rosbag recording:
    ros2 launch ft_sensor_pipeline ft_pipeline_launch.py record_bag:=true

    # Override participant/trial:
    ros2 launch ft_sensor_pipeline ft_pipeline_launch.py participant_id:=P001 trial_id:=T01
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('ft_sensor_pipeline')
    config_file = os.path.join(pkg_share, 'config', 'ft_sensor_params.yaml')

    # --- Launch Arguments ---
    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='false',
        description='Use mock sensor instead of real hardware'
    )
    record_bag_arg = DeclareLaunchArgument(
        'record_bag', default_value='false',
        description='Record rosbag2 of all F/T topics'
    )
    participant_arg = DeclareLaunchArgument(
        'participant_id', default_value='P000',
        description='Participant ID for logging'
    )
    trial_arg = DeclareLaunchArgument(
        'trial_id', default_value='T00',
        description='Trial ID for logging'
    )
    condition_arg = DeclareLaunchArgument(
        'condition', default_value='with_robot',
        description='Experimental condition (with_robot / without_robot)'
    )
    visualize_arg = DeclareLaunchArgument(
        'visualize', default_value='true',
        description='Launch real-time visualizer'
    )

    # --- Sensor Driver (Real) ---
    sensor_driver_node = Node(
        package='ft_sensor_pipeline',
        executable='ft_sensor_driver_node.py',
        name='ft_sensor_driver',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('use_mock'), "' == 'false'"])
        ),
    )

    # --- Mock Sensor (for testing) ---
    mock_sensor_node = Node(
        package='ft_sensor_pipeline',
        executable='ft_mock_sensor_node.py',
        name='ft_mock_sensor',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mock')),
    )

    # --- Data Processor ---
    processor_node = Node(
        package='ft_sensor_pipeline',
        executable='ft_data_processor_node.py',
        name='ft_data_processor',
        parameters=[config_file],
        output='screen',
    )

    # --- Data Logger ---
    logger_node = Node(
        package='ft_sensor_pipeline',
        executable='ft_data_logger_node.py',
        name='ft_data_logger',
        parameters=[
            config_file,
            {'participant_id': LaunchConfiguration('participant_id')},
            {'trial_id': LaunchConfiguration('trial_id')},
            {'condition': LaunchConfiguration('condition')},
        ],
        output='screen',
    )

    # --- Visualizer ---
    visualizer_node = Node(
        package='ft_sensor_pipeline',
        executable='ft_visualizer_node.py',
        name='ft_visualizer',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualize')),
    )

    # --- Rosbag2 Recording ---
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/ft_sensor_data/rosbags/session_{timestamp}')

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/ft_sensor/raw',
            '/ft_sensor/filtered',
            '/ft_sensor/force_magnitude',
            '/ft_sensor/torque_magnitude',
            '/ft_sensor/grip_detected',
            '/ft_sensor/status',
            '-o', bag_dir,
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag')),
    )

    return LaunchDescription([
        # Arguments
        use_mock_arg,
        record_bag_arg,
        participant_arg,
        trial_arg,
        condition_arg,
        visualize_arg,

        # Nodes
        sensor_driver_node,
        mock_sensor_node,
        processor_node,
        logger_node,
        visualizer_node,
        rosbag_record,
    ])
