#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for SO100 teleoperation system
    
    This launches:
    1. SO100 Leader Node - reads leader robot and publishes pose
    2. SO100 Follower Node - controls follower robot
    3. Teleoperation Coordinator - monitors and ensures safety
    """
    
    # Launch arguments
    leader_port_arg = DeclareLaunchArgument(
        'leader_port',
        default_value='COM5',
        description='Serial port for leader robot'
    )
    
    follower_port_arg = DeclareLaunchArgument(
        'follower_port', 
        default_value='COM4',
        description='Serial port for follower robot'
    )
    
    # Nodes
    leader_node = Node(
        package='so100_teleoperation',
        executable='so100_leader_node.py',
        name='so100_leader_node',
        parameters=[{
            'leader_port': LaunchConfiguration('leader_port'),
            'publish_rate': 30.0
        }],
        output='screen'
    )
    
    follower_node = Node(
        package='so100_teleoperation',
        executable='so100_follower_node.py', 
        name='so100_follower_node',
        parameters=[{
            'follower_port': LaunchConfiguration('follower_port'),
            'feedback_rate': 50.0,
            'position_tolerance': 0.01
        }],
        output='screen'
    )
    
    coordinator_node = Node(
        package='so100_teleoperation',
        executable='so100_teleop_coordinator.py',
        name='so100_teleop_coordinator',
        parameters=[{
            'max_position_error': 0.05,      # 5cm
            'max_joint_error': 0.5,          # ~30 degrees  
            'emergency_stop_error': 0.15     # 15cm
        }],
        output='screen'
    )
    
    return LaunchDescription([
        leader_port_arg,
        follower_port_arg,
        leader_node,
        follower_node,
        coordinator_node
    ])