#!/usr/bin/env python3
"""
SO100 Follower Robot Node

This node controls the follower robot by subscribing to target poses
and publishing actual joint states for feedback. Integrates with MoveIt!
for collision detection and smooth path planning.

Created by Sandor Burian
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveGroupGoal, Constraints, JointConstraint
import sys
import os
from pathlib import Path
import numpy as np

# Add the SO100 driver path
script_dir = Path(__file__).parent
package_root = script_dir.parent.parent.parent.parent  # Go back to main project root
sys.path.insert(0, str(package_root))

from package.drivers.SO100_Robot.so100_core import SO100Robot as FollowerRobot

class SO100FollowerNode(Node):
    def __init__(self):
        super().__init__('so100_follower_node')
        
        # Parameters
        self.declare_parameter('follower_port', 'COM4')
        self.declare_parameter('feedback_rate', 50.0)  # 50 Hz feedback
        self.declare_parameter('position_tolerance', 0.01)  # 1cm tolerance
        
        follower_port = self.get_parameter('follower_port').value
        feedback_rate = self.get_parameter('feedback_rate').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/so100/leader/target_pose',
            self.target_pose_callback,
            10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/so100/follower/joint_states',
            10
        )
        
        self.feedback_pub = self.create_publisher(
            PoseStamped,
            '/so100/follower/current_pose',
            10
        )
        
        # Action clients (for future MoveIt integration)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Initialize robot
        try:
            config_path = os.path.join(package_root, "package", "drivers", "SO100_Robot", "config")
            self.robot = FollowerRobot(port=follower_port, config_dir=config_path)
            self.robot.torque_enable(True)  # Active mode
            self.get_logger().info(f'SO100 Follower connected to {follower_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to follower: {e}')
            return
        
        # Timer for feedback publishing
        self.feedback_timer = self.create_timer(1.0 / feedback_rate, self.publish_feedback)
        
        # State variables
        self.current_target = None
        self.last_move_time = 0
        
        self.get_logger().info('SO100 Follower Node started')
    
    def target_pose_callback(self, msg):
        """Receive target pose from leader"""
        self.current_target = msg
        
        # Extract position
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        target_z = msg.pose.position.z
        
        # Safety limits
        target_z = max(0.02, target_z)  # Table protection
        target_x = np.clip(target_x, -0.35, 0.35)
        target_y = np.clip(target_y, -0.35, 0.35)
        
        # Move robot (direct control for now, MoveIt later)
        try:
            self.robot.move_to_cartesian(target_x, target_y, target_z, time_ms=50)
            self.last_move_time = self.get_clock().now().nanoseconds / 1e9
        except Exception as e:
            self.get_logger().error(f'Movement error: {e}')
    
    def publish_feedback(self):
        """Publish current robot state for feedback"""
        try:
            # Get joint angles
            joint_angles = self.robot.get_joint_angles()
            
            # Publish joint states
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = "so100_base_link"
            joint_msg.name = [
                'joint_1', 'joint_2', 'joint_3', 
                'joint_4', 'joint_5', 'gripper_joint'
            ]
            joint_msg.position = joint_angles
            joint_msg.velocity = [0.0] * len(joint_angles)
            joint_msg.effort = [0.0] * len(joint_angles)
            
            self.joint_state_pub.publish(joint_msg)
            
            # Publish current cartesian position
            try:
                current_xyz = self.robot.get_cartesian_position()
                
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "so100_base_link"
                pose_msg.pose.position.x = current_xyz[0]
                pose_msg.pose.position.y = current_xyz[1]
                pose_msg.pose.position.z = current_xyz[2]
                
                # Simple orientation
                pose_msg.pose.orientation.w = 1.0
                
                self.feedback_pub.publish(pose_msg)
                
                # Check position error if we have a target
                if self.current_target:
                    error = np.linalg.norm([
                        current_xyz[0] - self.current_target.pose.position.x,
                        current_xyz[1] - self.current_target.pose.position.y,
                        current_xyz[2] - self.current_target.pose.position.z
                    ])
                    
                    if error > self.position_tolerance:
                        self.get_logger().warn(f'Position error: {error*1000:.1f}mm')
                
            except Exception as e:
                self.get_logger().error(f'Cartesian position error: {e}')
                
        except Exception as e:
            self.get_logger().error(f'Feedback error: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'robot'):
            self.robot.torque_enable(False)
            self.robot.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = SO100FollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()