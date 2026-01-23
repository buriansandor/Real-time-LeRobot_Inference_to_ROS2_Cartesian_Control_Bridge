#!/usr/bin/env python3
"""
SO100 Leader Robot Node

This node reads the leader robot's joint positions and publishes them as ROS2 topics.
It provides clean joint state feedback and cartesian pose for the follower to track.

Created by Sandor Burian
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import sys
import os
from pathlib import Path

# Add the SO100 driver path
script_dir = Path(__file__).parent
package_root = script_dir.parent.parent.parent.parent  # Go back to main project root
sys.path.insert(0, str(package_root))

from package.drivers.SO100_Robot.leader_robot import SO100LeaderToCartesianControl as SO100Leader

class SO100LeaderNode(Node):
    def __init__(self):
        super().__init__('so100_leader_node')
        
        # Parameters
        self.declare_parameter('leader_port', 'COM5')
        self.declare_parameter('publish_rate', 30.0)  # 30 Hz
        
        leader_port = self.get_parameter('leader_port').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 
            '/so100/leader/joint_states', 
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            '/so100/leader/target_pose', 
            10
        )
        
        # Initialize robot
        try:
            config_path = os.path.join(package_root, "package", "drivers", "SO100_Robot", "config")
            self.robot = SO100Leader(port=leader_port)
            self.robot.torque_disable()  # Passive mode for manual movement
            self.get_logger().info(f'SO100 Leader connected to {leader_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to leader: {e}')
            return
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_robot_state)
        
        self.get_logger().info('SO100 Leader Node started')
    
    def publish_robot_state(self):
        """Publish current robot state"""
        try:
            # Get joint angles
            joint_angles = self.robot.get_joint_angles()
            
            # Get cartesian pose
            target_xyz, gripper_state = self.robot.get_cartesian_pose()
            
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
            
            # Publish target pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "so100_base_link"
            pose_msg.pose.position.x = target_xyz[0]
            pose_msg.pose.position.y = target_xyz[1] 
            pose_msg.pose.position.z = target_xyz[2]
            
            # Simple orientation (pointing down)
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            
            self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error reading robot state: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'robot'):
            self.robot.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = SO100LeaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()