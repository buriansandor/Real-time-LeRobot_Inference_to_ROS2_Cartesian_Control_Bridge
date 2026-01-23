#!/usr/bin/env python3
"""
Universal Leader Node - Robot Agnostic Implementation

This node can read from ANY robot that implements the AbstractRobotController interface
and publishes standard ROS2 compatible topics that any follower can understand.

Created by Sandor Burian
"""

import sys
import os
import time
import numpy as np
from pathlib import Path

# Add project paths  
script_dir = Path(__file__).parent
bridge_root = script_dir.parent
package_root = bridge_root.parent.parent.parent  # Go up one more level to find the package
sys.path.insert(0, str(package_root))
sys.path.insert(0, str(script_dir))

from minimal_ros import MinimalNode
from messages import JointState, PoseStamped, RobotStatus, TeleopCommand
from robot_interface import create_robot_controller, AbstractRobotController

class UniversalLeaderNode:
    """
    Robot-agnostic leader node that publishes standard ROS2 topics.
    
    Any robot type can be the leader:
    - SO100 
    - Universal Robots (UR5, UR10)
    - Franka Emika
    - etc.
    """
    
    def __init__(self, robot_type: str = "SO100", robot_port: str = "COM3"):
        self.node = MinimalNode('universal_leader_node', base_port=5556)
        
        # Parameters
        self.robot_type = robot_type
        self.robot_port = robot_port
        self.publish_rate = 30.0  # Hz - teleoperation frequency
        
        # Initialize state tracking variables first
        self.last_pose = None
        self.last_joints = None
        self.last_gripper_state = None
        self.publish_interval = 1.0 / self.publish_rate
        self.last_publish_time = 0
        
        # Initialize robot controller
        try:
            self.robot_controller: AbstractRobotController = create_robot_controller(
                robot_type,
                config_dir=os.path.join(package_root, "package", "drivers", "SO100_Robot", "config")
            )
            
            if not self.robot_controller.connect(robot_port, passive_mode=True):
                raise Exception(f"Failed to connect to {robot_type} robot on {robot_port}")
            
            # Set to passive mode - leader is for reading only (can be moved by hand)
            self.robot_controller.set_passive_mode(True)
                
            capabilities = self.robot_controller.get_capabilities()
            self.node.get_logger().info(f'{robot_type} leader connected: {capabilities.joint_count} joints')
            
        except Exception as e:
            self.node.get_logger().error(f'Robot initialization failed: {e}')
            return
        
        # Publishers - Standard ROS2 compatible topics
        self.target_pose_pub = self.node.create_publisher(
            'robot_target_pose',                    # Universal topic - any follower can subscribe
            PoseStamped,
            10
        )
        
        self.target_joints_pub = self.node.create_publisher(
            'robot_target_joints',                  # For joint-space followers
            JointState,
            10
        )
        
        self.teleop_command_pub = self.node.create_publisher(
            'teleop_command',                       # Unified command interface
            TeleopCommand,
            10
        )
        
        self.leader_status_pub = self.node.create_publisher(
            'leader_status',
            RobotStatus,
            10
        )
        
        self.node.get_logger().info(f'Universal Leader Node started for {robot_type}')
        self.node.get_logger().info('Publishing standard ROS2 topics:')
        self.node.get_logger().info('  - robot_target_pose (PoseStamped)')
        self.node.get_logger().info('  - robot_target_joints (JointState)')
        self.node.get_logger().info('  - teleop_command (TeleopCommand)')
    
    def spin(self):
        """Main loop - read robot and publish commands"""
        try:
            while True:
                current_time = time.time()
                
                if current_time - self.last_publish_time >= self.publish_interval:
                    self.publish_robot_state()
                    self.last_publish_time = current_time
                
                time.sleep(0.001)  # Small sleep
                
        except KeyboardInterrupt:
            self.node.get_logger().info("Shutting down...")
            self.destroy()
    
    def publish_robot_state(self):
        """Read robot state and publish as commands for followers"""
        try:
            print("[DEBUG] Leader: Reading robot state...")  # Debug message
            # Get current robot state
            position, orientation = self.robot_controller.get_cartesian_pose()
            joint_names, joint_positions, joint_velocities, joint_efforts = self.robot_controller.get_joint_states()
            
            print(f"[DEBUG] Leader: Got position {position}, joints {joint_positions}")  # Debug message
            current_time = time.time()
            
            # 1. Publish cartesian target pose
            pose_msg = PoseStamped.create(position, orientation)
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "base_link"
            self.target_pose_pub.publish(pose_msg)
            
            # 2. Publish joint target positions
            joint_msg = JointState.create(joint_names, joint_positions, joint_velocities, joint_efforts)
            joint_msg.header.stamp = current_time
            joint_msg.header.frame_id = "base_link"
            self.target_joints_pub.publish(joint_msg)
            
            # 3. Publish unified teleop command
            teleop_cmd = TeleopCommand(
                header={"stamp": current_time, "frame_id": "base_link"},
                command_type="pose",  # Default to pose control
                target_pose=pose_msg,
                target_joints=joint_msg,
                gripper_command=self.get_gripper_command()
            )
            self.teleop_command_pub.publish(teleop_cmd)
            
            # 4. Publish leader status
            status_msg = RobotStatus(
                header={"stamp": current_time, "frame_id": "base_link"},
                robot_name=f"{self.robot_type}_leader",
                is_connected=True,
                is_enabled=True,
                error_message="",
                position_error=0.0,
                joint_error=0.0
            )
            self.leader_status_pub.publish(status_msg)
            
            # Simple status output
            print(f"\r{self.robot_type} Leader: Pos({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
                 f"Joints: [{', '.join(f'{j:.1f}' for j in joint_positions[:3])}...]", end="")
            
        except Exception as e:
            self.node.get_logger().error(f'Publishing error: {e}')
            
            # Publish error status
            error_status = RobotStatus(
                header={"stamp": time.time(), "frame_id": "base_link"},
                robot_name=f"{self.robot_type}_leader",
                is_connected=False,
                is_enabled=False,
                error_message=str(e)
            )
            self.leader_status_pub.publish(error_status)
    
    def get_gripper_command(self):
        """Get gripper state as command value"""
        try:
            if self.robot_controller.has_gripper():
                # Simple gripper state detection
                if hasattr(self.robot_controller, 'get_gripper_state'):
                    gripper_value = self.robot_controller.get_gripper_state()
                    return 1.0 if gripper_value > 0.5 else 0.0  # Normalize to 0-1
                else:
                    # Try to get from joint states (last joint usually gripper)
                    _, joint_positions, _, _ = self.robot_controller.get_joint_states()
                    if joint_positions:
                        # Normalize gripper position to 0-1 range
                        gripper_raw = joint_positions[-1]
                        return max(0.0, min(1.0, (gripper_raw - 1500) / 1000))  # SO100 specific
        except:
            pass
        return -1.0  # No gripper command
    
    def destroy(self):
        """Clean shutdown"""
        if hasattr(self, 'robot_controller'):
            self.robot_controller.disconnect()
        self.node.destroy_node()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Universal Robot Leader Node')
    parser.add_argument('--robot_type', default='SO100', choices=['SO100', 'UR5', 'UR10'],
                       help='Type of robot to read from')
    parser.add_argument('--robot_port', default='COM3', help='Robot connection port')
    
    args = parser.parse_args()
    
    node = UniversalLeaderNode(args.robot_type, args.robot_port)
    node.spin()

if __name__ == '__main__':
    main()