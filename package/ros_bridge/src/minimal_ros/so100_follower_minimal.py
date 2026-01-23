#!/usr/bin/env python3
"""
Universal Follower Node - Robot Agnostic Implementation

This node can control ANY robot that implements the AbstractRobotController interface.
Supports SO100, Universal Robots, Franka, etc. through plugin architecture.

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
from messages import JointState, PoseStamped, RobotStatus, TeleopCommand, to_dict, from_dict
from robot_interface import create_robot_controller, AbstractRobotController

class UniversalFollowerNode:
    """
    Robot-agnostic follower node that can work with any robot type.
    
    Configuration determines which robot controller to load:
    - 'SO100' -> SO100RobotController
    - 'UR5' -> UniversalRobotController  
    - 'Franka' -> FrankaRobotController (future)
    """
    
    def __init__(self, robot_type: str = "SO100", robot_port: str = "COM4"):
        self.node = MinimalNode('universal_follower_node', base_port=5555)
        
        # Parameters
        self.robot_type = robot_type
        self.robot_port = robot_port
        self.feedback_rate = 10.0  # Hz - Further reduced to 10Hz for serial stability
        self.position_tolerance = 0.02  # 2cm tolerance for better dead-band
        
        # State variables - initialize first
        self.current_target_pose = None
        self.current_target_joints = None
        self.last_feedback_time = 0
        self.last_move_time = 0  # Add rate limiting for movements
        self.min_move_interval = 0.5  # Minimum 500ms between moves for reliability
        self.last_target_position = None  # Track last target position
        self.position_tolerance = 0.01  # 10mm tolerance - only move if change is significant
        self.consecutive_failures = 0  # Track consecutive communication failures
        self.max_failures_before_pause = 3  # Pause after fewer failures (reduced from 5)
        self.recovery_pause_time = 1.0  # Seconds to pause after failures (reduced from 2.0)
        self.feedback_interval = 1.0 / self.feedback_rate
        
        # Initialize robot controller
        try:
            self.robot_controller: AbstractRobotController = create_robot_controller(
                robot_type, 
                config_dir=os.path.join(package_root, "package", "drivers", "SO100_Robot", "config")
            )
            
            if not self.robot_controller.connect(robot_port, passive_mode=False):
                raise Exception(f"Failed to connect to {robot_type} robot on {robot_port}")
                
            capabilities = self.robot_controller.get_capabilities()
            self.node.get_logger().info(f'{robot_type} connected: {capabilities.joint_count} joints, gripper: {capabilities.has_gripper}')
            
        except Exception as e:
            self.node.get_logger().error(f'Robot initialization failed: {e}')
            return
        
        # Subscribers - Standard ROS2 compatible topics
        self.node.create_subscription(
            'robot_target_pose',                    # Standard topic name
            PoseStamped,
            self.target_pose_callback,
            10
        )
        
        self.node.create_subscription(
            'robot_target_joints',                  # For joint-space control
            JointState,
            self.target_joints_callback,
            10
        )
        
        self.node.create_subscription(
            'teleop_command',                       # Unified command interface
            TeleopCommand,
            self.teleop_command_callback,
            10
        )
        
        # Publishers - Standard ROS2 compatible topics  
        self.joint_state_pub = self.node.create_publisher(
            'joint_states',                         # Standard ROS2 topic
            JointState,
            10
        )
        
        self.current_pose_pub = self.node.create_publisher(
            'robot_current_pose',
            PoseStamped,
            10
        )
        
        self.robot_status_pub = self.node.create_publisher(
            'robot_status',
            RobotStatus, 
            10
        )
        
        self.node.get_logger().info(f'Universal Follower Node started for {robot_type}')
    
    def target_pose_callback(self, msg):
        """Receive target pose (cartesian control)"""
        current_time = time.time()
        print(f"[DEBUG] Follower: target_pose_callback called with msg: {msg}")
        # Rate limiting - skip if too soon since last move
        if current_time - self.last_move_time < self.min_move_interval:
            print("[DEBUG] Follower: Skipping due to min_move_interval")
            return
        
        # Skip if too many consecutive failures - give system time to recover
        if self.consecutive_failures >= self.max_failures_before_pause:
            if current_time - self.last_move_time < self.recovery_pause_time:
                print("[DEBUG] Follower: Skipping due to recovery_pause_time")
                return
            else:
                self.consecutive_failures = 0  # Reset failure count after pause
            
        self.current_target_pose = msg
        
        # Extract position and orientation
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation = [msg.pose.orientation.x, msg.pose.orientation.y, 
                      msg.pose.orientation.z, msg.pose.orientation.w]
        
        # Dead-band check - only move if position changed significantly
        if self.last_target_position is not None:
            pos_diff = sum(abs(a - b) for a, b in zip(position[:3], self.last_target_position[:3]))
            if pos_diff < self.position_tolerance:
                print(f"[DEBUG] Follower: Skipping due to dead-band (diff={pos_diff})")
                return  # Skip if position change is too small
        
        # Rate limiting - ensure minimum time between moves (use configured value)
        if current_time - self.last_move_time < self.min_move_interval:
            print("[DEBUG] Follower: Skipping due to min_move_interval (2)")
            return
        
        self.last_target_position = position[:3]  # Store for next comparison
        
        # Apply safety limits
        position[2] = max(0.02, position[2])  # Table protection
        position[0] = np.clip(position[0], -0.35, 0.35)
        position[1] = np.clip(position[1], -0.35, 0.35)
        
        # Move robot with much slower timing for better reliability
        print(f"[DEBUG] Follower: Calling move_to_cartesian with pos={position}, orientation={orientation}")
        try:
            # Use slower move times to reduce command rate and improve motor reliability
            success = self.robot_controller.move_to_cartesian(position, orientation, time_ms=2500)
            print(f"[DEBUG] Follower: move_to_cartesian returned {success}")
            if not success:
                self.node.get_logger().warn(f'Cartesian move failed')
                self.consecutive_failures += 1
            else:
                self.consecutive_failures = 0  # Reset on success
            self.last_move_time = current_time  # Update last move time
        except Exception as e:
            print(f"[DEBUG] Follower: Exception in move_to_cartesian: {e}")
            self.node.get_logger().error(f'Cartesian movement error: {e}')
            self.consecutive_failures += 1
    
    def target_joints_callback(self, msg):
        """Receive target joints (joint-space control)"""
        print(f"[DEBUG] Follower: target_joints_callback called with msg: {msg}")
        self.current_target_joints = msg
        # Only send joint moves if last sent more than min_move_interval ago
        current_time = time.time()
        if current_time - self.last_move_time < self.min_move_interval:
            print("[DEBUG] Follower: Skipping joint command due to min_move_interval")
            return

        try:
            success = self.robot_controller.move_to_joints(msg.position, time_ms=500)
            print(f"[DEBUG] Follower: move_to_joints returned {success}")
            if not success:
                self.node.get_logger().warn(f'Joint move failed')
            else:
                self.last_move_time = current_time
        except Exception as e:
            print(f"[DEBUG] Follower: Exception in move_to_joints: {e}")
            self.node.get_logger().error(f'Joint movement error: {e}')
    
    def teleop_command_callback(self, msg):
        """Unified teleoperation command interface"""
        try:
            if msg.command_type == "pose" and msg.target_pose:
                self.target_pose_callback(msg.target_pose)
            elif msg.command_type == "joints" and msg.target_joints:
                self.target_joints_callback(msg.target_joints)
            elif msg.command_type == "stop":
                self.robot_controller.emergency_stop()
            
            # Handle gripper commands
            if msg.gripper_command >= 0.0 and self.robot_controller.has_gripper():
                if msg.gripper_command > 0.5:
                    self.robot_controller.gripper_open()
                else:
                    self.robot_controller.gripper_close()
                    
        except Exception as e:
            self.node.get_logger().error(f'Teleop command error: {e}')
    
    def spin(self):
        """Main loop"""
        try:
            while True:
                current_time = time.time()
                
                # Publish feedback at specified rate
                if current_time - self.last_feedback_time >= self.feedback_interval:
                    self.publish_feedback()
                    self.last_feedback_time = current_time
                
                time.sleep(0.001)  # Small sleep
                
        except KeyboardInterrupt:
            self.node.get_logger().info("Shutting down...")
            self.destroy()
    
    def publish_feedback(self):
        """Publish current robot state for feedback"""
        try:
            # Get joint states from robot
            joint_names, positions, velocities, efforts = self.robot_controller.get_joint_states()
            
            if joint_names:
                # Publish joint states (standard ROS2 format)
                joint_msg = JointState.create(joint_names, positions, velocities, efforts)
                self.joint_state_pub.publish(joint_msg)
                
                # Get and publish current pose
                position, orientation = self.robot_controller.get_cartesian_pose()
                pose_msg = PoseStamped.create(position, orientation)
                self.current_pose_pub.publish(pose_msg)
                
                # Publish robot status
                status_msg = RobotStatus(
                    header=joint_msg.header,
                    robot_name=self.robot_type,
                    is_connected=True,
                    is_enabled=True,
                    error_message="",
                    position_error=self.calculate_position_error(),
                    joint_error=self.calculate_joint_error()
                )
                self.robot_status_pub.publish(status_msg)
                
                # Simple status output
                print(f"\r{self.robot_type}: Pos({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
                     f"Err: {status_msg.position_error*1000:.1f}mm", end="")
                
        except Exception as e:
            self.node.get_logger().error(f'Feedback error: {e}')
            
            # Publish error status
            error_status = RobotStatus(
                header={"stamp": time.time(), "frame_id": "base_link"},
                robot_name=self.robot_type,
                is_connected=False,
                is_enabled=False,
                error_message=str(e)
            )
            self.robot_status_pub.publish(error_status)
    
    def calculate_position_error(self):
        """Calculate position tracking error"""
        if not self.current_target_pose:
            return 0.0
        
        try:
            current_pos, _ = self.robot_controller.get_cartesian_pose()
            target_pos = [
                self.current_target_pose.pose.position.x,
                self.current_target_pose.pose.position.y,
                self.current_target_pose.pose.position.z
            ]
            return np.linalg.norm(np.array(current_pos) - np.array(target_pos))
        except:
            return 0.0
    
    def calculate_joint_error(self):
        """Calculate joint tracking error"""
        if not self.current_target_joints:
            return 0.0
        
        try:
            _, current_joints, _, _ = self.robot_controller.get_joint_states()
            target_joints = self.current_target_joints.position
            if len(current_joints) == len(target_joints):
                return max(abs(c - t) for c, t in zip(current_joints, target_joints))
        except:
            pass
        return 0.0
    
    def destroy(self):
        """Clean shutdown"""
        if hasattr(self, 'robot_controller'):
            self.robot_controller.disconnect()
        self.node.destroy_node()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Universal Robot Follower Node')
    parser.add_argument('--robot_type', default='SO100', choices=['SO100', 'UR5', 'UR10'], 
                       help='Type of robot to control')
    parser.add_argument('--robot_port', default='COM4', help='Robot connection port')
    
    args = parser.parse_args()
    
    node = UniversalFollowerNode(args.robot_type, args.robot_port)
    node.spin()

if __name__ == '__main__':
    main()