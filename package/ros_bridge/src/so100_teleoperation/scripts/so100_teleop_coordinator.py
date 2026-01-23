#!/usr/bin/env python3
"""
SO100 Teleoperation Coordinator

This node coordinates the leader-follower teleoperation by:
1. Monitoring position errors between leader and follower
2. Providing safety oversight and emergency stop
3. Logging performance metrics
4. Managing gripper coordination

Created by Sandor Burian
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
import time

class SO100TeleopCoordinator(Node):
    def __init__(self):
        super().__init__('so100_teleop_coordinator')
        
        # Parameters
        self.declare_parameter('max_position_error', 0.05)  # 5cm max error
        self.declare_parameter('max_joint_error', 0.5)      # ~30 degrees max joint error
        self.declare_parameter('emergency_stop_error', 0.15) # 15cm emergency stop
        
        self.max_pos_error = self.get_parameter('max_position_error').value
        self.max_joint_error = self.get_parameter('max_joint_error').value
        self.emergency_threshold = self.get_parameter('emergency_stop_error').value
        
        # Subscribers
        self.leader_pose_sub = self.create_subscription(
            PoseStamped,
            '/so100/leader/target_pose',
            self.leader_pose_callback,
            10
        )
        
        self.follower_pose_sub = self.create_subscription(
            PoseStamped,
            '/so100/follower/current_pose',
            self.follower_pose_callback,
            10
        )
        
        self.leader_joints_sub = self.create_subscription(
            JointState,
            '/so100/leader/joint_states',
            self.leader_joints_callback,
            10
        )
        
        self.follower_joints_sub = self.create_subscription(
            JointState,
            '/so100/follower/joint_states',
            self.follower_joints_callback,
            10
        )
        
        # Publishers
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/so100/emergency_stop',
            10
        )
        
        # State variables
        self.leader_pose = None
        self.follower_pose = None
        self.leader_joints = None
        self.follower_joints = None
        
        # Performance tracking
        self.position_errors = []
        self.joint_errors = []
        self.last_warning_time = 0
        
        # Timer for coordination monitoring
        self.monitor_timer = self.create_timer(0.1, self.monitor_coordination)  # 10 Hz
        
        self.get_logger().info('SO100 Teleoperation Coordinator started')
    
    def leader_pose_callback(self, msg):
        """Receive leader target pose"""
        self.leader_pose = msg
    
    def follower_pose_callback(self, msg):
        """Receive follower current pose"""
        self.follower_pose = msg
    
    def leader_joints_callback(self, msg):
        """Receive leader joint states"""
        self.leader_joints = msg
    
    def follower_joints_callback(self, msg):
        """Receive follower joint states"""
        self.follower_joints = msg
    
    def monitor_coordination(self):
        """Monitor coordination between leader and follower"""
        current_time = time.time()
        
        # Check position error
        if self.leader_pose and self.follower_pose:
            pos_error = self.calculate_position_error()
            self.position_errors.append(pos_error)
            
            # Keep only last 100 measurements
            if len(self.position_errors) > 100:
                self.position_errors.pop(0)
            
            # Emergency stop check
            if pos_error > self.emergency_threshold:
                self.get_logger().error(f'EMERGENCY STOP: Position error {pos_error*1000:.1f}mm > {self.emergency_threshold*1000:.1f}mm')
                emergency_msg = Bool()
                emergency_msg.data = True
                self.emergency_stop_pub.publish(emergency_msg)
                return
            
            # Warning for large errors
            if pos_error > self.max_pos_error and current_time - self.last_warning_time > 2.0:
                avg_error = np.mean(self.position_errors[-10:]) * 1000  # Last 10 measurements in mm
                self.get_logger().warn(f'Position tracking error: {pos_error*1000:.1f}mm (avg: {avg_error:.1f}mm)')
                self.last_warning_time = current_time
        
        # Check joint errors
        if self.leader_joints and self.follower_joints:
            joint_error = self.calculate_joint_error()
            self.joint_errors.append(joint_error)
            
            # Keep only last 100 measurements
            if len(self.joint_errors) > 100:
                self.joint_errors.pop(0)
            
            # Warning for large joint errors
            if joint_error > self.max_joint_error and current_time - self.last_warning_time > 2.0:
                avg_joint_error = np.mean(self.joint_errors[-10:]) * 57.3  # Convert to degrees
                self.get_logger().warn(f'Joint tracking error: {joint_error*57.3:.1f}° (avg: {avg_joint_error:.1f}°)')
                self.last_warning_time = current_time
    
    def calculate_position_error(self):
        """Calculate position error between leader and follower"""
        leader_pos = [
            self.leader_pose.pose.position.x,
            self.leader_pose.pose.position.y,
            self.leader_pose.pose.position.z
        ]
        
        follower_pos = [
            self.follower_pose.pose.position.x,
            self.follower_pose.pose.position.y,
            self.follower_pose.pose.position.z
        ]
        
        return np.linalg.norm(np.array(leader_pos) - np.array(follower_pos))
    
    def calculate_joint_error(self):
        """Calculate joint error between leader and follower"""
        if len(self.leader_joints.position) != len(self.follower_joints.position):
            return 0.0
        
        # Compare first 5 joints (exclude gripper)
        n_joints = min(5, len(self.leader_joints.position))
        joint_diffs = []
        
        for i in range(n_joints):
            diff = abs(self.leader_joints.position[i] - self.follower_joints.position[i])
            joint_diffs.append(diff)
        
        return max(joint_diffs) if joint_diffs else 0.0
    
    def get_performance_stats(self):
        """Get performance statistics"""
        if not self.position_errors:
            return "No data yet"
        
        pos_avg = np.mean(self.position_errors) * 1000  # mm
        pos_max = np.max(self.position_errors) * 1000   # mm
        
        joint_avg = np.mean(self.joint_errors) * 57.3 if self.joint_errors else 0  # degrees
        joint_max = np.max(self.joint_errors) * 57.3 if self.joint_errors else 0   # degrees
        
        return f"Position - Avg: {pos_avg:.1f}mm, Max: {pos_max:.1f}mm | Joints - Avg: {joint_avg:.1f}°, Max: {joint_max:.1f}°"

def main(args=None):
    rclpy.init(args=args)
    
    node = SO100TeleopCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stats = node.get_performance_stats()
        node.get_logger().info(f'Final performance stats: {stats}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()