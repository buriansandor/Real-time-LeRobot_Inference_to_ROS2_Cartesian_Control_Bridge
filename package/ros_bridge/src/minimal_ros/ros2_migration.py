#!/usr/bin/env python3
"""
ROS2 Migration Utilities

Utilities for converting between Minimal ROS (ZeroMQ) and full ROS2.
Enables seamless migration path from venv-compatible development to production ROS2.

Created by Sandor Burian
"""

import json
import time
from typing import Dict, Any, List, Optional, Union

# ROS2 imports (conditional - only if ROS2 is available)
try:
    import rclpy
    from geometry_msgs.msg import PoseStamped as ROS2PoseStamped, Point as ROS2Point, Quaternion as ROS2Quaternion
    from sensor_msgs.msg import JointState as ROS2JointState
    from std_msgs.msg import Header as ROS2Header
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

# Import our minimal ROS messages
from messages import PoseStamped, JointState, Point, Quaternion, Header, RobotStatus, TeleopCommand

class MessageConverter:
    """Convert between Minimal ROS and ROS2 message formats"""
    
    @staticmethod
    def minimal_to_ros2_header(minimal_header: Dict[str, Any]) -> 'ROS2Header':
        """Convert minimal header to ROS2 Header"""
        if not HAS_ROS2:
            raise RuntimeError("ROS2 not available")
            
        ros2_header = ROS2Header()
        ros2_header.stamp.sec = int(minimal_header.get('stamp', time.time()))
        ros2_header.stamp.nanosec = int((minimal_header.get('stamp', time.time()) % 1) * 1e9)
        ros2_header.frame_id = minimal_header.get('frame_id', 'base_link')
        return ros2_header
    
    @staticmethod
    def ros2_to_minimal_header(ros2_header: 'ROS2Header') -> Dict[str, Any]:
        """Convert ROS2 Header to minimal header"""
        return {
            'stamp': ros2_header.stamp.sec + ros2_header.stamp.nanosec * 1e-9,
            'frame_id': ros2_header.frame_id
        }
    
    @staticmethod
    def minimal_to_ros2_point(minimal_point: Dict[str, float]) -> 'ROS2Point':
        """Convert minimal point to ROS2 Point"""
        if not HAS_ROS2:
            raise RuntimeError("ROS2 not available")
            
        ros2_point = ROS2Point()
        ros2_point.x = minimal_point.get('x', 0.0)
        ros2_point.y = minimal_point.get('y', 0.0) 
        ros2_point.z = minimal_point.get('z', 0.0)
        return ros2_point
    
    @staticmethod
    def ros2_to_minimal_point(ros2_point: 'ROS2Point') -> Dict[str, float]:
        """Convert ROS2 Point to minimal point"""
        return {
            'x': ros2_point.x,
            'y': ros2_point.y,
            'z': ros2_point.z
        }
    
    @staticmethod
    def minimal_to_ros2_quaternion(minimal_quat: Dict[str, float]) -> 'ROS2Quaternion':
        """Convert minimal quaternion to ROS2 Quaternion"""
        if not HAS_ROS2:
            raise RuntimeError("ROS2 not available")
            
        ros2_quat = ROS2Quaternion()
        ros2_quat.x = minimal_quat.get('x', 0.0)
        ros2_quat.y = minimal_quat.get('y', 0.0)
        ros2_quat.z = minimal_quat.get('z', 0.0)
        ros2_quat.w = minimal_quat.get('w', 1.0)
        return ros2_quat
    
    @staticmethod
    def ros2_to_minimal_quaternion(ros2_quat: 'ROS2Quaternion') -> Dict[str, float]:
        """Convert ROS2 Quaternion to minimal quaternion"""
        return {
            'x': ros2_quat.x,
            'y': ros2_quat.y,
            'z': ros2_quat.z,
            'w': ros2_quat.w
        }
    
    @staticmethod
    def minimal_to_ros2_pose_stamped(minimal_pose: PoseStamped) -> 'ROS2PoseStamped':
        """Convert minimal PoseStamped to ROS2 PoseStamped"""
        if not HAS_ROS2:
            raise RuntimeError("ROS2 not available")
            
        ros2_pose = ROS2PoseStamped()
        ros2_pose.header = MessageConverter.minimal_to_ros2_header(minimal_pose.header)
        ros2_pose.pose.position = MessageConverter.minimal_to_ros2_point(minimal_pose.pose.position)
        ros2_pose.pose.orientation = MessageConverter.minimal_to_ros2_quaternion(minimal_pose.pose.orientation)
        return ros2_pose
    
    @staticmethod
    def ros2_to_minimal_pose_stamped(ros2_pose: 'ROS2PoseStamped') -> PoseStamped:
        """Convert ROS2 PoseStamped to minimal PoseStamped"""
        position = MessageConverter.ros2_to_minimal_point(ros2_pose.pose.position)
        orientation = MessageConverter.ros2_to_minimal_quaternion(ros2_pose.pose.orientation)
        header = MessageConverter.ros2_to_minimal_header(ros2_pose.header)
        
        return PoseStamped.create_from_dict({
            'header': header,
            'pose': {
                'position': position,
                'orientation': orientation
            }
        })
    
    @staticmethod
    def minimal_to_ros2_joint_state(minimal_joint: JointState) -> 'ROS2JointState':
        """Convert minimal JointState to ROS2 JointState"""
        if not HAS_ROS2:
            raise RuntimeError("ROS2 not available")
            
        ros2_joint = ROS2JointState()
        ros2_joint.header = MessageConverter.minimal_to_ros2_header(minimal_joint.header)
        ros2_joint.name = minimal_joint.name
        ros2_joint.position = minimal_joint.position
        ros2_joint.velocity = minimal_joint.velocity
        ros2_joint.effort = minimal_joint.effort
        return ros2_joint
    
    @staticmethod
    def ros2_to_minimal_joint_state(ros2_joint: 'ROS2JointState') -> JointState:
        """Convert ROS2 JointState to minimal JointState"""
        header = MessageConverter.ros2_to_minimal_header(ros2_joint.header)
        
        return JointState.create_from_dict({
            'header': header,
            'name': list(ros2_joint.name),
            'position': list(ros2_joint.position),
            'velocity': list(ros2_joint.velocity),
            'effort': list(ros2_joint.effort)
        })

class ROS2Bridge:
    """Bridge between Minimal ROS (ZeroMQ) and ROS2"""
    
    def __init__(self, node_name: str = 'minimal_ros_bridge'):
        if not HAS_ROS2:
            raise RuntimeError("ROS2 not available. Install ROS2 to use bridge functionality.")
        
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.converter = MessageConverter()
        
        # ZeroMQ imports for bridge
        try:
            from minimal_ros import MinimalNode
            self.minimal_node = MinimalNode(f"{node_name}_minimal", base_port=5600)
        except Exception as e:
            self.node.get_logger().error(f"Failed to create minimal ROS node: {e}")
            raise
        
        self.node.get_logger().info("ROS2 Bridge initialized")
    
    def create_pose_bridge(self, minimal_topic: str, ros2_topic: str, 
                          minimal_to_ros2: bool = True):
        """Create bidirectional bridge for PoseStamped messages"""
        
        if minimal_to_ros2:
            # Minimal ROS -> ROS2
            ros2_pub = self.node.create_publisher(ROS2PoseStamped, ros2_topic, 10)
            
            def callback(minimal_msg: PoseStamped):
                try:
                    ros2_msg = self.converter.minimal_to_ros2_pose_stamped(minimal_msg)
                    ros2_pub.publish(ros2_msg)
                    self.node.get_logger().debug(f"Bridged pose: {minimal_topic} -> {ros2_topic}")
                except Exception as e:
                    self.node.get_logger().error(f"Pose bridge error: {e}")
            
            self.minimal_node.create_subscription(minimal_topic, PoseStamped, callback, 10)
            
        else:
            # ROS2 -> Minimal ROS
            minimal_pub = self.minimal_node.create_publisher(minimal_topic, PoseStamped, 10)
            
            def callback(ros2_msg: ROS2PoseStamped):
                try:
                    minimal_msg = self.converter.ros2_to_minimal_pose_stamped(ros2_msg)
                    minimal_pub.publish(minimal_msg)
                    self.node.get_logger().debug(f"Bridged pose: {ros2_topic} -> {minimal_topic}")
                except Exception as e:
                    self.node.get_logger().error(f"Pose bridge error: {e}")
            
            self.node.create_subscription(ROS2PoseStamped, ros2_topic, callback, 10)
    
    def create_joint_bridge(self, minimal_topic: str, ros2_topic: str, 
                           minimal_to_ros2: bool = True):
        """Create bidirectional bridge for JointState messages"""
        
        if minimal_to_ros2:
            # Minimal ROS -> ROS2
            ros2_pub = self.node.create_publisher(ROS2JointState, ros2_topic, 10)
            
            def callback(minimal_msg: JointState):
                try:
                    ros2_msg = self.converter.minimal_to_ros2_joint_state(minimal_msg)
                    ros2_pub.publish(ros2_msg)
                    self.node.get_logger().debug(f"Bridged joint: {minimal_topic} -> {ros2_topic}")
                except Exception as e:
                    self.node.get_logger().error(f"Joint bridge error: {e}")
            
            self.minimal_node.create_subscription(minimal_topic, JointState, callback, 10)
            
        else:
            # ROS2 -> Minimal ROS
            minimal_pub = self.minimal_node.create_publisher(minimal_topic, JointState, 10)
            
            def callback(ros2_msg: ROS2JointState):
                try:
                    minimal_msg = self.converter.ros2_to_minimal_joint_state(ros2_msg)
                    minimal_pub.publish(minimal_msg)
                    self.node.get_logger().debug(f"Bridged joint: {ros2_topic} -> {minimal_topic}")
                except Exception as e:
                    self.node.get_logger().error(f"Joint bridge error: {e}")
            
            self.node.create_subscription(ROS2JointState, ros2_topic, callback, 10)
    
    def spin(self):
        """Run the bridge"""
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.001)
                time.sleep(0.001)
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy()
    
    def destroy(self):
        """Clean shutdown"""
        if hasattr(self, 'minimal_node'):
            self.minimal_node.destroy_node()
        if hasattr(self, 'node'):
            self.node.destroy_node()
        rclpy.shutdown()

def create_standard_bridge():
    """Create a standard bridge for teleoperation topics"""
    
    if not HAS_ROS2:
        print("ROS2 not available. Bridge functionality disabled.")
        return None
    
    bridge = ROS2Bridge('teleoperation_bridge')
    
    # Bridge teleoperation topics
    bridge.create_pose_bridge('robot_target_pose', '/robot_target_pose', minimal_to_ros2=True)
    bridge.create_joint_bridge('robot_target_joints', '/joint_states', minimal_to_ros2=True)
    
    # Bridge feedback topics
    bridge.create_pose_bridge('robot_current_pose', '/robot_current_pose', minimal_to_ros2=True)
    bridge.create_joint_bridge('joint_states', '/joint_states_feedback', minimal_to_ros2=True)
    
    print("Standard teleoperation bridge created:")
    print("  Minimal ROS -> ROS2:")
    print("    robot_target_pose -> /robot_target_pose")
    print("    robot_target_joints -> /joint_states")
    print("    robot_current_pose -> /robot_current_pose")
    print("    joint_states -> /joint_states_feedback")
    
    return bridge

def main():
    """Run the ROS2 bridge"""
    bridge = create_standard_bridge()
    
    if bridge:
        print("Starting ROS2 bridge...")
        bridge.spin()
    else:
        print("Bridge not available - ROS2 not installed")

if __name__ == '__main__':
    main()