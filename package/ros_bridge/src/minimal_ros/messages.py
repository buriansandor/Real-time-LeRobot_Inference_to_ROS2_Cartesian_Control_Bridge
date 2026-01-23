#!/usr/bin/env python3
"""
ROS2 Compatible Message Definitions

These message types are designed to be compatible with standard ROS2 messages.
When transitioning to full ROS2, these can be directly replaced with:

- sensor_msgs/JointState
- geometry_msgs/PoseStamped  
- geometry_msgs/Twist
- std_msgs/Header

Created by Sandor Burian
"""

from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Optional
import time
import json

# Standard ROS2 message equivalents

@dataclass
class Header:
    """Equivalent to std_msgs/Header"""
    stamp: float
    frame_id: str = "base_link"
    
    @classmethod
    def now(cls, frame_id: str = "base_link"):
        return cls(stamp=time.time(), frame_id=frame_id)

@dataclass  
class Point:
    """Equivalent to geometry_msgs/Point"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Quaternion:
    """Equivalent to geometry_msgs/Quaternion"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

@dataclass
class Pose:
    """Equivalent to geometry_msgs/Pose"""
    position: Point
    orientation: Quaternion

@dataclass
class PoseStamped:
    """Equivalent to geometry_msgs/PoseStamped"""
    header: Header
    pose: Pose
    
    @classmethod
    def create(cls, position: List[float], orientation: List[float], frame_id: str = "base_link"):
        """Create PoseStamped from lists"""
        return cls(
            header=Header.now(frame_id),
            pose=Pose(
                position=Point(position[0], position[1], position[2]),
                orientation=Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
            )
        )

@dataclass
class JointState:
    """Equivalent to sensor_msgs/JointState"""
    header: Header
    name: List[str]
    position: List[float]
    velocity: List[float]
    effort: List[float]
    
    @classmethod
    def create(cls, names: List[str], positions: List[float], 
               velocities: Optional[List[float]] = None, 
               efforts: Optional[List[float]] = None,
               frame_id: str = "base_link"):
        """Create JointState from basic data"""
        n = len(positions)
        return cls(
            header=Header.now(frame_id),
            name=names,
            position=positions,
            velocity=velocities or [0.0] * n,
            effort=efforts or [0.0] * n
        )

@dataclass
class Twist:
    """Equivalent to geometry_msgs/Twist (for velocity commands)"""
    linear: Point
    angular: Point
    
    @classmethod
    def create(cls, linear_vel: List[float], angular_vel: List[float]):
        return cls(
            linear=Point(linear_vel[0], linear_vel[1], linear_vel[2]),
            angular=Point(angular_vel[0], angular_vel[1], angular_vel[2])
        )

@dataclass
class TwistStamped:
    """Equivalent to geometry_msgs/TwistStamped"""
    header: Header
    twist: Twist

# Robot-specific messages

@dataclass
class RobotStatus:
    """Custom message for robot status"""
    header: Header
    robot_name: str
    is_connected: bool
    is_enabled: bool
    error_message: str = ""
    position_error: float = 0.0  # meters
    joint_error: float = 0.0     # radians

@dataclass
class TeleopCommand:
    """Custom message for teleoperation commands"""
    header: Header
    command_type: str  # "pose", "joints", "velocity", "stop"
    target_pose: Optional[PoseStamped] = None
    target_joints: Optional[JointState] = None
    target_velocity: Optional[Twist] = None
    gripper_command: float = -1.0  # -1=no change, 0.0=close, 1.0=open

# JSON serialization helpers (for ZeroMQ transport)

def to_dict(obj) -> Dict[str, Any]:
    """Convert dataclass to dictionary"""
    return asdict(obj)

def to_json(obj) -> str:
    """Convert dataclass to JSON string"""
    return json.dumps(to_dict(obj), default=str)

def from_dict(data: Dict[str, Any], msg_type: type):
    """Convert dictionary back to message type"""
    if msg_type == PoseStamped:
        return PoseStamped(
            header=Header(**data['header']),
            pose=Pose(
                position=Point(**data['pose']['position']),
                orientation=Quaternion(**data['pose']['orientation'])
            )
        )
    elif msg_type == JointState:
        return JointState(
            header=Header(**data['header']),
            name=data['name'],
            position=data['position'],
            velocity=data['velocity'],
            effort=data['effort']
        )
    elif msg_type == RobotStatus:
        return RobotStatus(
            header=Header(**data['header']),
            robot_name=data['robot_name'],
            is_connected=data['is_connected'],
            is_enabled=data['is_enabled'],
            error_message=data.get('error_message', ''),
            position_error=data.get('position_error', 0.0),
            joint_error=data.get('joint_error', 0.0)
        )
    else:
        # Generic fallback
        return msg_type(**data)

def from_json(json_str: str, msg_type: type):
    """Convert JSON string back to message type"""
    data = json.loads(json_str)
    return from_dict(data, msg_type)

# ROS2 migration helpers

class ROS2CompatibilityLayer:
    """
    Helper class to make migration to full ROS2 easier.
    
    When using full ROS2, replace:
    - from minimal_ros.messages import PoseStamped
    + from geometry_msgs.msg import PoseStamped
    
    - node.publish_pose_stamped(topic, msg)
    + publisher.publish(msg)
    """
    
    @staticmethod
    def convert_to_ros2_pose_stamped(pose_stamped):
        """Convert our PoseStamped to ROS2 geometry_msgs/PoseStamped"""
        try:
            from geometry_msgs.msg import PoseStamped as ROS2PoseStamped
            from geometry_msgs.msg import Pose as ROS2Pose, Point as ROS2Point, Quaternion as ROS2Quaternion
            from std_msgs.msg import Header as ROS2Header
            
            ros2_msg = ROS2PoseStamped()
            ros2_msg.header.stamp.sec = int(pose_stamped.header.stamp)
            ros2_msg.header.stamp.nanosec = int((pose_stamped.header.stamp % 1) * 1e9)
            ros2_msg.header.frame_id = pose_stamped.header.frame_id
            
            ros2_msg.pose.position.x = pose_stamped.pose.position.x
            ros2_msg.pose.position.y = pose_stamped.pose.position.y
            ros2_msg.pose.position.z = pose_stamped.pose.position.z
            
            ros2_msg.pose.orientation.x = pose_stamped.pose.orientation.x
            ros2_msg.pose.orientation.y = pose_stamped.pose.orientation.y
            ros2_msg.pose.orientation.z = pose_stamped.pose.orientation.z
            ros2_msg.pose.orientation.w = pose_stamped.pose.orientation.w
            
            return ros2_msg
        except ImportError:
            # ROS2 not available, return original
            return pose_stamped
    
    @staticmethod
    def convert_from_ros2_pose_stamped(ros2_pose_stamped):
        """Convert ROS2 geometry_msgs/PoseStamped to our format"""
        stamp = ros2_pose_stamped.header.stamp.sec + ros2_pose_stamped.header.stamp.nanosec / 1e9
        return PoseStamped.create(
            position=[
                ros2_pose_stamped.pose.position.x,
                ros2_pose_stamped.pose.position.y, 
                ros2_pose_stamped.pose.position.z
            ],
            orientation=[
                ros2_pose_stamped.pose.orientation.x,
                ros2_pose_stamped.pose.orientation.y,
                ros2_pose_stamped.pose.orientation.z,
                ros2_pose_stamped.pose.orientation.w
            ],
            frame_id=ros2_pose_stamped.header.frame_id
        )