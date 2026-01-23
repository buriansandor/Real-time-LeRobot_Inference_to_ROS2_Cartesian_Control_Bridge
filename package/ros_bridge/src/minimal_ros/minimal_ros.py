#!/usr/bin/env python3
"""
Minimal ROS-like Communication Bridge (venv compatible)

This provides ROS-like topic communication without requiring full ROS2 installation.
Uses ZeroMQ for inter-process communication and pickle for message serialization.

Created by Sandor Burian
"""

import threading
import time
import queue
import json
import socket
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Any, Callable
import zmq

# Import message types for proper deserialization
from messages import Header, Point, Quaternion, Pose, PoseStamped, JointState, TeleopCommand, RobotStatus

# Import message types for proper deserialization
from messages import Header, Point, Quaternion, Pose, PoseStamped, JointState, TeleopCommand, RobotStatus

@dataclass
class LegacyJointState:
    """Legacy ROS-like JointState message (for backwards compatibility)"""
    header: Dict[str, Any]
    name: List[str]
    position: List[float]
    velocity: List[float]
    effort: List[float]

@dataclass
class LegacyPose:
    """Legacy ROS-like Pose message (for backwards compatibility)"""
    position: Dict[str, float]  # x, y, z
    orientation: Dict[str, float]  # x, y, z, w

@dataclass
class LegacyPoseStamped:
    """Legacy ROS-like PoseStamped message (for backwards compatibility)"""
    header: Dict[str, Any]
    pose: LegacyPose

class MinimalNode:
    """Minimal ROS-like Node implementation"""
    
    def __init__(self, name: str, base_port: int = 5555):
        self.name = name
        self.base_port = base_port
        self.context = zmq.Context()
        
        # Publishers and subscribers
        self.publishers: Dict[str, zmq.Socket] = {}
        self.subscribers: Dict[str, zmq.Socket] = {}
        self.callbacks: Dict[str, Callable] = {}
        
        # Fixed topic-to-port mapping for consistent communication
        self.topic_ports = {
            'robot_target_pose': 5560,
            'robot_target_joints': 5561, 
            'teleop_command': 5562,
            'leader_status': 5563,
            'robot_status': 5564,
            'joint_states': 5565,
            'robot_current_pose': 5566
        }
        
        # Threading
        self.running = True
        self.subscriber_threads = []
        
        # Simple logging
        self.log_level = "INFO"
        
    def create_publisher(self, topic: str, msg_type: type, queue_size: int = 10):
        """Create a publisher for a topic"""
        port = self.topic_ports.get(topic, self.base_port + hash(topic) % 1000)
        socket = self.context.socket(zmq.PUB)
        socket.bind(f"tcp://*:{port}")
        self.publishers[topic] = socket
        self.get_logger().info(f"Publisher created for {topic} on port {port}")
        return Publisher(socket, topic, msg_type)
    
    def create_subscription(self, topic: str, msg_type: type, callback: Callable, queue_size: int = 10):
        """Create a subscription to a topic"""
        port = self.topic_ports.get(topic, self.base_port + hash(topic) % 1000)
        socket = self.context.socket(zmq.SUB)
        socket.connect(f"tcp://localhost:{port}")
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.subscribers[topic] = socket
        self.callbacks[topic] = callback
        
        # Start subscriber thread
        thread = threading.Thread(target=self._subscriber_worker, args=(topic, socket, callback, msg_type))
        thread.daemon = True
        thread.start()
        self.subscriber_threads.append(thread)
        
        self.get_logger().info(f"Subscription created for {topic} on port {port}")
        
    def _subscriber_worker(self, topic: str, socket: zmq.Socket, callback: Callable, msg_type: type):
        """Worker thread for handling subscription messages"""
        while self.running:
            try:
                # Non-blocking receive with timeout
                if socket.poll(100, zmq.POLLIN):
                    message = socket.recv_json()
                    # Deserialize message to proper type
                    msg = self._deserialize_message(message, msg_type)
                    callback(msg)
            except Exception as e:
                if self.running:  # Only log if we're supposed to be running
                    self.get_logger().error(f"Subscription error on {topic}: {e}")
                    time.sleep(0.1)
    
    def _deserialize_message(self, data: Dict[str, Any], msg_type: type):
        """Convert dictionary data back to proper message object"""
        try:
            if msg_type == JointState:
                return JointState(
                    header=Header(**data['header']),
                    name=data['name'],
                    position=data['position'],
                    velocity=data['velocity'],
                    effort=data['effort']
                )
            elif msg_type == PoseStamped:
                pose_data = data['pose']
                return PoseStamped(
                    header=Header(**data['header']),
                    pose=Pose(
                        position=Point(**pose_data['position']),
                        orientation=Quaternion(**pose_data['orientation'])
                    )
                )
            elif msg_type == TeleopCommand:
                cmd = TeleopCommand(
                    header=Header(**data['header']),
                    command_type=data['command_type'],
                    gripper_command=data.get('gripper_command', -1.0)
                )
                
                # Deserialize optional fields
                if data.get('target_pose'):
                    cmd.target_pose = self._deserialize_message(data['target_pose'], PoseStamped)
                if data.get('target_joints'):
                    cmd.target_joints = self._deserialize_message(data['target_joints'], JointState)
                    
                return cmd
                
            elif msg_type == RobotStatus:
                return RobotStatus(
                    header=Header(**data['header']),
                    robot_name=data['robot_name'],
                    is_connected=data['is_connected'],
                    is_enabled=data['is_enabled'],
                    error_message=data.get('error_message', ""),
                    position_error=data.get('position_error', 0.0),
                    joint_error=data.get('joint_error', 0.0)
                )
            else:
                # Return as-is for unknown types
                return data
                
        except Exception as e:
            self.get_logger().error(f"Message deserialization error for {msg_type.__name__}: {e}")
            # Fallback to dict
            return data
    
    def get_logger(self):
        """Get logger (simple implementation)"""
        return SimpleLogger(self.name)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        for socket in self.publishers.values():
            socket.close()
        for socket in self.subscribers.values():
            socket.close()
        self.context.term()

class Publisher:
    """Simple publisher class"""
    
    def __init__(self, socket: zmq.Socket, topic: str, msg_type: type):
        self.socket = socket
        self.topic = topic
        self.msg_type = msg_type
        
    def publish(self, message):
        """Publish a message"""
        try:
            # Serialize message to JSON
            if hasattr(message, '__dict__'):
                data = asdict(message) if hasattr(message, '__dataclass_fields__') else message.__dict__
            else:
                data = message
            
            self.socket.send_json(data)
        except Exception as e:
            print(f"Publish error on {self.topic}: {e}")

class SimpleLogger:
    """Simple logger implementation"""
    
    def __init__(self, name: str):
        self.name = name
        
    def info(self, message: str):
        print(f"[{time.strftime('%H:%M:%S')}] [INFO] [{self.name}]: {message}")
        
    def warn(self, message: str):
        print(f"[{time.strftime('%H:%M:%S')}] [WARN] [{self.name}]: {message}")
        
    def error(self, message: str):
        print(f"[{time.strftime('%H:%M:%S')}] [ERROR] [{self.name}]: {message}")

def create_header():
    """Create a simple header with timestamp"""
    return {
        "stamp": time.time(),
        "frame_id": "base_link"
    }