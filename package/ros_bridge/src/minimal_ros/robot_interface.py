#!/usr/bin/env python3
"""
Abstract Robot Interface for ROS2 Compatible Teleoperation

This module defines the interface that any follower robot must implement
to work with the ROS2 teleoperation system. This allows easy switching
between SO100, Universal Robots, Franka, etc.

Created by Sandor Burian
"""

from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass

@dataclass
class RobotCapabilities:
    """Describes what a robot can do"""
    has_gripper: bool = False
    max_reach: float = 0.5  # meters
    joint_count: int = 6
    cartesian_control: bool = True
    joint_control: bool = True
    force_feedback: bool = False

class AbstractRobotController(ABC):
    """
    Abstract interface for any robot that can follow teleoperation commands
    
    This interface ensures that any robot (SO100, UR5, Franka, etc.) can be
    controlled via the same ROS2 topics and messages.
    """
    
    @abstractmethod
    def get_capabilities(self) -> RobotCapabilities:
        """Return robot capabilities"""
        pass
    
    @abstractmethod
    def connect(self, port: str, **kwargs) -> bool:
        """Connect to robot hardware"""
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from robot"""
        pass
    
    @abstractmethod
    def get_joint_states(self) -> Tuple[List[str], List[float], List[float], List[float]]:
        """Get current joint states: (names, positions, velocities, efforts)"""
        pass
    
    @abstractmethod
    def get_cartesian_pose(self) -> Tuple[List[float], List[float]]:
        """Get current pose: (position [x,y,z], orientation [x,y,z,w])"""
        pass
    
    @abstractmethod
    def move_to_cartesian(self, position: List[float], orientation: List[float], **kwargs) -> bool:
        """Move to cartesian pose"""
        pass
    
    @abstractmethod
    def move_to_joints(self, joint_positions: List[float], **kwargs) -> bool:
        """Move to joint positions"""
        pass
    
    @abstractmethod
    def enable_torque(self, enable: bool) -> None:
        """Enable/disable motor torque"""
        pass
    
    @abstractmethod
    def emergency_stop(self) -> None:
        """Emergency stop"""
        pass
    
    # Optional gripper interface
    def has_gripper(self) -> bool:
        """Check if robot has gripper"""
        return self.get_capabilities().has_gripper
    
    def gripper_open(self) -> None:
        """Open gripper (override if robot has gripper)"""
        pass
    
    def gripper_close(self) -> None:
        """Close gripper (override if robot has gripper)"""  
        pass
    
    def set_gripper_position(self, position: float) -> None:
        """Set gripper position 0.0-1.0 (override if robot has gripper)"""
        pass

class SO100RobotController(AbstractRobotController):
    """SO100 implementation of the robot interface"""
    
    def __init__(self, config_dir: Optional[str] = None):
        self.robot = None
        self.config_dir = config_dir
        self._capabilities = RobotCapabilities(
            has_gripper=True,
            max_reach=0.6,
            joint_count=6,
            cartesian_control=True,
            joint_control=True,
            force_feedback=False
        )
    
    def get_capabilities(self) -> RobotCapabilities:
        return self._capabilities
    
    def connect(self, port: str, **kwargs) -> bool:
        """Connect to SO100 robot"""
        try:
            import sys
            from pathlib import Path
            # Add SO100 driver to path - go up to find the package directory
            script_dir = Path(__file__).parent
            package_root = script_dir.parent.parent.parent.parent  # Go up to find the main package
            sys.path.insert(0, str(package_root))
            
            from package.drivers.SO100_Robot.so100_core import SO100Robot
            
            self.robot = SO100Robot(port=port, config_dir=self.config_dir)
            
            # Determine mode based on usage context
            is_passive = kwargs.get('passive_mode', False)
            if is_passive:
                self.robot.torque_enable(False)
                print(f"SO100 set to passive mode - can be moved by hand")
            else:
                self.robot.torque_enable(True)  
                print(f"SO100 set to active mode - ready to follow commands")
                
            print(f"SO100 connected to {port}")
            return True
        except ImportError as e:
            print(f"SO100 import failed: {e}")
            print(f"Looking for package at: {package_root}")
            return False
        except Exception as e:
            print(f"SO100 connection failed: {e}")
            return False
    
    def disconnect(self) -> None:
        if self.robot:
            self.robot.torque_enable(False)
            self.robot.close()
    
    def get_joint_states(self) -> Tuple[List[str], List[float], List[float], List[float]]:
        if not self.robot:
            return [], [], [], []
        
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper_joint']
        positions = self.robot.get_joint_angles()
        velocities = [0.0] * len(positions)  # SO100 doesn't provide velocity feedback
        efforts = [0.0] * len(positions)     # SO100 doesn't provide effort feedback
        
        return joint_names, positions, velocities, efforts
    
    def get_cartesian_pose(self) -> Tuple[List[float], List[float]]:
        if not self.robot:
            return [0, 0, 0], [0, 0, 0, 1]
        
        position = self.robot.get_cartesian_position()
        orientation = [0.0, 0.0, 0.0, 1.0]  # SO100 uses fixed orientation
        
        return position, orientation
    
    def move_to_cartesian(self, position: List[float], orientation: List[float], **kwargs) -> bool:
        if not self.robot:
            return False
        
        try:
            # Extract time parameter
            time_ms = kwargs.get('time_ms', 100)
            self.robot.move_to_cartesian(position[0], position[1], position[2], time_ms=time_ms)
            return True
        except Exception as e:
            print(f"SO100 move_to_cartesian error: {e}")
            return False
    
    def move_to_joints(self, joint_positions: List[float], **kwargs) -> bool:
        if not self.robot:
            return False
        
        try:
            time_ms = kwargs.get('time_ms', 100)
            self.robot.move_to_joints(joint_positions, time_ms=time_ms)
            return True
        except Exception as e:
            print(f"SO100 move_to_joints error: {e}")
            return False
    
    def enable_torque(self, enable: bool) -> None:
        if self.robot:
            self.robot.torque_enable(enable)
    
    def emergency_stop(self) -> None:
        if self.robot:
            self.robot.torque_enable(False)
    
    def set_passive_mode(self, passive: bool) -> None:
        """Set robot to passive mode for manual manipulation"""
        if self.robot:
            if passive:
                self.robot.torque_enable(False)  # Disable torque for manual movement
                print("SO100 set to passive mode - can be moved by hand")
            else:
                self.robot.torque_enable(True)   # Enable torque for active control
                print("SO100 set to active mode - ready for commands")
    
    def gripper_open(self) -> None:
        if self.robot:
            self.robot.gripper_open()
    
    def gripper_close(self) -> None:
        if self.robot:
            self.robot.gripper_close()

class LeRobotController(AbstractRobotController):
    """LeRobot API based robot controller"""
    
    def __init__(self, robot_name: str = "so100", config_path: str = None):
        self.robot_name = robot_name
        self.config_path = config_path
        self.robot = None
        self.device = None
        
        # Try to import LeRobot
        try:
            from lerobot.common.robot_devices.robots.factory import make_robot
            from lerobot.common.robot_devices.cameras.factory import make_camera
            self.make_robot = make_robot
            self.lerobot_available = True
        except ImportError:
            print("LeRobot not installed. Install with: pip install lerobot")
            self.lerobot_available = False
        
        self._capabilities = RobotCapabilities(
            has_gripper=True,
            max_reach=0.35,     # SO100 reach
            joint_count=6,      # 5 joints + gripper
            cartesian_control=True,
            joint_control=True,
            force_feedback=False
        )
    
    def get_capabilities(self) -> RobotCapabilities:
        return self._capabilities
    
    def connect(self, port: str = None, **kwargs) -> bool:
        """Connect to robot via LeRobot API"""
        if not self.lerobot_available:
            print("LeRobot not available")
            return False
        
        try:
            # Create robot through LeRobot API
            self.robot = self.make_robot(
                robot_type=self.robot_name,
                overrides=kwargs.get('overrides', {})
            )
            
            # Connect and setup
            self.robot.connect()
            print(f"LeRobot {self.robot_name} connected successfully")
            return True
            
        except Exception as e:
            print(f"LeRobot connection error: {e}")
            return False
    
    def disconnect(self) -> None:
        if self.robot:
            try:
                self.robot.disconnect()
            except:
                pass
            self.robot = None
    
    def get_joint_states(self) -> Tuple[List[str], List[float], List[float], List[float]]:
        """Get current joint states via LeRobot observation"""
        if not self.robot:
            return [], [], [], []
        
        try:
            # Get observation from LeRobot
            observation = self.robot.capture_observation()
            
            # Extract joint positions
            if 'arm' in observation:
                joint_positions = observation['arm'].tolist()
            elif 'joint_positions' in observation:
                joint_positions = observation['joint_positions'].tolist()
            else:
                # Fallback - try direct state reading
                joint_positions = self.robot.get_state()
            
            joint_names = [f'joint_{i+1}' for i in range(len(joint_positions)-1)] + ['gripper_joint']
            velocities = [0.0] * len(joint_positions)
            efforts = [0.0] * len(joint_positions)
            
            return joint_names, joint_positions, velocities, efforts
            
        except Exception as e:
            print(f"LeRobot get_joint_states error: {e}")
            return [], [], [], []
    
    def get_cartesian_pose(self) -> Tuple[List[float], List[float]]:
        """Get cartesian pose via LeRobot forward kinematics"""
        if not self.robot:
            return [0, 0, 0], [0, 0, 0, 1]
        
        try:
            # Method 1: Direct cartesian reading if available
            if hasattr(self.robot, 'get_cartesian_position'):
                position = self.robot.get_cartesian_position()
                orientation = [0.0, 0.0, 0.0, 1.0]  # Default orientation
                return position, orientation
            
            # Method 2: Via observation + forward kinematics
            observation = self.robot.capture_observation()
            
            # Check if observation already contains cartesian coordinates
            if 'cartesian_position' in observation:
                position = observation['cartesian_position'].tolist()
                orientation = observation.get('cartesian_orientation', [0, 0, 0, 1]).tolist()
                return position, orientation
            
            # Method 3: Forward kinematics from joint positions
            if 'arm' in observation:
                joint_positions = observation['arm'].tolist()
                # Use robot's built-in forward kinematics
                if hasattr(self.robot, 'forward_kinematics'):
                    cartesian = self.robot.forward_kinematics(joint_positions[:-1])  # Exclude gripper
                    if len(cartesian) >= 3:
                        position = cartesian[:3]
                        orientation = cartesian[3:] if len(cartesian) >= 7 else [0, 0, 0, 1]
                        return position, orientation
            
            # Fallback: return zero position
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
            
        except Exception as e:
            print(f"LeRobot get_cartesian_pose error: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    
    def move_to_cartesian(self, position: List[float], orientation: List[float], **kwargs) -> bool:
        """Move to cartesian position via LeRobot"""
        if not self.robot:
            return False
        
        try:
            # Method 1: Direct cartesian control if available
            if hasattr(self.robot, 'move_to_cartesian'):
                self.robot.move_to_cartesian(position, orientation)
                return True
            
            # Method 2: Use inverse kinematics + joint control
            if hasattr(self.robot, 'inverse_kinematics'):
                joint_positions = self.robot.inverse_kinematics(position, orientation)
                return self.move_to_joints(joint_positions, **kwargs)
            
            print("LeRobot cartesian control not available")
            return False
            
        except Exception as e:
            print(f"LeRobot move_to_cartesian error: {e}")
            return False
    
    def move_to_joints(self, joint_positions: List[float], **kwargs) -> bool:
        """Move to joint positions via LeRobot"""
        if not self.robot:
            return False
        
        try:
            # Use LeRobot's action interface
            if hasattr(self.robot, 'send_action'):
                action = {'arm': joint_positions}
                self.robot.send_action(action)
                return True
            elif hasattr(self.robot, 'move_to_joints'):
                self.robot.move_to_joints(joint_positions)
                return True
            else:
                print("LeRobot joint control not available")
                return False
                
        except Exception as e:
            print(f"LeRobot move_to_joints error: {e}")
            return False
    
    def enable_torque(self, enable: bool) -> None:
        """Enable/disable torque via LeRobot"""
        if self.robot and hasattr(self.robot, 'enable_torque'):
            self.robot.enable_torque(enable)
    
    def emergency_stop(self) -> None:
        """Emergency stop via LeRobot"""
        if self.robot and hasattr(self.robot, 'emergency_stop'):
            self.robot.emergency_stop()
    
    def has_gripper(self) -> bool:
        return True
    
    def gripper_open(self) -> None:
        """Open gripper via LeRobot"""
        if self.robot and hasattr(self.robot, 'gripper_open'):
            self.robot.gripper_open()
    
    def gripper_close(self) -> None:
        """Close gripper via LeRobot"""
        if self.robot and hasattr(self.robot, 'gripper_close'):
            self.robot.gripper_close()
    
    def set_passive_mode(self, passive: bool) -> None:
        """Set robot to passive mode for manual manipulation"""
        if self.robot and hasattr(self.robot, 'set_passive_mode'):
            self.robot.set_passive_mode(passive)
        elif passive:
            self.enable_torque(False)

class UniversalRobotController(AbstractRobotController):
    """Placeholder for Universal Robot (UR5/UR10) implementation"""
    
    def __init__(self):
        self._capabilities = RobotCapabilities(
            has_gripper=False,  # Depends on end-effector
            max_reach=0.85,     # UR5 reach
            joint_count=6,
            cartesian_control=True,
            joint_control=True,
            force_feedback=True
        )
    
    def get_capabilities(self) -> RobotCapabilities:
        return self._capabilities
    
    def connect(self, port: str, **kwargs) -> bool:
        # TODO: Implement UR RTDe connection
        print("Universal Robot controller not yet implemented")
        return False
    
    def disconnect(self) -> None:
        pass
    
    def get_joint_states(self) -> Tuple[List[str], List[float], List[float], List[float]]:
        return [], [], [], []
    
    def get_cartesian_pose(self) -> Tuple[List[float], List[float]]:
        return [0, 0, 0], [0, 0, 0, 1]
    
    def move_to_cartesian(self, position: List[float], orientation: List[float], **kwargs) -> bool:
        return False
    
    def move_to_joints(self, joint_positions: List[float], **kwargs) -> bool:
        return False
    
    def enable_torque(self, enable: bool) -> None:
        pass
    
    def emergency_stop(self) -> None:
        pass

def create_robot_controller(robot_type: str, **kwargs) -> AbstractRobotController:
    """Factory function to create robot controllers"""
    robot_type = robot_type.lower()
    
    if robot_type == 'so100':
        return SO100RobotController(kwargs.get('config_dir'))
    elif robot_type in ['ur5', 'ur10', 'universal']:
        return UniversalRobotController()
    elif robot_type in ['lerobot', 'lerobot_so100']:
        return LeRobotController(robot_name="so100", config_path=kwargs.get('config_path'))
    elif robot_type.startswith('lerobot_'):
        # Extract robot name: lerobot_aloha -> aloha
        robot_name = robot_type.replace('lerobot_', '')
        return LeRobotController(robot_name=robot_name, config_path=kwargs.get('config_path'))
    else:
        raise ValueError(f"Unknown robot type: {robot_type}. Supported: so100, ur5, ur10, lerobot, lerobot_<robot_name>")

def get_supported_robots() -> List[str]:
    """Get list of all supported robot types"""
    return [
        'SO100',           # Direct SO100 driver
        'UR5', 'UR10',     # Universal Robots  
        'LeRobot',         # Generic LeRobot
        'LeRobot_SO100',   # LeRobot SO100
        'LeRobot_Aloha',   # LeRobot Aloha
        'LeRobot_Koch',    # LeRobot Koch
        # Add more as needed...
    ]