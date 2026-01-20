"""
SO100 Robot Core Module
Handles robot initialization, kinematics, and motor control.
Created by Sandor Burian based on the summarization of Google Gemini Pro about the so100_robot.py.
Extended and translated with Copilot.
"""

import time
import math
import os
import csv
import numpy as np
from pathlib import Path

from utils import find_config_directory, find_file, find_project_file
from .sts3215 import STS3215Driver
from .kinematics import SO100Kinematics

class SO100Robot:
    def __init__(self, port, config_dir="config"):
        self.driver = STS3215Driver(port)
        
        # Use utilities for intelligent file searching
        config_path = find_config_directory(config_dir, "SO100_Robot")
        
        # Find URDF with fallbacks
        urdf_path = find_file(config_path, "so100.urdf", [
            "demo/SO100/URDF/so100.urdf",
            "package/drivers/SO100_Robot/config/so100.urdf"
        ])
        
        print(f"Using URDF: {urdf_path}")
        self.kinematics = SO100Kinematics(str(urdf_path))
        
        # Find calibration file
        calib_path = find_project_file("follower_calibration.csv", [
            str(config_path / "follower_calibration.csv"),
            "demo/follower_calibration.csv"
        ])
        
        # Find gripper configuration
        gripper_path = find_project_file("gripper_values.csv", [
            str(config_path / "gripper_values.csv"),
            "demo/gripper_values.csv"
        ])
        
        # Load configurations
        self.offsets = {}
        self.directions = {}
        self.adjustments = {}
        self._load_calibration(calib_path)
        
        self.gripper_limits = {'open': 2000, 'close': 1500}
        self._load_gripper_config(gripper_path)
        
        # Current state (Seed for IK)
        self.current_joint_state = [0.0] * 6

    def _load_calibration(self, filepath):
        print(f"Loading calibration from: {filepath}")
        try:
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if row[0] == 'ZERO_OFFSETS':
                        self.offsets = {i: int(val) for i, val in enumerate(row[1:])}
                    elif row[0] == 'DIRECTIONS':
                        self.directions = {i: int(val) for i, val in enumerate(row[1:])}
                    elif row[0] == 'CALIBRATION_POSE_ADJUSTMENTS':
                        self.adjustments = {i: float(val) for i, val in enumerate(row[1:])}
        except Exception as e:
            print(f"WARN: Calibration load failed ({e}). Using defaults.")
            # Default fallback (so it doesn't crash)
            self.offsets = {i: 2048 for i in range(6)}
            self.directions = {i: 1 for i in range(6)}
            self.adjustments = {i: 0.0 for i in range(6)}

    def _load_gripper_config(self, filepath):
        try:
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if row[0] == 'GRIPPER_OPEN': self.gripper_limits['open'] = int(row[1])
                    elif row[0] == 'GRIPPER_CLOSE': self.gripper_limits['close'] = int(row[1])
        except:
            print("WARN: No gripper config found, using defaults.")

    def _angle_to_raw(self, motor_index, radians):
        # Fixed logic to match original so100_control_driver.py
        # 1. Apply calibration adjustment (subtract, not add!)
        corrected_angle = radians - self.adjustments.get(motor_index, 0.0)
        
        # 2. Convert using the original formula
        # raw = ((rad - adjust) / (ratio * dir)) + offset
        ratio = (2 * math.pi / 4096)
        direction = self.directions.get(motor_index, 1)
        offset = self.offsets.get(motor_index, 2048)
        
        raw_float = (corrected_angle / (ratio * direction)) + offset
        final_raw = int(raw_float)
        
        # 3. Safety clipping (0..4095)
        final_raw = max(0, min(4095, final_raw))
        return final_raw

    def get_joint_angles(self):
        """
        Returns the current angles in radians.
        (This will be needed for the LEADER arm and LeRobot dataset!)
        """
        angles = []
        for i in range(6): # Motor 1-6 (Index 0-5)
            raw = self.driver.read_position(i + 1)
            if raw is None:
                angles.append(0.0) # On error
                continue
            
            # Raw -> Radians reverse calculation (matching original so100_control_driver.py)
            offset = self.offsets.get(i, 2048)
            direction = self.directions.get(i, 1)
            ratio = (2 * math.pi / 4096)
            
            # Original formula: angle = (raw_pos - offset) * direction * ratio
            angle = (raw - offset) * direction * ratio
            
            # Add back the adjustment (reverse of what we subtract in _angle_to_raw)
            final_rad = angle + self.adjustments.get(i, 0.0)
            angles.append(final_rad)
            
        return angles

    def move_to_joints(self, joint_angles, time_ms=1000):
        """
        Direct joint control (LeRobot mode) - Optimized for speed
        """
        raw_commands = []
        
        # Pre-calculate all raw values
        for i, angle in enumerate(joint_angles):
            if i == 5: continue # Gripper handled separately
            raw_val = self._angle_to_raw(i, angle)
            raw_commands.append((i + 1, raw_val))
        
        # Send all commands quickly without delays
        for motor_id, raw_val in raw_commands:
            self.driver.write_position(motor_id, raw_val, speed=time_ms, acc=0)
            
        self.current_joint_state = list(joint_angles) # Update internal state

    def move_to_cartesian(self, x, y, z, pitch_rad=0.0, roll_rad=0.0, time_ms=1500):
        """
        XYZ control (ROS mode)
        """
        # IK calculation starting from current state (seed)
        # IKPy's 0th element is the base (fixed), so [0] + current
        seed = [0] + self.current_joint_state[:5] + [0] 
        
        target_joints = self.kinematics.inverse_kinematics(
            target_pos=[x, y, z],
            target_orient=[0, 0, -1], # Looking downwards
            orientation_mode="Z",
            seed_state=seed
        )
        
        # target_joints[1:6] contains the motor angles (Shoulder -> Wrist Roll)
        motor_commands = target_joints[1:6] # 5 motors
        
        # Send commands
        self.move_to_joints(motor_commands, time_ms)
        # No delay - let hardware handle timing

    def gripper_open(self):
        val = self.gripper_limits['open']
        self.driver.write_position(6, val)
        time.sleep(0.5)
    
    def get_gripper_open_position(self):
        return self.gripper_limits['open']

    def gripper_close(self):
        val = self.gripper_limits['close']
        self.driver.write_position(6, val)
        time.sleep(0.5)
    
    def get_gripper_close_position(self):
        return self.gripper_limits['close']
    
    def torque_enable(self, enable=True):
        """Enable or disable torque on all motors"""
        for i in range(1, 7):
            self.driver.torque_enable(i, enable)
    
    def getCalibrationvalues(self):
        return {
            "offsets": self.offsets,
            "directions": self.directions,
            "adjustments": self.adjustments
        }

    def close(self):
        self.driver.close()