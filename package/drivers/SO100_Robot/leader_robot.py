#!/usr/bin/env python3
"""
SO100 Leader Robot Driver for cartesian control


This class handles communication with the SO100 Leader robot,
reads joint angles, and computes Cartesian positions using kinematics.
It also integrates with IKPy for kinematics calculations.
"""

import time
import os
import math
import csv
import numpy as np
from pathlib import Path

try:
    from so100_core.kinematics import SO100Kinematics
except ImportError:
    import sys
    sys.path.append(os.path.join(os.path.dirname(__file__), 'so100_core'))
    from kinematics import SO100Kinematics

class SO100LeaderToCartesianControl:
    def __init__(self, port, calibration_file_name="SO100leader_to_cartesian_calibration.csv", gripper_calibration_values = "gripper_values.csv", config_dir=None):
        print(f"[LEADER] Connecting to: {port}")
        
        # Use DIRECT serial connection like SO100Robot, not STS3215Driver
        self.ser = None
        self.simulation = False
        if port:
            try:
                import serial
                self.ser = serial.Serial(port, 1000000, timeout=0.1)
                print(f"[LEADER] Connected to: {port}")
                
                # Test connection by reading motor 1
                test_read = self._read_raw_position(1)
                if test_read is None:
                    print(f"[LEADER] [ERROR] No response from motors on {port}")
                    self.ser.close()
                    self.simulation = True
                else:
                    print(f"[LEADER] [SUCCESS] Connection test successful (Motor 1: {test_read})")
            except Exception as e:
                print(f"[LEADER] Connection failed to {port}: {e}")
                self.simulation = True
        else:
            self.simulation = True
        
        if self.simulation:
            print(f"[LEADER] [CRITICAL ERROR] Could not connect to port {port}!")
            print("[LEADER] Leader robot is in SIMULATION mode - teleoperation will NOT work!")
            raise Exception(f"Leader robot failed to connect to {port}")
        
        # --- Config and URDF search ---
        if config_dir is None:
            # Default: 'config' folder next to this file
            config_dir = os.path.join(os.path.dirname(__file__), "config")
            
        print(f"[LEADER] Config folder: {config_dir}")
        urdf_path = os.path.join(config_dir, "so100.urdf")
        
        # Load kinematics (Same URDF as the Follower!)
        self.kinematics = SO100Kinematics(urdf_path)
        
        # Default values
        self.offsets = {i: 2048 for i in range(6)}
        self.directions = {i: 1 for i in range(6)}
        self.adjustments = {i: 0.0 for i in range(6)}  # Add calibration adjustments
        
        # --- Load your special calibration ---
        # The calibrate_leader_candle.py creates this file:
        calib_filename = calibration_file_name
        calib_file = os.path.join(config_dir, calib_filename)
        
        if os.path.exists(calib_file):
             print(f"[LEADER] Loading your calibration: {calib_filename}")
             self._load_calibration(calib_file)
        else:
             print(f"[LEADER] [WARNING]: Cannot find '{calib_filename}'!")
             print("Please run 'calibrate_leader_candle.py' first!")
             print("Using follower calibration as a fallback.")
             fallback = os.path.join(config_dir, "follower_calibration.csv")
             self._load_calibration(fallback)
        
        # Gripper limits (will be loaded from config)
        self.gripper_limits = {'open': 2000, 'close': 1500}  # defaults
        
        # Load gripper configuration
        gripper_config = os.path.join(config_dir, gripper_calibration_values)
        self._load_gripper_config(gripper_config)

    def _checksum(self, data):
        """STS3215 Checksum calculation (original working method)"""
        return (~sum(data)) & 0xFF

    def _read_raw_position(self, motor_id):
        """Read raw position from motor (original working method)"""
        if getattr(self, 'simulation', False): 
            return 2048
        
        msg = [0xFF, 0xFF, motor_id, 0x04, 0x02, 0x38, 0x02]
        msg.append(self._checksum(msg[2:]))
        self.ser.reset_input_buffer()
        self.ser.write(bytearray(msg))
        response = self.ser.read(8)
        if len(response) == 8:
            import struct
            return struct.unpack('<H', response[5:7])[0]
        return None

    def _load_gripper_config(self, filepath):
        """Load gripper open/close values from CSV file"""
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        if len(row) >= 2:
                            if row[0] == 'GRIPPER_OPEN': 
                                self.gripper_limits['open'] = int(row[1])
                            elif row[0] == 'GRIPPER_CLOSE': 
                                self.gripper_limits['close'] = int(row[1])
                print(f"[LEADER] Gripper config loaded: {self.gripper_limits}")
            except Exception as e:
                print(f"[LEADER] [WARNING] Gripper config load failed: {e}")
        else:
            print(f"[LEADER] [WARNING] Gripper config not found: {filepath}")

    def _load_calibration(self, filepath):
        """Load calibration data from CSV file"""
        try:
            with open(filepath, 'r', encoding='utf-8-sig') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row: continue
                    key = row[0].strip()
                    if key == 'ZERO_OFFSETS':
                        # IDs from 0 to 5
                        self.offsets = {i: int(val) for i, val in enumerate(row[1:])}
                    elif key == 'DIRECTIONS':
                        self.directions = {i: int(val) for i, val in enumerate(row[1:])}
                    elif key == 'CALIBRATION_POSE_ADJUSTMENTS':
                        self.adjustments = {i: float(val) for i, val in enumerate(row[1:])}
            print(f"[LEADER] Offsets: {self.offsets}")
            print(f"[LEADER] Directions: {self.directions}")
            print(f"[LEADER] Adjustments: {self.adjustments}")
        except Exception as e:
            print(f"[LEADER] [ERROR] Calibration error: {e}")

    def torque_disable(self):
        """Disables the motors so you can move them by hand"""
        print("[LEADER] Releasing motors (Torque OFF)...")
        for i in range(1, 7):
            # Use direct serial communication like SO100Robot
            msg = [0xFF, 0xFF, i, 0x04, 0x03, 0x18, 0x00]  # Torque disable command
            msg.append(self._checksum(msg[2:]))
            self.ser.write(bytearray(msg))
            time.sleep(0.01)

    def get_joint_angles(self):
        """
        Reads the current joint angles in radians.
        IMPORTANT: At the 'Candle' position (where you calibrated), this should return [0,0,0,0,0]!
        """
        angles = []
        for i in range(6):
            raw = self._read_raw_position(i + 1)  # Use direct method
            if raw is None:
                angles.append(0.0)
                continue
            
            offset = self.offsets.get(i, 2048)
            direction = self.directions.get(i, 1)
            adjustment = self.adjustments.get(i, 0.0)  # Add calibration adjustment
            ratio = (2 * math.pi / 4096)
            
            if direction == 0: direction = 1
            
            # MATH: Same as in the Follower 'robot.py', just reversed (Raw -> Rad)
            # Follower: raw = ((rad - adjust) / (ratio * dir)) + offset
            # Leader (reverse): rad = (raw - offset) * (ratio * direction) + adjust
            
            rad = (raw - offset) * (ratio * direction) + adjustment
            angles.append(rad)
            
        return angles

    def get_cartesian_pose(self):
        """
        Returns the XYZ position and the Gripper state.
        This is what we send to the Follower!
        """
        joints = self.get_joint_angles()
        
        # The first 5 motors determine the position
        arm_joints = joints[:5]
        
        # Forward Kinematics (FK)
        # This function comes from kinematics.py and tells us 
        # where the hand is in space based on the current angles.
        xyz = self.kinematics.forward_kinematics(arm_joints)
        
        # Gripper state (0.0 = closed, 1.0 = open)
        raw_gripper = self._read_raw_position(6)  # Use direct method
        if raw_gripper is None: raw_gripper = self.gripper_limits['close']
        
        # Normalize between 0 and 1 using loaded config values
        grip_norm = (raw_gripper - self.gripper_limits['close']) / (self.gripper_limits['open'] - self.gripper_limits['close'])
        # Clamp between 0 and 1
        grip_norm = max(0.0, min(1.0, grip_norm))
        
        return xyz, grip_norm

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# Demo/Test when run directly
if __name__ == "__main__":
    print("\n=== SO100 Leader Robot Demo ===")
    print("This script demonstrates the Leader Robot functionality.")
    print("Note: Run calibrate_leader_candle.py first for proper calibration!\n")
    
    # Test with simulation (no port)
    try:
        print("[DEMO] Testing class initialization...")
        leader = SO100LeaderToCartesianControl(port=None)  # This will simulate
        print("[DEMO] OK\tClass created successfully!")
        print("[DEMO] OK\tConfiguration loaded!")
        print("[DEMO] OK\tKinematics initialized!")
        
        # Show current config
        print(f"\n[DEMO] Gripper limits: {leader.gripper_limits}")
        print(f"[DEMO] Calibration offsets: {leader.offsets}")
        print(f"[DEMO] Direction multipliers: {leader.directions}")
        
        leader.close()
        print("\n[DEMO] OK\tTest completed successfully!")
        print("[DEMO] Ready for real robot connection.")
        
    except Exception as e:
        print(f"[DEMO] [FAIL]\t Error during demo: {e}")