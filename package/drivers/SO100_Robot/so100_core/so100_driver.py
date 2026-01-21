#!/usr/bin/env python3
"""
SO100 Robot Control Driver (Simplified)
Based on the original working driver by Sandor Burian
Adapted for the new package structure while keeping the working logic
"""

import os
import serial
import time
import struct
import math
import csv
import ikpy.chain

class SO100Robot:
    def __init__(self, port=None, config_dir=None, calibration_file="follower_calibration.csv", simulation=False):
        """
        Initialize SO100 Robot with simplified configuration
        """
        self.simulation = simulation
        self.port = port
        
        # Find config directory
        if config_dir is None:
            # Default to SO100_Robot/config
            script_dir = os.path.dirname(__file__)  # so100_core directory
            so100_dir = os.path.dirname(script_dir)  # SO100_Robot directory
            config_dir = os.path.join(so100_dir, "config")
        elif not os.path.isabs(config_dir):
            # Relative path - from SO100_Robot directory
            script_dir = os.path.dirname(__file__)
            so100_dir = os.path.dirname(script_dir)
            config_dir = os.path.join(so100_dir, config_dir)
            
        print(f"[SO100] Config directory: {config_dir}")
        
        # Load URDF
        urdf_path = os.path.join(config_dir, "so100.urdf")
        if os.path.exists(urdf_path):
            self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["base"])
            print(f"[SO100] URDF loaded: {urdf_path}")
        else:
            raise FileNotFoundError(f"URDF not found: {urdf_path}")
        
        # Load calibration
        calib_path = os.path.join(config_dir, calibration_file)
        self._load_calibration(calib_path)
        
        # Motor communication
        self.ser = None
        if not simulation and port:
            try:
                self.ser = serial.Serial(port, 1000000, timeout=0.1)
                print(f"[SO100] Connected to: {port}")
            except Exception as e:
                print(f"[SO100] Connection failed: {e}")
                self.simulation = True
        
        # State tracking
        self.current_joint_state = [0.0] * 6
        
        # Gripper limits  
        self.gripper_limits = {'open': 2000, 'close': 1500}
        self._load_gripper_config(os.path.join(config_dir, "gripper_values.csv"))

    def _load_calibration(self, calib_path):
        """Load calibration from CSV file (original working logic)"""
        try:
            with open(calib_path, 'r') as file:
                reader = csv.reader(file)
                print(f"[SO100] Loading calibration: {calib_path}")
                for row in reader:
                    if len(row) > 1:
                        param_name = row[0]
                        values = [float(x) if '.' in x else int(x) for x in row[1:]]
                        if param_name == 'ZERO_OFFSETS': 
                            self.ZERO_OFFSETS = values
                        elif param_name == 'DIRECTIONS': 
                            self.DIRECTIONS = values
                        elif param_name == 'CALIBRATION_POSE_ADJUSTMENTS': 
                            self.CALIBRATION_POSE_ADJUSTMENTS = values
                            
                print(f"[SO100] Calibration loaded successfully")
                print(f"  Offsets: {self.ZERO_OFFSETS}")
                print(f"  Directions: {self.DIRECTIONS}")
                print(f"  Adjustments: {self.CALIBRATION_POSE_ADJUSTMENTS}")
                            
        except FileNotFoundError:
            print(f"[SO100] ERROR: Calibration file not found: {calib_path}")
            print("[SO100] Using default calibration values (may cause incorrect movement)")
            # Safe defaults
            self.ZERO_OFFSETS = [2048] * 6
            self.DIRECTIONS = [1] * 6
            self.CALIBRATION_POSE_ADJUSTMENTS = [0.0] * 6

    def _load_gripper_config(self, gripper_path):
        """Load gripper configuration"""
        try:
            with open(gripper_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if row[0] == 'GRIPPER_OPEN': self.gripper_limits['open'] = int(row[1])
                    elif row[0] == 'GRIPPER_CLOSE': self.gripper_limits['close'] = int(row[1])
                print(f"[SO100] Gripper config: {self.gripper_limits}")
        except FileNotFoundError:
            print("[SO100] Gripper config not found, using defaults")

    def _checksum(self, data):
        """STS3215 Checksum calculation"""
        return (~sum(data)) & 0xFF

    def _write_packet(self, motor_id, instruction, parameters):
        """Send packet to motor (original working logic)"""
        if self.simulation: 
            return
        
        # Protocol: [FF, FF, ID, LEN, INSTR, P1, P2..., CHK]
        length = len(parameters) + 2
        packet = [0xFF, 0xFF, motor_id, length, instruction] + parameters
        checksum = self._checksum(packet[2:])
        packet.append(checksum)
        
        try:
            self.ser.write(bytearray(packet))
            time.sleep(0.0002) 
        except Exception as e:
            print(f"[SO100] Write error: {e}")

    def set_target_angle(self, motor_index, angle_radians, move_time_ms=1000):
        """
        Move single motor to specific angle (original working method)
        motor_index: 0-5 (0 = motor 1)
        angle_radians: target angle in radians
        move_time_ms: movement time in milliseconds
        """
        if self.simulation: 
            return

        motor_id = motor_index + 1
        
        # 1. Enable torque first (CRITICAL for movement!)
        self._write_packet(motor_id, 0x03, [0x28, 0x01])

        # 2. Angle conversion (original working formula)
        offset = self.ZERO_OFFSETS[motor_index]
        direction = self.DIRECTIONS[motor_index]
        calib = self.CALIBRATION_POSE_ADJUSTMENTS[motor_index]
        
        ratio = (2 * math.pi / 4096)
        raw_val = ((angle_radians - calib) / (ratio * direction)) + offset
        raw_int = int(raw_val)
        raw_int = max(0, min(4095, raw_int))  # Safety clamp
        
        # 3. Position command (original working protocol)
        p_low = raw_int & 0xFF
        p_high = (raw_int >> 8) & 0xFF
        t_low = move_time_ms & 0xFF
        t_high = (move_time_ms >> 8) & 0xFF
        
        params = [0x2A, p_low, p_high, t_low, t_high, 0, 0]
        self._write_packet(motor_id, 0x03, params)

    def move_to_joints(self, joint_angles, time_ms=1000):
        """
        Move to joint angles with motor-by-motor control (original working logic)
        joint_angles: List of 6 angles in radians
        time_ms: Movement time in milliseconds
        """
        if len(joint_angles) < 6:
            print(f"[SO100] ERROR: Need 6 joint angles, got {len(joint_angles)}")
            return
            
        # ORIGINAL WORKING METHOD: Motor-by-motor with delays
        for i, target_rad in enumerate(joint_angles):
            if i >= 6: break
            
            self.set_target_angle(i, target_rad, time_ms)
            
            # CRITICAL: 0.05s delay between motors (from original working code)
            time.sleep(0.05)
        
        # Wait for movement completion
        time.sleep(time_ms / 1000.0)
        
        # Update internal state
        self.current_joint_state = list(joint_angles)

    def read_raw_position(self, motor_id):
        """
        Read raw position from motor (original working method)
        """
        if self.simulation: 
            return 2048
        
        msg = [0xFF, 0xFF, motor_id, 0x04, 0x02, 0x38, 0x02]
        msg.append(self._checksum(msg[2:]))
        self.ser.reset_input_buffer()
        self.ser.write(bytearray(msg))
        response = self.ser.read(8)
        if len(response) == 8:
            return struct.unpack('<H', response[5:7])[0]
        return None

    def get_joint_angles(self):
        """
        Get current joint angles in radians (original working method)
        """
        if self.simulation:
            return [0.0] * 6

        angles = []
        for i in range(6):  # Motors 1-6
            raw = self.read_raw_position(i + 1)
            if raw is None: 
                raw = self.ZERO_OFFSETS[i]

            # Convert raw to radians (original working formula)
            offset = self.ZERO_OFFSETS[i]
            direction = self.DIRECTIONS[i]
            ratio = (2 * math.pi / 4096)
            rel_rad = (raw - offset) * ratio * direction
            final_rad = rel_rad + self.CALIBRATION_POSE_ADJUSTMENTS[i]
            angles.append(final_rad)
            
        return angles

    def move_to_cartesian(self, x, y, z, time_ms=1500):
        """
        Move to cartesian position using motor-by-motor control (original working method)
        """
        # IK calculation with seed from current state
        seed = [0] + self.current_joint_state[:5] + [0]
        
        target_joints = self.chain.inverse_kinematics(
            target_position=[x, y, z],
            initial_position=seed
        )
        
        # Extract motor angles (skip base and end effector)
        motor_commands = target_joints[1:6]
        
        # ORIGINAL WORKING METHOD: Motor-by-motor control
        print(f"[SO100] Moving to cartesian ({x:.2f}, {y:.2f}, {z:.2f})")
        for i, angle in enumerate(motor_commands):
            self.set_target_angle(i, angle, time_ms)
            # CRITICAL: 0.05s delay between motors (from original demo)
            time.sleep(0.05)
        
        # Wait for movement completion + buffer
        time.sleep(time_ms / 1000.0 + 0.1)
        
        # Update state
        self.current_joint_state = list(motor_commands) + [0]

    def gripper_open(self):
        """Open gripper (original working method)"""
        val = self.gripper_limits['open']
        if not self.simulation:
            # Enable torque first
            self._write_packet(6, 0x03, [0x28, 0x01])
            # Position command
            p_low = val & 0xFF
            p_high = (val >> 8) & 0xFF
            params = [0x2A, p_low, p_high, 200, 0, 0, 0]  # 200ms movement
            self._write_packet(6, 0x03, params)
        time.sleep(0.3)

    def gripper_close(self):
        """Close gripper (original working method)"""
        val = self.gripper_limits['close']
        if not self.simulation:
            # Enable torque first
            self._write_packet(6, 0x03, [0x28, 0x01])
            # Position command
            p_low = val & 0xFF
            p_high = (val >> 8) & 0xFF
            params = [0x2A, p_low, p_high, 200, 0, 0, 0]  # 200ms movement
            self._write_packet(6, 0x03, params)
        time.sleep(0.3)

    def torque_enable(self, enable=True):
        """Enable/disable motor torque (original working method)"""
        for i in range(1, 7):
            val = 1 if enable else 0
            self._write_packet(i, 0x03, [0x28, val])
            time.sleep(0.01)  # Small delay between motors

    def torque_disable(self):
        """Disable all motor torque (from original working code)"""
        for i in range(1, 7):
            self._write_packet(i, 0x03, [0x28, 0x00])
            time.sleep(0.01)

    def getCalibrationvalues(self):
        """Get current calibration values"""
        return {
            "offsets": {i: self.ZERO_OFFSETS[i] for i in range(6)},
            "directions": {i: self.DIRECTIONS[i] for i in range(6)}, 
            "adjustments": {i: self.CALIBRATION_POSE_ADJUSTMENTS[i] for i in range(6)},
            "gripper_limits": self.gripper_limits
        }

    def close(self):
        """Close serial connection"""
        if self.ser:
            self.ser.close()
    
    def __del__(self):
        self.close()
        print("SO100Robot driver closed.")
    