"""
SO100 Robot Core Module
Handles robot initialization, kinematics, and motor control.
Created by Copilot based on Sandor Burian's summarization of so100_robot.py.
"""
import time
import math
import os
import csv
import numpy as np
from .sts3215 import STS3215Driver
from .kinematics import SO100Kinematics

class SO100Robot:
    def __init__(self, port, calibration_file="follower_calibration.csv", gripper_configuration_values="gripper_values.csv", config_dir=None):
        # Initialize DIRECT serial connection like original
        self.ser = None
        self.port = port  # Store port for reconnection
        if port:
            try:
                import serial
                self.ser = serial.Serial(port, 115200, timeout=3.0)  # Slower baud rate for reliability
                print(f"[SO100] Connected to: {port}")
            except Exception as e:
                print(f"[SO100] Connection failed: {e}")
                self.simulation = True
        else:
            self.simulation = True
        
        # --- PATH SEARCH ---
        if config_dir is None:
            current_file_path = os.path.abspath(__file__)
            current_dir = os.path.dirname(current_file_path)
            config_dir = os.path.join(current_dir, "config")
        else:
            # If config_dir is specified, check if it's relative
            if not os.path.isabs(config_dir):
                # If relative, start from SO100_Robot directory
                so100_robot_dir = os.path.dirname(os.path.dirname(__file__))  # SO100_Robot folder
                config_dir = os.path.join(so100_robot_dir, config_dir)
            
        print(f"[DEBUG] Config folder location: {config_dir}")

        urdf_path = os.path.join(config_dir, "so100.urdf")
        if not os.path.exists(urdf_path):
            print(f"[WARN] URDF not found: {urdf_path}")
            print("[INFO] Fallback: using SO100_Robot/config...")
            so100_robot_dir = os.path.dirname(os.path.dirname(__file__))
            config_dir = os.path.join(so100_robot_dir, "config")
            urdf_path = os.path.join(config_dir, "so100.urdf")
            print(f"[DEBUG] New URDF path: {urdf_path}")
        
        self.kinematics = SO100Kinematics(urdf_path)
        
        # Default values
        self.offsets = {i: 2048 for i in range(6)}
        self.directions = {i: 1 for i in range(6)}
        self.adjustments = {i: 0.0 for i in range(6)}
        
        self._load_calibration(os.path.join(config_dir, calibration_file))
        
        self.gripper_limits = {'open': 2000, 'close': 1500}
        self._load_gripper_config(os.path.join(config_dir, gripper_configuration_values))
        
        self.current_joint_state = [0.0] * 6 

    def _checksum(self, data):
        """STS3215 Checksum calculation (original working method)"""
        return (~sum(data)) & 0xFF

    def _write_packet(self, motor_id, instruction, parameters):
        """Send packet to motor (original working method)"""
        if not self.ser or getattr(self, 'simulation', False):
            return
        
        # Protocol: [FF, FF, ID, LEN, INSTR, P1, P2..., CHK]
        length = len(parameters) + 2
        packet = [0xFF, 0xFF, motor_id, length, instruction] + parameters
        checksum = self._checksum(packet[2:])
        packet.append(checksum)
        
        try:
            if self.ser and hasattr(self.ser, 'reset_input_buffer'):
                self.ser.reset_input_buffer()
            self.ser.write(bytearray(packet))
            time.sleep(0.001)  # Increased timing for reliability
        except Exception as e:
            print(f"[SO100] Write error: {e}")
    
    def hard_reset_connection(self):
        """Aggressive connection reset - simulates power cycle"""
        if getattr(self, 'simulation', False):
            return True
            
        print("[SO100] Performing hard reset...")
        try:
            if self.ser:
                self.ser.close()
                time.sleep(1.0)  # Wait for hardware to reset
            
            import serial
            port = getattr(self, 'port', None)
            if port:
                self.ser = serial.Serial(port, 115200, timeout=3.0)  # Use slower baud rate for reliability
                time.sleep(0.5)  # Let connection stabilize
                
                # Send reset commands to all motors
                for motor_id in range(1, 7):
                    try:
                        self._write_packet(motor_id, 0x03, [0x28, 0x00])  # Torque disable
                        time.sleep(0.05)
                        self._write_packet(motor_id, 0x03, [0x28, 0x01])  # Torque enable
                        time.sleep(0.05)
                    except:
                        pass
                
                print("[SO100] Hard reset completed successfully")
                return True
        except Exception as e:
            print(f"[SO100] Hard reset failed: {e}")
            return False
        return False

    def _load_calibration(self, filepath):
        print(f"Loading calibration from: {filepath}")
        try:
            with open(filepath, 'r', encoding='utf-8-sig') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row: continue
                    key = row[0].strip()
                    if key == 'ZERO_OFFSETS':
                        self.offsets = {i: int(val) for i, val in enumerate(row[1:])}
                    elif key == 'DIRECTIONS':
                        self.directions = {i: int(val) for i, val in enumerate(row[1:])}
                    elif key == 'CALIBRATION_POSE_ADJUSTMENTS':
                        vals = []
                        for val in row[1:]:
                            if val.strip(): vals.append(float(val))
                            else: vals.append(0.0)
                        self.adjustments = {i: v for i, v in enumerate(vals)}
            
            # DEBUG OUTPUT
            print(f"[DEBUG] Offsets: {self.offsets}")
            print(f"[DEBUG] Directions: {self.directions}")
            print(f"[DEBUG] Adjustments: {self.adjustments}")

        except Exception as e:
            print(f"WARN: Calibration load failed ({e}). Using defaults.")

    def _load_gripper_config(self, filepath):
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        if row[0] == 'GRIPPER_OPEN': self.gripper_limits['open'] = int(row[1])
                        elif row[0] == 'GRIPPER_CLOSE': self.gripper_limits['close'] = int(row[1])
            except: pass

    def _angle_to_raw(self, motor_index, radians):
        """
        RESTORED TO ORIGINAL (so100_control_driver.py) LOGIC!
        """
        offset = self.offsets.get(motor_index, 2048)
        direction = self.directions.get(motor_index, 1)
        calib = self.adjustments.get(motor_index, 0.0)
        
        # ORIGINAL FORMULA:
        # raw_val = ((angle_radians - calib) / (ratio * direction)) + offset
        
        ratio = (2 * math.pi / 4096)
        
        # Note the formula: subtraction and division (my version had multiplication)
        raw_val = ((radians - calib) / (ratio * direction)) + offset
        
        return int(raw_val)

    def get_joint_angles(self):
        """Get current joint angles in radians (original working method)"""
        if getattr(self, 'simulation', False):
            return [0.0] * 6

        angles = []
        for i in range(6): 
            raw = self._read_raw_position(i + 1)
            if raw is None:
                angles.append(0.0)
                continue
            
            # Reverse calculation based on original logic
            offset = self.offsets.get(i, 2048)
            direction = self.directions.get(i, 1)
            calib = self.adjustments.get(i, 0.0)
            ratio = (2 * math.pi / 4096)
            
            # (raw - offset) * ratio * dir + calib
            val = (raw - offset) * (ratio * direction) + calib
            angles.append(val)
        return angles

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

    def set_target_angle(self, motor_index, angle_radians, move_time_ms=1000):
        """
        Move single motor to specific angle (DIRECT SERIAL like original working driver)
        motor_index: 0-5 (0 = motor 1)
        angle_radians: target angle in radians
        move_time_ms: movement time in milliseconds
        """
        motor_id = motor_index + 1
        raw_val = self._angle_to_raw(motor_index, angle_radians)
        
        # 1. Torque enable (original working protocol)
        self._write_packet(motor_id, 0x03, [0x28, 0x01])
        
        # 2. Position command (EXACT original working protocol)
        p_low = raw_val & 0xFF
        p_high = (raw_val >> 8) & 0xFF
        t_low = move_time_ms & 0xFF
        t_high = (move_time_ms >> 8) & 0xFF
        s_low = 0; s_high = 0  # Speed 0 = Max speed (original working code!)
        
        # EXACT original 7-byte protocol: [0x2A, p_low, p_high, t_low, t_high, s_low, s_high]
        params = [0x2A, p_low, p_high, t_low, t_high, s_low, s_high]
        self._write_packet(motor_id, 0x03, params)

    def move_to_joints(self, joint_angles, time_ms=200):  
        """
        ORIGINAL LOGIC: each motor separately, like in the working code
        """
        for i, angle in enumerate(joint_angles):
            if i == 5: continue  # Gripper handled separately
            
            motor_id = i + 1
            raw_val = self._angle_to_raw(i, angle)
            
            # 1. Torque enable (original)
            self._write_packet(motor_id, 0x03, [0x28, 0x01])
            
            # 2. Position command (original 7-byte protocol)
            p_low = raw_val & 0xFF
            p_high = (raw_val >> 8) & 0xFF
            t_low = time_ms & 0xFF
            t_high = (time_ms >> 8) & 0xFF
            s_low = 0  # Max speed
            s_high = 0
            
            # ORIGINAL protocol
            params = [0x2A, p_low, p_high, t_low, t_high, s_low, s_high]
            self._write_packet(motor_id, 0x03, params)
            
            # ORIGINAL: 0.05s sleep after each motor (this was in the working code!)
            time.sleep(0.05)
            
        self.current_joint_state = list(joint_angles)
        
        # Wait for movement to complete
        time.sleep(time_ms / 1000.0)

    def move_to_cartesian(self, x, y, z,time_ms=200, ignore_orientation=True):  # Faster default
        seed = [0] + self.current_joint_state[:5] + [0] 
        
        # Direct coordinates without inversion - test if this fixes coordination
        # Previously: corrected_y = -y (this might be causing the mismatch)
        
        # IKPy call (now with the fixed kinematics.py)
        orient_mode = None if ignore_orientation else "Z"
        target_orient = None if ignore_orientation else [0, 0, -1]

        target_joints = self.kinematics.inverse_kinematics(
            target_pos=[x, y, z],
            target_orient=target_orient,
            orientation_mode=orient_mode,
            seed_state=seed
        )
        
        motor_commands = target_joints[1:6]
        
        # ORIGINAL METHOD: motor-by-motor programming like the working demo!
        print("Moving...")
        for motor_idx in range(5):  # 0..4 (Motor 1..5)
            ik_angle = motor_commands[motor_idx] 
            
            # Just like the original demo: set_target_angle for each motor separately
            motor_id = motor_idx + 1
            raw_val = self._angle_to_raw(motor_idx, ik_angle)
            
            # 1. Torque enable (original)
            self._write_packet(motor_id, 0x03, [0x28, 0x01])
            
            # 2. Position command (original protocol)
            p_low = raw_val & 0xFF
            p_high = (raw_val >> 8) & 0xFF
            t_low = time_ms & 0xFF
            t_high = (time_ms >> 8) & 0xFF
            s_low = 0; s_high = 0
            
            params = [0x2A, p_low, p_high, t_low, t_high, s_low, s_high]
            self._write_packet(motor_id, 0x03, params)
            
            # ORIGINAL: 0.05s sleep after each motor!
            time.sleep(0.05)
        
        # Wait for movement to complete (plus small buffer)
        time.sleep(time_ms / 1000.0 + 0.1)

    def gripper_open(self):
        """Open gripper (EXACT original working gripper_handler.py protocol)"""
        if not getattr(self, 'simulation', False):
            val = self.gripper_limits['open']
            print(f"[DEBUG] Opening gripper to raw value: {val}")
            
            # EXACT original protocol from gripper_handler.py
            # [0x2A, 0x00, val & 0xFF, (val >> 8) & 0xFF, 0x00, 0x00, 0x00, 0x00]
            params = [0x2A, 0x00, val & 0xFF, (val >> 8) & 0xFF, 0x00, 0x00, 0x00, 0x00]
            print(f"[DEBUG] Gripper packet (8 bytes): {params}")
            
            self._write_packet(6, 0x03, params)
        time.sleep(1.0)  # Original wait time from gripper_handler

    def gripper_close(self):
        """Close gripper (EXACT original working gripper_handler.py protocol)"""
        if not getattr(self, 'simulation', False):
            val = self.gripper_limits['close']
            print(f"[DEBUG] Closing gripper to raw value: {val}")
            
            # EXACT original protocol from gripper_handler.py
            # [0x2A, 0x00, val & 0xFF, (val >> 8) & 0xFF, 0x00, 0x00, 0x00, 0x00]
            params = [0x2A, 0x00, val & 0xFF, (val >> 8) & 0xFF, 0x00, 0x00, 0x00, 0x00]
            print(f"[DEBUG] Gripper packet (8 bytes): {params}")
            
            self._write_packet(6, 0x03, params)
        time.sleep(1.0)  # Original wait time from gripper_handler
    
    def torque_enable(self, enable=True):
        """Enable/disable motor torque (original working method)"""
        for i in range(1, 7):
            val = 1 if enable else 0
            self._write_packet(i, 0x03, [0x28, val])
            time.sleep(0.002)  # Much faster than 0.01

    def getCalibrationvalues(self):
        """Return current calibration values"""
        return {
            "offsets": self.offsets,
            "directions": self.directions,
            "adjustments": self.adjustments,
            "gripper_limits": self.gripper_limits
        }

    def get_cartesian_position(self):
        """
        Get current cartesian position using forward kinematics
        Returns [x, y, z] coordinates in meters
        """
        current_joints = self.get_joint_angles()
        if current_joints:
            # Use first 5 joints for position (exclude gripper)
            arm_joints = current_joints[:5]
            xyz = self.kinematics.forward_kinematics(arm_joints)
            return xyz
        return [0.0, 0.0, 0.0]
    
    def get_cartesian_pose(self):
        """
        Get current cartesian position and gripper state
        Returns xyz coordinates and gripper normalized value (0.0-1.0)
        """
        xyz = self.get_cartesian_position()
        
        # Read gripper position and normalize
        raw_gripper = self._read_raw_position(6)
        if raw_gripper is None: 
            raw_gripper = self.gripper_limits['close']
        
        # Normalize between 0 and 1
        grip_norm = (raw_gripper - self.gripper_limits['close']) / (self.gripper_limits['open'] - self.gripper_limits['close'])
        grip_norm = max(0.0, min(1.0, grip_norm))  # Clamp to [0,1]
        
        return xyz, grip_norm
    
    def get_current_angles(self):
        """
        Alias for get_joint_angles for compatibility
        """
        return self.get_joint_angles()
    
    @property
    def calibration_offsets(self):
        """Return offsets as list for compatibility"""
        return [self.offsets[i] for i in range(6)]
    
    @property
    def calibration_directions(self):
        """Return directions as list for compatibility"""
        return [self.directions[i] for i in range(6)]
    
    @property
    def calibration_adjustments(self):
        """Return adjustments as list for compatibility"""
        return [self.adjustments[i] for i in range(6)]

    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
