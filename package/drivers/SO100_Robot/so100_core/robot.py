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
from .sts3215 import STS3215Driver
from .kinematics import SO100Kinematics

class SO100Robot:
    def __init__(self, port, config_dir=None):
        # Initialize DIRECT serial connection like original
        self.ser = None
        if port:
            try:
                import serial
                self.ser = serial.Serial(port, 1000000, timeout=0.1)
                print(f"[SO100] Connected to: {port}")
            except Exception as e:
                print(f"[SO100] Connection failed: {e}")
                self.simulation = True
        else:
            self.simulation = True
        
        # --- ÚTVONAL KERESÉS ---
        if config_dir is None:
            current_file_path = os.path.abspath(__file__)
            current_dir = os.path.dirname(current_file_path)
            config_dir = os.path.join(current_dir, "config")
        else:
            # Ha meg van adva config_dir, ellenőrizzük hogy relatív-e
            if not os.path.isabs(config_dir):
                # Ha relatív, akkor a SO100_Robot könyvtárból induljon
                so100_robot_dir = os.path.dirname(os.path.dirname(__file__))  # SO100_Robot mappa
                config_dir = os.path.join(so100_robot_dir, config_dir)
            
        print(f"[DEBUG] Config mappa helye: {config_dir}")

        urdf_path = os.path.join(config_dir, "so100.urdf")
        if not os.path.exists(urdf_path):
            print(f"[WARN] URDF nem található: {urdf_path}")
            print("[INFO] Fallback: SO100_Robot/config használata...")
            so100_robot_dir = os.path.dirname(os.path.dirname(__file__))
            config_dir = os.path.join(so100_robot_dir, "config")
            urdf_path = os.path.join(config_dir, "so100.urdf")
            print(f"[DEBUG] Új URDF path: {urdf_path}")
        
        self.kinematics = SO100Kinematics(urdf_path)
        
        # Alapértékek
        self.offsets = {i: 2048 for i in range(6)}
        self.directions = {i: 1 for i in range(6)}
        self.adjustments = {i: 0.0 for i in range(6)}
        
        self._load_calibration(os.path.join(config_dir, "follower_calibration.csv"))
        
        self.gripper_limits = {'open': 2000, 'close': 1500}
        self._load_gripper_config(os.path.join(config_dir, "gripper_values.csv"))
        
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
            self.ser.write(bytearray(packet))
            time.sleep(0.0002)  # Original working timing
        except Exception as e:
            print(f"[SO100] Write error: {e}")

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
            
            # DEBUG KIÍRÁS
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
        VISSZAÁLLÍTVA AZ EREDETI (so100_control_driver.py) LOGIKÁRA!
        """
        offset = self.offsets.get(motor_index, 2048)
        direction = self.directions.get(motor_index, 1)
        calib = self.adjustments.get(motor_index, 0.0)
        
        # EREDETI KÉPLET:
        # raw_val = ((angle_radians - calib) / (ratio * direction)) + offset
        
        ratio = (2 * math.pi / 4096)
        
        # Figyeld a képletet: kivonás és osztás (az én verziómban szorzás volt)
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
            
            # Visszafejtés az eredeti logika alapján
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
        EREDETI LOGIKA: minden motor külön-külön, mint a working kódban
        """
        for i, angle in enumerate(joint_angles):
            if i == 5: continue  # Gripper külön kezelése
            
            motor_id = i + 1
            raw_val = self._angle_to_raw(i, angle)
            
            # 1. Torque enable (eredeti)
            self._write_packet(motor_id, 0x03, [0x28, 0x01])
            
            # 2. Pozíció parancs (eredeti 7-byte protokoll)
            p_low = raw_val & 0xFF
            p_high = (raw_val >> 8) & 0xFF
            t_low = time_ms & 0xFF
            t_high = (time_ms >> 8) & 0xFF
            s_low = 0  # Max speed
            s_high = 0
            
            # EREDETI protokoll
            params = [0x2A, p_low, p_high, t_low, t_high, s_low, s_high]
            self._write_packet(motor_id, 0x03, params)
            
            # EREDETI: 0.05s sleep minden motor után (ez volt a working kódban!)
            time.sleep(0.05)
            
        self.current_joint_state = list(joint_angles)
        
        # Wait for movement to complete
        time.sleep(time_ms / 1000.0)

    def move_to_cartesian(self, x, y, z, pitch_rad=0.0, roll_rad=0.0, time_ms=200, ignore_orientation=True):  # Faster default
        seed = [0] + self.current_joint_state[:5] + [0] 
        
        # COORDINATE SYSTEM FIX: If "left is forward", negate Y axis
        # Try: y = -y to fix orientation issue
        corrected_y = -y  # Experimental fix for coordinate system
        
        # IKPy hívás (most már a javított kinematics.py-val)
        orient_mode = None if ignore_orientation else "Z"
        target_orient = None if ignore_orientation else [0, 0, -1]

        target_joints = self.kinematics.inverse_kinematics(
            target_pos=[x, corrected_y, z],
            target_orient=target_orient,
            orientation_mode=orient_mode,
            seed_state=seed
        )
        
        motor_commands = target_joints[1:6]
        
        # EREDETI MÓDSZER: motor-by-motor programozás mint a working demo!
        print("Moving...")
        for motor_idx in range(5):  # 0..4 (Motor 1..5)
            ik_angle = motor_commands[motor_idx] 
            
            # Pont mint az eredeti demo: set_target_angle minden motorra külön
            motor_id = motor_idx + 1
            raw_val = self._angle_to_raw(motor_idx, ik_angle)
            
            # 1. Torque enable (eredeti)
            self._write_packet(motor_id, 0x03, [0x28, 0x01])
            
            # 2. Pozíció parancs (eredeti protokoll)
            p_low = raw_val & 0xFF
            p_high = (raw_val >> 8) & 0xFF
            t_low = time_ms & 0xFF
            t_high = (time_ms >> 8) & 0xFF
            s_low = 0; s_high = 0
            
            params = [0x2A, p_low, p_high, t_low, t_high, s_low, s_high]
            self._write_packet(motor_id, 0x03, params)
            
            # EREDETI: 0.05s sleep minden motor után!
            time.sleep(0.05)
        
        # Wait for movement to complete (plus small buffer)
        time.sleep(time_ms / 1000.0 + 0.1)

    def gripper_open(self):
        """Open gripper (original working method)"""
        val = self.gripper_limits['open']
        if not getattr(self, 'simulation', False):
            # Enable torque first
            self._write_packet(6, 0x03, [0x28, 0x01])
            # Position command
            p_low = val & 0xFF
            p_high = (val >> 8) & 0xFF
            params = [0x2A, p_low, p_high, 200, 0, 0, 0]  # 200ms movement
            self._write_packet(6, 0x03, params)
        time.sleep(0.2)

    def gripper_close(self):
        """Close gripper (original working method)"""
        val = self.gripper_limits['close']
        if not getattr(self, 'simulation', False):
            # Enable torque first
            self._write_packet(6, 0x03, [0x28, 0x01])
            # Position command
            p_low = val & 0xFF
            p_high = (val >> 8) & 0xFF
            params = [0x2A, p_low, p_high, 200, 0, 0, 0]  # 200ms movement
            self._write_packet(6, 0x03, params)
        time.sleep(0.2)
    
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

    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()