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
        self.driver = STS3215Driver(port)
        
        if config_dir is None:
            current_file_path = os.path.abspath(__file__)
            current_dir = os.path.dirname(current_file_path)
            config_dir = os.path.join(current_dir, "config")
        else:
            # Ha meg van adva config_dir, tegyük abszolúttá
            if not os.path.isabs(config_dir):
                # Ha relatív, akkor a SO100_Robot mappából induljon
                so100_robot_dir = os.path.dirname(os.path.dirname(__file__))
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
        
        self.offsets = {}
        self.directions = {}
        self.adjustments = {}
        
        self._load_calibration(os.path.join(config_dir, "follower_calibration.csv"))
        
        gripper_file = os.path.join(config_dir, "gripper_values.csv")
        self.gripper_limits = {'open': 2000, 'close': 1500}
        self._load_gripper_config(gripper_file)
        
        self.current_joint_state = [0.0] * 6 

    def _load_calibration(self, filepath):
        print(f"Loading calibration from: {filepath}")
        try:
            with open(filepath, 'r', encoding='utf-8-sig') as f: # utf-8-sig kezeli a BOM-ot!
                reader = csv.reader(f)
                for row in reader:
                    if not row: continue
                    # Javított tisztítás (.strip())
                    key = row[0].strip()
                    
                    if key == 'ZERO_OFFSETS':
                        self.offsets = {i: int(val) for i, val in enumerate(row[1:])}
                    elif key == 'DIRECTIONS':
                        self.directions = {i: int(val) for i, val in enumerate(row[1:])}
                    elif key == 'CALIBRATION_POSE_ADJUSTMENTS':
                        self.adjustments = {i: float(val) for i, val in enumerate(row[1:])}
            print(f"[DEBUG] Offsets loaded: {self.offsets}")
            print(f"[DEBUG] Directions loaded: {self.directions}")
            print(f"[DEBUG] Adjustments loaded: {self.adjustments}")
                        
        except Exception as e:
            print(f"[WARNING]: Calibration load failed ({e}). Using defaults.")

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
        corrected_angle = radians + self.adjustments.get(motor_index, 0.0)
        raw_scale = (corrected_angle / (2 * math.pi)) * 4096
        direction = self.directions.get(motor_index, 1)
        offset = self.offsets.get(motor_index, 2048)
        return int((raw_scale * direction) + offset)

    def get_joint_angles(self):
        angles = []
        for i in range(6): 
            raw = self.driver.read_position(i + 1)
            if raw is None:
                angles.append(0.0)
                continue
            offset = self.offsets.get(i, 2048)
            direction = self.directions.get(i, 1)
            val = (raw - offset) * direction
            rad = (val / 4096.0) * (2 * math.pi)
            final_rad = rad - self.adjustments.get(i, 0.0)
            angles.append(final_rad)
        return angles

    def move_to_joints(self, joint_angles, time_ms=1000):
        # ÚJ: SYNC WRITE HASZNÁLATA
        sync_data = []
        
        for i, angle in enumerate(joint_angles):
            if i == 5: continue # A gripper külön van
            
            raw_val = self._angle_to_raw(i, angle)
            # Motor ID: i+1
            sync_data.append((i + 1, raw_val, time_ms))
            
        # Egyetlen csomagban küldjük el az 5 motor parancsát
        self.driver.sync_write_pos_time(sync_data)
            
        self.current_joint_state = list(joint_angles)

    def move_to_cartesian(self, x, y, z, pitch_rad=0.0, roll_rad=0.0, orientation=None, time_ms=1000):
        seed = [0] + self.current_joint_state[:5] + [0] 
        if(orientation is None):
            target_joints = self.kinematics.inverse_kinematics(
                target_pos=[x, y, z],
                seed_state=seed
            )
        elif(orientation is not None):
            target_joints = self.kinematics.inverse_kinematics(
                target_pos=[x, y, z],
                target_orient=[0, 0, -1],
                orientation_mode=orientation, # "Z" is up
                seed_state=seed
            )
        motor_commands = target_joints[1:6]
        
        self.move_to_joints(motor_commands, time_ms)
        # Várunk, amíg odaér
        time.sleep(time_ms / 1000.0)

    def gripper_open(self):
        val = self.gripper_limits['open']
        self.driver.write_position(6, val, time_ms=500)
        time.sleep(0.5)

    def gripper_close(self):
        val = self.gripper_limits['close']
        self.driver.write_position(6, val, time_ms=500)
        time.sleep(0.5)
    
    def torque_enable(self, enable=True):
        for i in range(1, 7):
            self.driver.torque_enable(i, enable)
    
    def getCalibrationvalues(self):
        return {
            "offsets": self.offsets,
            "directions": self.directions,
            "adjustments": self.adjustments,
            "gripper_limits": self.gripper_limits
        }
    def close(self):
        self.driver.close()