import time
import math
import os
import csv
import numpy as np
from .sts3215 import STS3215Driver
from .kinematics import SO100Kinematics

class SO100Robot:
    def __init__(self, port, config_dir="config"):
        self.driver = STS3215Driver(port)
        
        # --- PATH DEBUG ---
        # Abszolúttá tesszük az útvonalat, hogy lássuk, hova mutat
        abs_config_dir = os.path.abspath(config_dir)
        print(f"[DEBUG] Config keresése itt: {abs_config_dir}")
        
        urdf_path = os.path.join(abs_config_dir, "so100.urdf")
        if not os.path.exists(urdf_path):
            print(f"[WARN] URDF nem található: {urdf_path}. Próbálkozás a driver mappában...")
            # Fallback: a csomag saját mappája - so100_core szülője a SO100_Robot
            package_dir = os.path.dirname(os.path.dirname(__file__))
            urdf_path = os.path.join(package_dir, "config", "so100.urdf")
            
        self.kinematics = SO100Kinematics(urdf_path)
        
        self.offsets = {}
        self.directions = {}
        self.adjustments = {}
        # Betöltés az abszolút útvonalról
        self._load_calibration(os.path.join(abs_config_dir, "follower_calibration.csv"))
        
        self.gripper_limits = {'open': 2000, 'close': 1500}
        self._load_gripper_config(os.path.join(abs_config_dir, "gripper_values.csv"))
        
        self.current_joint_state = [0.0] * 6 

    def _load_calibration(self, filepath):
        if not os.path.exists(filepath):
            print(f"⚠️  [CRITICAL] HIBA! Nem találom a kalibrációs fájlt: {filepath}")
            print("⚠️  A robot alapértelmezett (2048) értékekkel indul, ami FERDE lesz!")
            self.offsets = {i: 2048 for i in range(6)}
            self.directions = {i: 1 for i in range(6)}
            self.adjustments = {i: 0.0 for i in range(6)}
            return

        print(f"✅ Kalibráció betöltése: {filepath}")
        try:
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row: continue
                    if row[0] == 'ZERO_OFFSETS':
                        self.offsets = {i: int(val) for i, val in enumerate(row[1:])}
                    elif row[0] == 'DIRECTIONS':
                        self.directions = {i: int(val) for i, val in enumerate(row[1:])}
                    elif row[0] == 'CALIBRATION_POSE_ADJUSTMENTS':
                        self.adjustments = {i: float(val) for i, val in enumerate(row[1:])}
        except Exception as e:
            print(f"⚠️ Hiba a CSV olvasásakor: {e}")

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
        # Most már átadjuk a time_ms paramétert a drivernek!
        for i, angle in enumerate(joint_angles):
            if i == 5: continue 
            raw_val = self._angle_to_raw(i, angle)
            # FONTOS: Itt használjuk az új time_ms paramétert!
            self.driver.write_position(i + 1, raw_val, time_ms=time_ms)
            
        self.current_joint_state = list(joint_angles)

    def move_to_cartesian(self, x, y, z, pitch_rad=0.0, roll_rad=0.0, time_ms=1500):
        seed = [0] + self.current_joint_state[:5] + [0] 
        target_joints = self.kinematics.inverse_kinematics(
            target_pos=[x, y, z],
            target_orient=[0, 0, -1],
            orientation_mode="Z",
            seed_state=seed
        )
        motor_commands = target_joints[1:6]
        
        self.move_to_joints(motor_commands, time_ms)
        # Nem kell extra sleep, mert a time_ms kezeli a sebességet
        # De várnunk kell, amíg odaér a programfutásban
        time.sleep(time_ms / 1000.0)

    def gripper_open(self):
        val = self.gripper_limits['open']
        self.driver.write_position(6, val, time_ms=500) # Fél másodperc nyitás
        time.sleep(0.5)

    def gripper_close(self):
        val = self.gripper_limits['close']
        self.driver.write_position(6, val, time_ms=500)
        time.sleep(0.5)
    
    def torque_enable(self, enable=True):
        for i in range(1, 7):
            self.driver.torque_enable(i, enable)

    def close(self):
        self.driver.close()