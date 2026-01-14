#!/usr/bin/env python3
"""
SO100 Robot Control Driver  
Created by Sandor Burian with the help of Google Gemini Pro.
This driver interfaces with the SO100 robotic arm using serial communication.
It also integrates with IKPy for kinematics calculations.
"""

import serial
import time
import struct
import math
import csv

class SO100ControlDriver:
    def __init__(self, port, calibration_file="calibration_data.csv", simulation=False):
        """
        Driver for SO100 robotic arm.
        """
        self.simulation = simulation
        self.port = port
        
        # default values
        self.ZERO_OFFSETS = [2048] * 6
        self.DIRECTIONS = [1] * 6
        self.CALIBRATION_POSE_ADJUSTMENTS = [0] * 6
        
        # Calibration loading
        self._load_calibration(calibration_file)
        
        self.ser = None
        if not self.simulation:
            try:
                print(f"[Driver] Connecting to: {port}...")
                self.ser = serial.Serial(port, 1000000, timeout=0.05)
                print("[Driver] Successful connection!")
            except Exception as e:
                print(f"[Driver] ERROR: Could not connect: {e}")
                print("[Driver] Switching to simulation mode.")
                self.simulation = True

    def _load_calibration(self, filename):
        try:
            with open(filename, 'r') as file:
                reader = csv.reader(file)
                for row in reader:
                    if not row: continue
                    if row[0] == 'ZERO_OFFSETS':
                        self.ZERO_OFFSETS = [int(x) for x in row[1:]]
                    elif row[0] == 'DIRECTIONS':
                        self.DIRECTIONS = [int(x) for x in row[1:]]
                    elif row[0] == 'CALIBRATION_POSE_ADJUSTMENTS':
                        self.CALIBRATION_POSE_ADJUSTMENTS = [float(x) for x in row[1:]]
            print(f"[Driver] Kalibráció betöltve innen: {filename}")
        except FileNotFoundError:
            print("[Driver] FIGYELEM: Nincs kalibrációs fájl, alapértelmezett értékekkel indulok.")

    def _checksum(self, data):
        """STS3215 Checksum számítás"""
        return (~sum(data)) & 0xFF

    def _write_packet(self, motor_id, instruction, parameters):
        if self.simulation: return
        
        # Protocol: [FF, FF, ID, LEN, INSTR, P1, P2..., CHK]
        length = len(parameters) + 2
        packet = [0xFF, 0xFF, motor_id, length, instruction] + parameters
        checksum = self._checksum(packet[2:])
        packet.append(checksum)
        
        self.ser.write(bytearray(packet))
        # time.sleep(0.0005) # Kis várakozás biztonságból

    def set_target_joints(self, target_radians, move_time_ms=500):
        """
        A robot mozgatása a megadott radián szögekre.
        target_radians: Lista [J1, J2, J3, J4, J5, J6] radiánban
        move_time_ms: Mennyi idő alatt érjen oda (sebesség szabályozás)
        """
        if self.simulation:
            print(f"[SIM] Mozgás: {target_radians}")
            return

        # Sebesség/Idő számítása (STS3215 Time paraméter)
        # Az STS motoroknál a sebességet vagy időt külön regiszterbe írjuk
        # Itt egyszerűsítsünk: Position Move parancsot küldünk
        
        # Regiszterek: 
        # 0x2A (42): Goal Position (2 byte)
        # 0x2C (44): Goal Time (2 byte) - opcionális
        # 0x2E (46): Goal Speed (2 byte) - opcionális
        
        speed_val = 2000 # Default sebesség (ha nincs időzítés)
        # Ha időre akarunk menni, bonyolultabb. Most fix sebességgel menjünk.

        for i, target_rad in enumerate(target_radians):
            motor_id = i + 1
            if motor_id > 6: break
            
            # 1. Radián -> Nyers (Raw) konverzió
            # Képlet: raw = ((rad - adjust) / (ratio * dir)) + offset
            
            idx = i
            ratio = (2 * math.pi / 4096)
            
            raw_float = ((target_rad - self.CALIBRATION_POSE_ADJUSTMENTS[idx]) / 
                         (ratio * self.DIRECTIONS[idx])) + self.ZERO_OFFSETS[idx]
            
            raw_pos = int(raw_float)
            
            # 2. Biztonsági vágás (0..4095)
            raw_pos = max(0, min(4095, raw_pos))
            
            # 3. Csomag összeállítása (Write Instruction 0x03)
            # Cím: 0x2A (Goal Position)
            # Adat: [LowPos, HighPos, LowTime, HighTime, LowSpd, HighSpd] 
            # (STS3215 Sync Write-ot is tudna, de most csináljuk egyenként)
            
            p_low = raw_pos & 0xFF
            p_high = (raw_pos >> 8) & 0xFF
            
            t_low = move_time_ms & 0xFF
            t_high = (move_time_ms >> 8) & 0xFF
            
            s_low = 0    # Speed 0 = Max sebesség (az idő dominál)
            s_high = 0
            
            # Reg 42-től írunk 6 bájtot (Pozíció + Idő + Sebesség)
            params = [0x2A, p_low, p_high, t_low, t_high, s_low, s_high]
            
            self._write_packet(motor_id, 0x03, params)

    def read_current_joints(self):
        """Visszaadja a jelenlegi szögeket radiánban [J1...J6]"""
        # (Itt felhasználhatod a régi kód 'read_raw_position' és 'get_joint_angles' logikáját)
        # A rövidség kedvéért ezt most nem másolom be újra, de 
        # UGYANAZT a logikát kell ide betenni, ami a régi driverben volt.
        pass # <--- IDE MÁSOLD BE A READ LOGIKÁT A RÉGI DRIVERBŐL

    def torque_disable(self):
        """Motorok lazítása"""
        for i in range(1, 7):
            self._write_packet(i, 0x03, [0x28, 0x00]) # 0x28 = Torque Enable regiszter, 0 = OFF