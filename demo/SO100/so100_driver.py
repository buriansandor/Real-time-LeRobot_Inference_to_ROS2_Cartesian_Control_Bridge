#!/usr/bin/env python3

"""
SO100 Robot Driver
Created by Sandor Burian with the help of Gooogle Gemini Pro.
This driver interfaces with the SO100 robotic arm using serial communication.
It also integrates with IKPy for kinematics calculations.
"""

import argparse
import serial
import time
import struct
import math
import ikpy.chain
import numpy as np
import csv

class SO100Driver:

    def __init__(self):
        args = self.parse_arguments()

        self.simulation = args.simulation
        self.chain = ikpy.chain.Chain.from_urdf_file(args.urdf, base_elements=["base"])
        self.n_motors = len(args.motor_ids)
        
        # --- Callibration ---
        calibration_file = args.callibration_file
        try:
            with open(calibration_file, 'r') as file:
                reader = csv.reader(file)
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
                    else:
                        self.OTHER = values
        except FileNotFoundError:
            print(f"Warning: {calibration_file} not found, using default values")
        
            self.ZERO_OFFSETS = [2197, 1994, 1091, 2045, 2928, 1675]
            self.DIRECTIONS = [-1, 1, 1, 1, 1, 1]
            self.CALIBRATION_POSE_ADJUSTMENTS = [0, 1.45, -2.8, -0.5, 1.5, 0]
        
        # Hardware init
        self.ser = None
        if not self.simulation:
            try:
                print(f"Connecting to port: {args.port}...")
                self.ser = serial.Serial(args.port, 1000000, timeout=0.05)
                self.relax_motors()
                print("✓ Successfully connected to SO100 robot.")
            except Exception as e:
                print(f"X Error: {e}")
                self.simulation = True # Fallback to simulation
                print("Running in simulation mode.")

    def parse_arguments(self):
        """Parse command line arguments with interactive fallback"""
        parser = argparse.ArgumentParser(description='SO-100 Robot Arm Visualization')
        parser.add_argument('--port', '-p', type=str, help='Serial port (e.g., COM5 or /dev/ttyUSB0)')
        parser.add_argument('--baud-rate', '-b', type=int, default=1000000, help='Baud rate (default: 1000000)')
        parser.add_argument('--simulation', '-s', action='store_true', help='Run in simulation mode')
        parser.add_argument('--loose_motors', '-l', action='store_true', help='Loosen motors for manual movement, default: false')
        parser.add_argument('--urdf', '-u', type=str, default='URDF/so100.urdf', help='Path to URDF file, default: URDF/so100.urdf')
        parser.add_argument('--motor-ids', '-m', nargs='+', type=int, default=[1, 2, 3, 4, 5, 6], help='Motor IDs (default: 1 2 3 4 5 6)')
        parser.add_argument('--logging_on', '-log', action='store_true', help='Enable movement logging, default: false')
        parser.add_argument('--logging_to_file', '-logf', action='store_true', help='Enable movement logging to file, default: false')
        parser.add_argument('--callibration_file', '-c', type=str, default='callibration_data.csv', help='Path to callibration data CSV file, default: callibration_data.csv')

        return parser.parse_args()

    def checksum(self, data):
        return (~sum(data)) & 0xFF

    def relax_motors(self):
        """Loosen the motors"""
        if self.simulation: return
        print("Loosening motors...")

        for i in range(1, self.n_motors + 1):
            # Message structure: [Header1, Header2, ID, Length, Instruction, Address_L, Address_H, Checksum]
            msg = [0xFF, 0xFF, i, 0x04, 0x03, 0x28, 0x00]
            msg.append(self.checksum(msg[2:]))
            self.ser.write(bytearray(msg))
            time.sleep(0.005)

    def read_raw_position(self, motor_id):
        if self.simulation: return 2048
        
        msg = [0xFF, 0xFF, motor_id, 0x04, 0x02, 0x38, 0x02]
        msg.append(self.checksum(msg[2:]))
        self.ser.reset_input_buffer()
        self.ser.write(bytearray(msg))
        response = self.ser.read(8)
        if len(response) == 8:
            return struct.unpack('<H', response[5:7])[0]
        return None

    def get_joint_angles(self):
        """Joint angle in radian"""
        if self.simulation:
            t = time.time()
            sim_base = [0, math.sin(t)*0.5, math.cos(t)*0.5, -1.0, -0.5, 0]
            return [b + a for b, a in zip(sim_base, self.CALIBRATION_POSE_ADJUSTMENTS)] + [0]

        angles = [0] # Base
        for i in range(1, self.n_motors + 1): # Motor ID 1..6
            idx = i - 1             
            raw = self.read_raw_position(i)
            if raw is None: raw = self.ZERO_OFFSETS[idx]

            # 1. raw -> relative
            offset = self.ZERO_OFFSETS[idx]
            direction = self.DIRECTIONS[idx]
            rel_rad = (raw - offset) * (2 * math.pi / 4096) * direction
            
            # 2. correction
            final_rad = rel_rad + self.CALIBRATION_POSE_ADJUSTMENTS[idx]
            angles.append(final_rad)
            
        # if angle list is shorter than number of motors (e.g. for gripper)
        while len(angles) < self.n_motors:
            angles.append(0)
            
        return angles

    def get_tcp_position(self, angles=None):
        """Calculate the XYZ position (Forward Kinematics)"""
        if angles is None:
            angles = self.get_joint_angles()
        
        # FK Calculation
        frame = self.chain.forward_kinematics(angles)
        return frame[:3, 3] # Only the XYZ vector is needed