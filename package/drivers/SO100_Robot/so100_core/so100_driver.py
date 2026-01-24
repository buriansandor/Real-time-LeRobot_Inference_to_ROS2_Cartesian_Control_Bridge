#!/usr/bin/env python3
"""
SO100 Robot Control Driver (OPTIMIZED SYNC WRITE)
Based on the original working driver by Sandor Burian
Optimized for high-speed teleoperation (ZeroMQ/ROS)
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
        self.port = port  # Store port for recovery
        self.baud = None
        if not simulation and port:
            # Probe for correct baudrate: try high-speed first (1_000_000)
            candidate_bauds = [1000000, 115200]
            found = False
            for b in candidate_bauds:
                try:
                    # Timeout set to 0.05 for non-blocking write/read speed
                    ser_try = serial.Serial(port, b, timeout=0.05)
                    try:
                        pkt = bytearray([0xFF, 0xFF, 0x01, 0x02, 0x01])
                        chk = (~sum(pkt[2:])) & 0xFF
                        pkt.append(chk)
                        ser_try.reset_input_buffer()
                        ser_try.write(pkt)
                        time.sleep(0.01)
                        resp = ser_try.read(8)
                        if resp and len(resp) >= 4:
                            self.ser = ser_try
                            self.baud = b
                            print(f"[SO100] Connected to: {port} at {b} baud (detected)")
                            found = True
                            break
                    except Exception:
                        try: ser_try.close()
                        except: pass
                        continue
                except Exception:
                    continue

            if not found:
                print(f"[SO100] Connection failed on {port}")
                self.simulation = True
            else:
                try:
                    self.enable_all_motors()
                except Exception as e:
                    print(f"[SO100] Warning: enable_all_motors failed: {e}")
        
        # State tracking
        self.current_joint_state = [0.0] * 6
        self.serial_error_count = 0 
        self.max_serial_errors = 3
        self.torque_enabled = [False] * 6
        
        # Gripper limits  
        self.gripper_limits = {'open': 2000, 'close': 1500}
        self._load_gripper_config(os.path.join(config_dir, "gripper_values.csv"))

    def _load_calibration(self, calib_path):
        try:
            with open(calib_path, 'r', encoding='utf-8-sig') as file:
                reader = csv.reader(file)
                print(f"[SO100] Loading calibration: {calib_path}")
                for row in reader:
                    if len(row) > 1:
                        param_name = row[0]
                        values = [float(x) if '.' in x else int(x) for x in row[1:]]
                        if param_name == 'ZERO_OFFSETS': self.ZERO_OFFSETS = values
                        elif param_name == 'DIRECTIONS': self.DIRECTIONS = values
                        elif param_name == 'CALIBRATION_POSE_ADJUSTMENTS': self.CALIBRATION_POSE_ADJUSTMENTS = values
        except FileNotFoundError:
            print(f"[SO100] ERROR: Calibration file not found: {calib_path}")
            self.ZERO_OFFSETS = [2048] * 6
            self.DIRECTIONS = [1] * 6
            self.CALIBRATION_POSE_ADJUSTMENTS = [0.0] * 6

    def _load_gripper_config(self, gripper_path):
        try:
            with open(gripper_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if row[0] == 'GRIPPER_OPEN': self.gripper_limits['open'] = int(row[1])
                    elif row[0] == 'GRIPPER_CLOSE': self.gripper_limits['close'] = int(row[1])
        except FileNotFoundError: pass

    def _angle_to_raw(self, motor_index, radians):
        """Convert radians to raw motor value using calibration"""
        offset = self.ZERO_OFFSETS[motor_index]
        direction = self.DIRECTIONS[motor_index]
        calib = self.CALIBRATION_POSE_ADJUSTMENTS[motor_index]
        ratio = (2 * math.pi / 4096)
        raw_val = ((radians - calib) / (ratio * direction)) + offset
        return int(raw_val)

    def enable_all_motors(self):
        """Enable torque on all motors"""
        if self.simulation or not self.ser: return
        # Use Broadcast ID 0xFE to enable all at once
        # ID=0xFE, Len=0x04, Instr=0x03(Write), Addr=0x28(Torque), Val=0x01
        pkt = [0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x28, 0x01]
        self._write_packet_raw(pkt)
        time.sleep(0.1)

    def _checksum(self, data):
        return (~sum(data)) & 0xFF

    def _write_packet_raw(self, packet_no_checksum):
        """Helper to write raw packet adding checksum"""
        chk = self._checksum(packet_no_checksum[2:])
        packet_no_checksum.append(chk)
        try:
            self.ser.write(bytearray(packet_no_checksum))
        except: pass

    def _write_packet(self, motor_id, instruction, parameters):
        """Legacy support for single packet write"""
        length = len(parameters) + 2
        packet = [0xFF, 0xFF, motor_id, length, instruction] + parameters
        self._write_packet_raw(packet)
        return True

    def move_to_joints(self, joint_angles, time_ms=30):
        """
        🚀 TURBO MODE: Move all motors at once using SYNC WRITE (ID 0xFE).
        This eliminates the 0.3s delay caused by sequential writing.
        """
        if self.simulation: return

        # Protocol: STS3215 Sync Write (Instr: 0x83)
        # Start Address: 0x2A (Target Position)
        # Data Length per motor: 4 bytes (Position L, Position H, Time L, Time H)
        
        payload = [0x2A, 0x04] # Start Addr, Length per motor
        
        # Build payload for the first 5 motors (Arm)
        # We skip the gripper (index 5) as it is handled by gripper_open/close
        for i, angle in enumerate(joint_angles):
            if i >= 5: break # Only map the first 5 arm motors
            
            motor_id = i + 1
            raw_val = self._angle_to_raw(i, angle)
            raw_val = max(0, min(4095, int(raw_val)))
            
            t_val = int(time_ms)
            
            # Add to payload: ID, PosL, PosH, TimeL, TimeH
            payload.extend([
                motor_id,
                raw_val & 0xFF, (raw_val >> 8) & 0xFF,
                t_val & 0xFF, (t_val >> 8) & 0xFF
            ])
            
        # Send one big packet to Broadcast ID 0xFE
        # Header: [FF, FF, FE, LEN, 83, ...payload..., CHK]
        length = len(payload) + 2
        packet = [0xFF, 0xFF, 0xFE, length, 0x83] + payload
        
        self._write_packet_raw(packet)
        
        # No sleeping here! We return immediately to process next frame.
        self.current_joint_state = list(joint_angles)

    def gripper_open(self):
        val = self.gripper_limits['open']
        self._move_single_motor(6, val, 200)

    def gripper_close(self):
        val = self.gripper_limits['close']
        self._move_single_motor(6, val, 200)

    def _move_single_motor(self, motor_id, raw_val, time_ms):
        if self.simulation: return
        # [0x2A, PosL, PosH, TimeL, TimeH, SpeedL, SpeedH]
        params = [
            0x2A, 
            raw_val & 0xFF, (raw_val >> 8) & 0xFF,
            time_ms & 0xFF, (time_ms >> 8) & 0xFF,
            0, 0
        ]
        self._write_packet(motor_id, 0x03, params)

    def torque_disable(self):
        if self.simulation: return
        # Broadcast disable
        pkt = [0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x28, 0x00]
        self._write_packet_raw(pkt)

    def torque_enable(self, enable=True):
        """Compatibility wrapper for scripts expecting torque_enable"""
        if enable:
            self.enable_all_motors()
        else:
            self.torque_disable()

    def close(self):
        if self.ser:
            self.ser.close()
    
    def get_joint_angles(self):
        return self.current_joint_state
    
    def get_cartesian_pose(self):
        return [0,0,0], [0,0,0,1] # Dummy for ROS