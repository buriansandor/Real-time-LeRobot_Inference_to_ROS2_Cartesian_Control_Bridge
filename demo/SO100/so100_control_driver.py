#!/usr/bin/env python3
"""
SO100 Robot Control Driver  
Created by Sandor Burian with the help of Google Gemini Pro.
This driver interfaces with the SO100 robotic arm using serial communication.
It also integrates with IKPy for kinematics calculations.
"""

import os
import ikpy.chain
import serial
import time
import struct
import math
import csv

class SO100ControlDriver:
    def __init__(self, port=None, urdf_path="URDF/so100.urdf", calibration_file="callibration_data.csv", simulation=False):

        if port is None and not simulation:
            args = self.parse_arguments()
            self.simulation = args.simulation
            if os.windows():
                self.port = getattr(args, 'port', 'COM5') # Default port for Windows
            else:
                self.port = getattr(args, 'port', '/dev/ttyUSB0') # Default port
            urdf_to_load = args.urdf
            calib_to_load = args.calibration_file
            motor_ids = args.motor_ids
        else:
            self.simulation = simulation
            self.port = port
            urdf_to_load = urdf_path
            calib_to_load = calibration_file
            motor_ids = [1, 2, 3, 4, 5, 6]

        urdf_to_load = self.config_file_reader(urdf_to_load)
        calib_to_load = self.config_file_reader(calib_to_load)

        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_to_load, base_elements=["base"])
        self.n_motors = len(motor_ids)
        
        try:
            with open(calib_to_load, 'r') as file:
                reader = csv.reader(file)
                print("✓ Calibration file loaded:", calib_to_load)
                for row in reader:
                    if len(row) > 1:
                        param_name = row[0]
                        values = [float(x) if '.' in x else int(x) for x in row[1:]]
                        if param_name == 'ZERO_OFFSETS': self.ZERO_OFFSETS = values
                        elif param_name == 'DIRECTIONS': self.DIRECTIONS = values
                        elif param_name == 'CALIBRATION_POSE_ADJUSTMENTS': self.CALIBRATION_POSE_ADJUSTMENTS = values
        except FileNotFoundError:
            print(f"ERROR: Cannot find calibration file ({calib_to_load})!")
            print("Using default calibration values...")
            # Default values to prevent crashes
            self.ZERO_OFFSETS = [2048] * 6
            self.DIRECTIONS = [1] * 6
            self.CALIBRATION_POSE_ADJUSTMENTS = [0] * 6
            print("[WARNING] Default calibration loaded: ", "zero offsets:", self.ZERO_OFFSETS, " directions: ", self.DIRECTIONS, " calibration pose adjustments: ", self.CALIBRATION_POSE_ADJUSTMENTS)

        # Connection
        self.ser = None
        if not self.simulation and self.port:
            try:
                self.ser = serial.Serial(self.port, 1000000, timeout=0.1)
                print(f"Successful connection: {self.port}")
            except Exception as e:
                print(f"Failed to connect: {e}")
                self.simulation = True

    def config_file_reader(self, file_to_load):
        """
        Smart path resolution - try current working directory first, then script directory
        Based on Cpilot
        """

        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # For URDF file
        if not os.path.isabs(file_to_load):
            # Try from current working directory first
            if os.path.exists(file_to_load):
                to_load = os.path.abspath(file_to_load).replace('\\', '/')
            else:
                # Fallback to script directory
                to_load = os.path.join(script_dir, file_to_load).replace('\\', '/')
        return to_load

    def show_current_calibration(self):
        print("Current Calibration Parameters:")
        print(f" - ZERO_OFFSETS: {self.ZERO_OFFSETS}")
        print(f" - DIRECTIONS: {self.DIRECTIONS}")
        print(f" - CALIBRATION_POSE_ADJUSTMENTS: {self.CALIBRATION_POSE_ADJUSTMENTS}")

    def torque_enable(self, servo_id):
        return self._write_packet(servo_id, 40, [1])

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
            print(f"[Driver] Calibration loaded from: {filename}")
        except FileNotFoundError:
            print("[Driver] WARNING: Calibration file not found, using default values.")

    def _checksum(self, data):
        """STS3215 Checksum calculation"""
        return (~sum(data)) & 0xFF

    def _write_packet(self, motor_id, instruction, parameters):
        if self.simulation: return
        
        # Protocol: [FF, FF, ID, LEN, INSTR, P1, P2..., CHK]
        length = len(parameters) + 2
        packet = [0xFF, 0xFF, motor_id, length, instruction] + parameters
        checksum = self._checksum(packet[2:])
        packet.append(checksum)
        
        try:
            self.ser.write(bytearray(packet))
            time.sleep(0.0002) 
        except Exception as e:
            print(f"Error while writing: {e}")

    def set_target_joints(self, target_radians, move_time_ms=500):
        """
        Move the robot to the specified joint angles in radians.
        target_radians: List [J1, J2, J3, J4, J5, J6] in radians
        move_time_ms: Time in milliseconds to reach the target (speed control)
        """
        if self.simulation:
            print(f"[SIM] Movement: {target_radians}")
            return

        # Speed/Time calculation (STS3215 Time parameter)
        # For STS motors, speed or time is written to separate registers
        # Here we simplify: we send a Position Move command with time parameter.
        
        # Registers: 
        # 0x2A (42): Goal Position (2 byte)
        # 0x2C (44): Goal Time (2 byte) - optional
        # 0x2E (46): Goal Speed (2 byte) - optional
        
        speed_val = 2000 # Default speed (if no timing)

        for i, target_rad in enumerate(target_radians):
            motor_id = i + 1
            if motor_id > 6: break
            
            # 1. Radian -> Raw conversion
            # Formula: raw = ((rad - adjust) / (ratio * dir)) + offset
            
            idx = i
            ratio = (2 * math.pi / 4096)
            
            raw_float = ((target_rad - self.CALIBRATION_POSE_ADJUSTMENTS[idx]) / 
                         (ratio * self.DIRECTIONS[idx])) + self.ZERO_OFFSETS[idx]
            
            raw_pos = int(raw_float)
            
            # 2. Safety clipping (0..4095)
            raw_pos = max(0, min(4095, raw_pos))
            
            # 3. Packet assembly (Write Instruction 0x03)
            # Address: 0x2A (Goal Position)
            # Data: [LowPos, HighPos, LowTime, HighTime, LowSpd, HighSpd] 
            # (STS3215 also supports Sync Write, but we'll do it individually for now)
            
            p_low = raw_pos & 0xFF
            p_high = (raw_pos >> 8) & 0xFF
            
            t_low = move_time_ms & 0xFF
            t_high = (move_time_ms >> 8) & 0xFF
            
            s_low = 0    # Speed 0 = Max speed (time dominates)
            s_high = 0
            
            # Starting at register 42 write 6 bytes (Position, Time, Speed)
            params = [0x2A, p_low, p_high, t_low, t_high, s_low, s_high]
            
            self._write_packet(motor_id, 0x03, params)

    def read_register(self, motor_id, reg_addr, length=2):
        """
        Read a register value from a motor.
        reg_addr: The register address (e.g., 9 or 11)
        length: Number of bytes to read (usually 2 bytes for STS)
        """
        if self.simulation: return 0
        
        # Instruction 0x02 = READ
        # Parameters: [Register Address, Read Length]
        params = [reg_addr, length]
        
        # Packet assembly (same as write, but without ID in params)
        # Packet: [FF, FF, ID, Len, Instr(02), Reg, ReadLen, Chk]
        msg_len = len(params) + 2
        packet = [0xFF, 0xFF, motor_id, msg_len, 0x02] + params
        checksum = self._checksum(packet[2:])
        packet.append(checksum)
        
        # 1. Clear the buffer (to avoid reading old garbage)
        self.ser.reset_input_buffer()
        
        # 2. Send
        self.ser.write(bytearray(packet))
        
        # 3. Read response
        # Response length: Header(2) + ID(1) + Len(1) + Err(1) + Data(length) + Chk(1)
        expected_bytes = 6 + length
        response = self.ser.read(expected_bytes)
        
        if len(response) < expected_bytes:
            print(f"[Driver] Error: No response from motor {motor_id}!")
            return None
            
        # 4. Unpack data (Little Endian)
        # Data starts from the 5th byte of the response
        if length == 2:
            value = response[5] + (response[6] << 8)
            return value
        elif length == 1:
            return response[5]
            
        return 0

    def read_current_joints(self):
        """Get current joint angles in radians"""
        if self.simulation: return 2048
        
        msg = [0xFF, 0xFF, motor_id, 0x04, 0x02, 0x38, 0x02]
        msg.append(self._checksum(msg[2:]))
        self.ser.reset_input_buffer()
        self.ser.write(bytearray(msg))
        response = self.ser.read(8)
        if len(response) == 8:
            return struct.unpack('<H', response[5:7])[0]
        return None

    def read_raw_position(self, motor_id):
        if self.simulation: return 2048
        
        msg = [0xFF, 0xFF, motor_id, 0x04, 0x02, 0x38, 0x02]
        msg.append(self._checksum(msg[2:]))
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

    def torque_disable(self):
        """Disable motors"""
        for i in range(1, 7):
            self._write_packet(i, 0x03, [0x28, 0x00]) # 0x28 = Torque Enable register, 0 = OFF
    
    def read_angle(self, servo_id):
        """
        Conversion from raw value to radians based on calibration
        
        :param self: Description
        :param servo_id: Description
        """
        raw_pos = self.read_raw_position(servo_id)
        if raw_pos == -1:
            return None
        
        # 1. Subtract the offset (the zero position)
        # 2. Multiply by the direction (1 or -1)
        # 3. Convert the 4096 steps to radians (2 * pi / 4096)
        angle = (raw_pos - self.ZERO_OFFSETS[servo_id]) * self.DIRECTIONS[servo_id] * (2 * 3.14159 / 4096)
        return angle
    
    def set_target_angle(self, motor_index, angle_radians, move_time_ms=1000):
        """
        Sets the target angle for a specified motor and initiates movement.
        This method converts the desired angle in radians to the motor's raw position value,
        applies necessary offsets, directions, and calibration adjustments, and sends a
        control packet to the motor to move to the target position over the specified time.
        Parameters:
        motor_index (int): Index of the motor to control, ranging from 0 to 5 (where 0 corresponds to motor 1).
        angle_radians (float): Target angle in radians to set for the motor.
        move_time_ms (int, optional): Time in milliseconds for the motor to reach the target angle. Defaults to 500.
        Notes:
        - The method performs no operation if the system is in simulation mode.
        - Raw position values are clamped to the range 0-4095.
        - The motor uses STS3215 protocol, writing to registers for goal position, time, and speed.
        
        move motors to a specific angle in radians
        angle_radians: target angle in radians
        motor_index: 0-tól 5-ig (0 = 1-es motor)
        """
        if self.simulation: return

        motor_id = motor_index + 1
        
        self._write_packet(motor_id, 0x03, [0x28, 0x01])

        # 1. Radian -> raw (Raw 0-4096)
        offset = self.ZERO_OFFSETS[motor_index]
        direction = self.DIRECTIONS[motor_index]
        calib = self.CALIBRATION_POSE_ADJUSTMENTS[motor_index]
        
        # read: final = (raw - offset) * ratio * dir + calib
        # write: raw = ((final - calib) / (ratio * dir)) + offset
        
        ratio = (2 * math.pi / 4096)
        raw_val = ((angle_radians - calib) / (ratio * direction)) + offset
        raw_int = int(raw_val)

        print(f"[DEBUG] Motor {motor_id}: Target angle={angle_radians:.2f} -> Offset={offset} -> Calculated RAW={raw_int}")
       
        raw_int = max(0, min(4095, raw_int))
        
        # 2. move
        # STS3215: Register 42 (Goal Position) + Reg 44 (Time) + Reg 46 (Speed)
        
        p_low = raw_int & 0xFF
        p_high = (raw_int >> 8) & 0xFF
        
        t_low = move_time_ms & 0xFF
        t_high = (move_time_ms >> 8) & 0xFF
        
        s_low = 0; s_high = 0
        
        # Write Instruction (0x03) to address 0x2A (42)
        params = [0x2A, p_low, p_high, t_low, t_high, s_low, s_high]
        self._write_packet(motor_id, 0x03, params)