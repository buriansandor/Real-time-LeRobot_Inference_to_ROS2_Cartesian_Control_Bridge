"""STS3215 Driver for SO100 Robotű
Created by Sandor Burian with the help of Copilot(Claude).

This module provides low-level communication with the STS3215 servomotors used in the SO100 robotic arm.
It supports reading positions, writing target positions, and synchronized movements.
"""

import time
import serial
import struct
import math

class STS3215Driver:
    def __init__(self, port, baudrate=1000000, timeout=0.05):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def close(self):
        if self.ser.is_open:
            self.ser.close()

    def _write_packet(self, motor_id, instruction, params):
        length = len(params) + 2
        msg = [0xFF, 0xFF, motor_id, length, instruction] + params
        chk = (~sum(msg[2:]) & 0xFF)
        msg.append(chk)
        self.ser.write(bytearray(msg))
        time.sleep(0.0002)  # Original timing from working code

    def _read_packet(self, motor_id, length):
        msg = [0xFF, 0xFF, motor_id, length + 2, 0x02] 
        chk = (~sum(msg[2:]) & 0xFF)
        msg.append(chk)
        self.ser.reset_input_buffer()
        self.ser.write(bytearray(msg))
        
        response = self.ser.read(length + 6)
        if len(response) < length + 6:
            return None
        return response

    def read_position(self, motor_id):
        resp = self._read_packet(motor_id, 2)
        if resp:
            try:
                low = resp[5]
                high = resp[6]
                return (high << 8) + low
            except IndexError:
                return None
        return None

    def write_position(self, motor_id, position, time_ms=0, speed=0):
        pos = int(position)
        t = int(time_ms)
        sp = int(speed)
        params = [
            0x2A, 0x00,
            pos & 0xFF, (pos >> 8) & 0xFF,
            t & 0xFF, (t >> 8) & 0xFF,
            sp & 0xFF, (sp >> 8) & 0xFF,
        ]
        self._write_packet(motor_id, 0x03, params)

    def sync_write_pos_time(self, id_pos_time_list):
        """
        Moves multiple motors simultaneously (SYNC WRITE).
        id_pos_time_list: List of tuples -> [(motor_id, position, time_ms), ...]
        """
        # STS3215 Sync Write (Instruction 0x83)
        # Start Address: 0x2A (Position Low)
        # Data Length per motor: 4 bytes (2 byte Position + 2 byte Time)
        
        start_addr = 0x2A
        data_len = 4 
        
        params = [start_addr, data_len]
        
        for mid, pos, time_val in id_pos_time_list:
            pos = int(pos)
            time_val = int(time_val)
            # Motor ID
            params.append(mid)
            # Position Little Endian
            params.append(pos & 0xFF)
            params.append((pos >> 8) & 0xFF)
            # Time Little Endian
            params.append(time_val & 0xFF)
            params.append((time_val >> 8) & 0xFF)
            
        # ID 0xFE is the Broadcast ID for Sync Write
        self._write_packet(0xFE, 0x83, params)

    def sync_write_pos_time_speed(self, id_pos_time_speed_list):
        """
        Moves multiple motors simultaneously (SYNC WRITE) with speed.
        id_pos_time_speed_list: List of tuples -> [(motor_id, position, time_ms, speed), ...]
        """
        # STS3215 Sync Write (Instruction 0x83)
        # Start Address: 0x2A (Position Low)
        # Data Length per motor: 6 bytes (2 byte Position + 2 byte Time + 2 byte Speed)
        
        start_addr = 0x2A
        data_len = 6 
        
        params = [start_addr, data_len]
        
        for mid, pos, time_val, speed_val in id_pos_time_speed_list:
            pos = int(pos)
            time_val = int(time_val)
            speed_val = int(speed_val)
            # Motor ID
            params.append(mid)
            # Position Little Endian
            params.append(pos & 0xFF)
            params.append((pos >> 8) & 0xFF)
            # Time Little Endian
            params.append(time_val & 0xFF)
            params.append((time_val >> 8) & 0xFF)
            # Speed Little Endian (0 = max speed)
            params.append(speed_val & 0xFF)
            params.append((speed_val >> 8) & 0xFF)
            
        # ID 0xFE is the Broadcast ID for Sync Write
        self._write_packet(0xFE, 0x83, params)

    def torque_enable(self, motor_id, enable=True):
        val = 1 if enable else 0
        self._write_packet(motor_id, 0x03, [0x28, 0x00, val])
