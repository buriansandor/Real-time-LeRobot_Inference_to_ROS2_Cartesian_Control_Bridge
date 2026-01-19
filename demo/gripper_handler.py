#!/usr/bin/env python3
"""
This is the gripper handler.

Created by Sandor Burian with the help of Copilot.

This module provides functions to control the gripper of the SO100 robotic arm.
open_gripper: Opens the gripper to a specified position.
close_gripper: Closes the gripper to a specified position.
get_gripper_position: Retrieves the current position of the gripper.
get_gripper_open_position: Retrieves the predefined open position of the gripper.
get_gripper_close_position: Retrieves the predefined close position of the gripper.
"""

import time


class GripperHandler:
    def __init__(self, driver, open_position=2000, close_position=1500, calibration_file="gripper_values.csv"):
        self.driver = driver
        self.calibration_file = calibration_file
        
        try:
            with open(self.calibration_file, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) == 2:
                        key, value = parts
                        if key == 'GRIPPER_OPEN':
                            self.open_position = int(value)
                        elif key == 'GRIPPER_CLOSE':
                            self.close_position = int(value)
                print("[Info] Gripper calibration loaded successfully.")
                print(f"[Info] Gripper open position: {self.open_position}, close position: {self.close_position}")
        except (FileNotFoundError, ValueError, OSError):
            print(f"[Warning] Could not read gripper calibration file! '{self.calibration_file}'. Using default values.")
            self.open_position = open_position
            self.close_position = close_position
            print(f"[Info] Gripper open position: {self.open_position}, close position: {self.close_position}")

    def get_gripper_position(self):
        """Returns the current position of the gripper."""
        return self.driver.get_current_position(motor_index=5)

    def get_gripper_open_position(self):
        """Returns the predefined open position of the gripper."""
        return self.open_position

    def get_gripper_close_position(self):
        """Returns the predefined close position of the gripper."""
        return self.close_position

    def close_gripper(self, close_position=None):
        """Closes the gripper to the specified close position."""
        if close_position is None:
            close_position = self.close_position
        self.move_gripper(close_position)
    
    def open_gripper(self, open_position=None):
        """Opens the gripper to the specified open position."""
        if open_position is None:
            open_position = self.open_position
        self.move_gripper(open_position)
    
    def move_gripper(self, val):
        """Moves the gripper to the specified raw position value."""
        # Write to motor 6
        self.driver._write_packet(6, 0x03, [0x2A, 0x00, val & 0xFF, (val >> 8) & 0xFF, 0x00, 0x00, 0x00, 0x00])
        time.sleep(1.0)
    
    def release(self):
        """Releases the gripper motor (relaxes it)."""
        self.driver._write_packet(6, 0x03, [0x28, 0x00])
    