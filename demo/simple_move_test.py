#!/usr/bin/env python3

"""
SO100 Robot Driver Demo
Created by Sandor Burian with the help of Gooogle Gemini Pro.
This script demonstrates simple movements of the SO100 robotic arm using the SO100ControlDriver.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from demo.SO100.so100_control_driver import SO100ControlDriver
import time
import math

port_id = input("Enter the COM port ID (e.g., COM4 or /dev/ttyUSB0) (If you are not sure, use the 'lerobot-find-port'): ").strip()
driver = SO100ControlDriver(port=port_id, urdf_path="URDF/so100.urdf", simulation=False)

print("Starting...")
time.sleep(2)

print("Movement 1: Left")
# Move the 1st motor (index 0) to 0.5 radians (approx 30 degrees) in 1 second
driver.set_target_angle(motor_index=0, angle_radians=0.5, move_time_ms=1000)
time.sleep(1.5)

print("Movement 2: Right")
# Move to -0.5 radians
driver.set_target_angle(motor_index=0, angle_radians=-0.5, move_time_ms=1000)
time.sleep(1.5)

print("Finished. Relaxing.")
# Turn off torque (Torque Off - Reg 40 = 0)
for i in range(1, 7):
    driver._write_packet(i, 0x03, [0x28, 0x00])