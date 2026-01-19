#!/usr/bin/env python3
"""
SO100 Robot Driver Inverse Kinematics (Cartesian) Demo

Created by Sandor Burian with the help of Google Gemini Pro.

This script allows users to input Cartesian coordinates (X, Y, Z)
and calculates the corresponding joint angles using IKPy.
It then commands the SO100 robotic arm to move to the specified position.
"""

import sys
import os
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from demo.SO100.so100_control_driver import SO100ControlDriver


PORT = input("Enter the port of the follower arm (e.g., COM4 or /dev/ttyUSB0) or get the porty with port detection(detect): ").strip()
if PORT == '':
    print("Setting port to the default: COM4")
    PORT = 'COM4'  # Change this to your follower's port if needed
elif PORT.lower() == 'detect':
    try:
        from lerobot_functions import PortFinder
        print("Set the port of leader arm")
        PORT = PortFinder.find_port_with_lerobot()
    except ImportError:
        print("Warning: PortFinder not available, using manual port configuration.\n Please set LEADER_PORT and FOLLOWER_PORT manually, use 'lerobot-find-port' to find them in a separate console.")
CALIB_FILE = input("Enter the calibration file name (default: follower_calibration.csv): ").strip()
if CALIB_FILE == '':
        print("Setting calibration file to default: follower_calibration.csv")
        CALIB_FILE = 'follower_calibration.csv'
URDF_PATH = input("Enter the URDF file path (default: URDF/so100.urdf): ").strip()
if URDF_PATH == '':
    print("Setting URDF path to default: URDF/so100.urdf")
    URDF_PATH = "URDF/so100.urdf"


def main():
    print("\n\n=== SO-100 INVERSE KINEMATICS (XYZ) TEST ===")
    
    # 1. Driver initialising
    print(f"Connecting ({PORT})...")
    # Important: use calibration_file parameter to use the GOOD 2019 values!
    driver = SO100ControlDriver(port=PORT, urdf_path=URDF_PATH, calibration_file=CALIB_FILE, simulation=False)

    # If file loading is still not stable, you can override it here for safety:
    # driver.ZERO_OFFSETS = [2059, 2019, 1042, 2071, 864, 1841] 

    # 2. Torque ON
    print("Waking up motors...")
    for i in range(1, 7):
        driver._write_packet(i, 0x03, [0x28, 0x01])
        time.sleep(0.05)
    time.sleep(1.0)

    # 3. Check if the IKPy chain loaded correctly
    if driver.chain is None:
        print("ERROR:\tCan't load URDF (ikpy chain is None)")
        print(f"Check the path to {URDF_PATH}!")
        return

    print("\nThe robot is ready.")
    print("Coordinate system (usually): X=Forward, Y=Left, Z=Up")
    print("Unit: METER (e.g., 0.2 = 20 cm)")

    while True:
        print("\n-----------------------------------")
        try:
            val = input("Enter XYZ coordinates (e.g., 0.2 0.0 0.15) or 'x' to exit: ")
            if val.lower() == 'x': break
            
            # Processing coordinates
            parts = val.split()
            if len(parts) != 3:
                print("Error: Please enter exactly 3 numbers separated by spaces!")
                continue
                
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            target_pos = [x, y, z]
            
            print(f"Calculating for: X={x}, Y={y}, Z={z} ...")

            # --- THE ESSENCE: INVERSE KINEMATICS CALCULATION ---
            # This is the magic: The machine calculates the angles from the coordinates
            # target_orientation=None -> Only the position matters, the wrist orientation is irrelevant (for now)
            # orientation_mode="all" -> It tries to set the final joint as well
            seed_state = [0] * len(driver.chain.links) 
            
            target_joints = driver.chain.inverse_kinematics(target_position=target_pos, initial_position=seed_state)
            
            # Printing the calculated angles (in Radians)
            # Note: target_joints[0] is usually the Base (which is fixed at 0), so motors start from 1
            print("Calculated angles (Radians):")
            for i in range(1, len(target_joints)):
                 # Handling ikpy index offset
                 if i <= 6:
                    print(f"  J{i}: {target_joints[i]:.2f}")

            # --- MOVEMENT ---
            move_time = 2000 # 2 seconds (safe speed)
            
            # The IKPy array also contains the "Base link" at index 0.
            # Our driver's set_target_angle(0) command corresponds to motor 1 (J1).
            # So: driver Motor 0 -> IKPy Joint 1
            
            print("Moving...")
            for motor_idx in range(6): # 0..5 (Motor 1..6)
                # IKPy Joint index: motor_idx + 1
                ik_angle = target_joints[motor_idx + 1]
                
                # Sending to the driver (which will add the calibration offset)
                driver.set_target_angle(motor_idx, ik_angle, move_time_ms=move_time)
                time.sleep(0.05)
        except ValueError:
            print("Error: Please enter numbers!")
        except Exception as e:
            print(f"An error occurred: {e}")

    print("Exiting. Releasing motors...")
    for i in range(1, 7):
        driver._write_packet(i, 0x03, [0x28, 0x00])

if __name__ == "__main__":
    main()