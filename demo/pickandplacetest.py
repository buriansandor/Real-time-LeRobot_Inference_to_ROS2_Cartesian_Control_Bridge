#!/usr/bin/env python3
"""
SO100 Robot Driver Demo
Simple Pick and Place Test
Created by Sandor Burian with the help of Google Gemini Pro.
This script demonstrates a simple pick and place operation using the SO100 robotic arm and its gripper with cartesian coordinate system.
"""

import sys
import os
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from demo.SO100.so100_control_driver import SO100ControlDriver
from gripper_handler import GripperHandler


def simple_pick_and_place():
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
      
    driver = SO100ControlDriver(port=PORT, urdf_path=URDF_PATH, calibration_file=CALIB_FILE, simulation=False)
    gripper = GripperHandler(driver)

    print("--- PICK & PLACE DEMO ---")
    
    
    GRIPPER_OPEN = gripper.get_gripper_open_position() 
    GRIPPER_CLOSE = gripper.get_gripper_close_position()
    
    # Test coordinates
    PICK_POS = [0.25, 0.05, 0.03]  # right, forward down
    PLACE_POS = [0.25, -0.05, 0.03] # left, forward, down
    PLACE_POS2 = [-0.20, 0.05, 0.1] # left, backward, down
    SAFE_Z = 0.15 # safety high
    # --------------------
    
    def move_to(driver, x, y, z):
        print(f"Move to: {x}, {y}, {z}")
        target_pos = [x, y, z]
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

    print("\n\n==== Pick and place Simulation ====\n\nStarting...")
    print("Open positions:", GRIPPER_OPEN)
    print("Close position:", GRIPPER_CLOSE)
    print("Waking up motors...")
    for i in range(1, 7):
        driver._write_packet(i, 0x03, [0x28, 0x01])
        time.sleep(0.05)
    time.sleep(1.0)

    # 1. Open
    gripper.open_gripper()
    move_to(driver, PICK_POS[0], PICK_POS[1], PICK_POS[2])

    # 2. Close (As if grabbing something)
    gripper.close_gripper()
    move_to(driver, PICK_POS[0], PICK_POS[1], SAFE_Z)
    
    move_to(driver, PLACE_POS[0], PLACE_POS[1], PLACE_POS[2])
    # 3. Open again
    gripper.open_gripper()
    move_to(driver, PICK_POS[0], PICK_POS[1], SAFE_Z)
    
    gripper.close_gripper()

    gripper.release()
    print("Test done!")
    finish = input("Press x to release the arm!")
    if finish == "x":
        print("Exiting. Releasing motors...")
        for i in range(1, 7):
            driver._write_packet(i, 0x03, [0x28, 0x00])

if __name__ == "__main__":
    simple_pick_and_place()