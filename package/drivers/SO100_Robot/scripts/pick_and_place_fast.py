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
from pathlib import Path

# Add the package root to Python path 
script_dir = Path(__file__).parent
package_root = script_dir.parent.parent.parent
sys.path.insert(0, str(package_root))

from drivers.SO100_Robot import SO100Robot
from utils.input_utils import get_robot_configuration


def simple_pick_and_place():
    # Get robot configuration using the utility function
    config = get_robot_configuration()
    PORT = config['port']
    CALIB_FILE = config['calib_file'] 
    URDF_PATH = config['urdf_path']
      
    robot = SO100Robot(port=PORT, config_dir="config")

    print("--- PICK & PLACE DEMO ---")
    
    
    #GRIPPER_OPEN = robot.get_gripper_open_position() 
    #GRIPPER_CLOSE = robot.get_gripper_close_position()
    
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
            move_time = 800 # 2 seconds (safe speed)
            
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
    #print("Open positions:", GRIPPER_OPEN)
    #print("Close position:", GRIPPER_CLOSE)
    print("Waking up motors...")
    robot.torque_enable(True)

    print("\n\nCurrent calibration values:\n", robot.getCalibrationvalues()["offsets"],"\n",robot.getCalibrationvalues()["directions"],"\n", robot.getCalibrationvalues()["adjustments"], "\n", robot.getCalibrationvalues()["gripper_limits"])
    a = input("Press x to exit the pick and place demo...")
    if a == "x":
        print("Exiting...")
        return

    # 1. Open gripper and move to pick position
    robot.gripper_open()
    robot.move_to_cartesian(PICK_POS[0], PICK_POS[1], SAFE_Z, time_ms=300)  # Fast move to safe height
    robot.move_to_cartesian(PICK_POS[0], PICK_POS[1], PICK_POS[2], time_ms=300)  # Fast descend
    
    # 2. Close gripper and lift
    robot.gripper_close()
    robot.move_to_cartesian(PICK_POS[0], PICK_POS[1], SAFE_Z, time_ms=300)  # Fast lift
    
    # 3. Move to place position
    robot.move_to_cartesian(PLACE_POS[0], PLACE_POS[1], PLACE_POS[2], time_ms=300)  # Fast move to place
    
    # 4. Open gripper and retreat
    robot.gripper_open()
    robot.move_to_cartesian(PLACE_POS[0], PLACE_POS[1], SAFE_Z, time_ms=300)  # Fast retreat
    
    robot.gripper_close()

    print("Test done!")
    finish = input("Press x to release the arm!")
    if finish == "x":
        print("Exiting. Releasing motors...")
        robot.torque_enable(False)

if __name__ == "__main__":
    simple_pick_and_place()