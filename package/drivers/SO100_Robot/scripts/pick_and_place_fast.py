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
import ikpy.chain

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
    
    def move_to_fast(robot, x, y, z, time_ms=500):  # Balanced speed - 500ms
        """Direct set_target_angle method like original demo"""
        print(f"Move to: {x}, {y}, {z}")
        
        # Fix: Use robot.kinematics.chain instead of robot.chain
        seed_state = [0] * len(robot.kinematics.chain.links) 
        target_joints = robot.kinematics.chain.inverse_kinematics(
            target_position=[x, y, z], 
            initial_position=seed_state
        )
            
        # Printing the calculated angles (in Radians)
        print("Calculated angles (Radians):")
        for i in range(1, min(7, len(target_joints))):
            print(f"  J{i}: {target_joints[i]:.2f}")

        # DIRECT MOTOR CONTROL (like original working demo)
        print("Moving...")
        for motor_idx in range(5):  # 0..4 (Motor 1..5, skip gripper)
            ik_angle = target_joints[motor_idx + 1]  # Skip base
            
            # Direct call like original demo
            robot.set_target_angle(motor_idx, ik_angle, move_time_ms=time_ms)
            time.sleep(0.05)  # Original demo timing - more reliable
        
        # Wait for completion
        time.sleep(time_ms / 1000.0 + 0.1)  # Original buffer timing

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
    time.sleep(1)  # Reduced gripper wait time
    
    move_to_fast(robot, PICK_POS[0], PICK_POS[1], SAFE_Z, time_ms=400)  # Fast movement
    
    move_to_fast(robot, PICK_POS[0], PICK_POS[1], PICK_POS[2], time_ms=400)  # Descend
    
    # 2. Close gripper and lift
    robot.gripper_close()
    time.sleep(1)  # Reduced gripper wait time
    
    move_to_fast(robot, PICK_POS[0], PICK_POS[1], SAFE_Z, time_ms=400)  # Lift
    
    # 3. Move to place position
    move_to_fast(robot, PLACE_POS[0], PLACE_POS[1], PLACE_POS[2], time_ms=400)  # Move to place
    
    # 4. Open gripper and retreat
    robot.gripper_open()
    time.sleep(1)  # Reduced gripper wait time
    
    move_to_fast(robot, PLACE_POS[0], PLACE_POS[1], SAFE_Z, time_ms=400)  # Retreat
    
    robot.gripper_close()

    print("Test done!")
    finish = input("Press x to release the arm!")
    if finish == "x":
        print("Exiting. Releasing motors...")
        robot.torque_enable(False)

if __name__ == "__main__":
    simple_pick_and_place()