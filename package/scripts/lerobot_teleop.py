#!/usr/bin/env python3
"""
SO100 Teleoperation Bridge (LeRobot Style)
Cartesian Control: Leader FK -> Follower IK

Created by Sandor Burian with the help of Gemini Pro
"""

import sys
import os
import time
import numpy as np
from pathlib import Path

script_dir = Path(__file__).parent
package_root = script_dir.parent.parent
sys.path.insert(0, str(package_root))

from package.drivers.SO100_Robot.so100_core import SO100Robot as FollowerRobot
from package.drivers.SO100_Robot.leader_robot import SO100LeaderToCartesianControl as SO100Leader
from package.utils.input_utils import get_port_input

def run_teleop():
    print("\n === SO100 CARTESIAN TELEOPERATION === ")
    print("Leader: Forward Kinematics")
    print("Follower: Inverse Kinematics\n")

    # 1. Configuration  
    # Leader port request
    print("--- LEADER SETUP ---")
    leader_port = get_port_input("COM5")  # Working port for leader
    
    # Follower port request
    print("\n--- FOLLOWER SETUP ---")
    follower_port = get_port_input("COM4")  # Try COM4 for follower
    try:
        # 2. STARTING ROBOTS
        print("\n[INIT] Connecting robots...")
        
        # LEADER: Passive, reader mode
        # Automatically loads SO100leader_to_cartesian_calibration.csv
        leader = SO100Leader(port=leader_port)
        leader.torque_disable()  # Relax to allow manual movement
        print("[LEADER] Connected (Torque OFF)")

        # FOLLOWER: Active, executor mode
        # We explicitly provide the config folder for safety
        config_path = os.path.join(package_root, "package", "drivers", "SO100_Robot", "config")
        follower = FollowerRobot(port=follower_port, config_dir=config_path)
        follower.torque_enable(True) # hold position
        
        # Initial position capture (Optional: Home position)
        # follower.move_to_cartesian(0.15, 0.0, 0.15, time_ms=2000)
        print("[FOLLOWER] Connected (Torque ON)")

    except Exception as e:
        print(f"\n [CRITICAL ERROR]: {e}")
        return

    # 3. SYNCHRONIZATION
    print("\n--- WARNING! ---")
    print("Hold the Leader arm.")
    print("The Follower will immediately follow the movement.")
    input("Press ENTER to START!")

    print("\nTELEOP ACTIVE! (Ctrl+C to exit)")
    
    # Set cycle time (30 Hz = smooth video-like motion)
    FREQUENCY = 30
    dt = 1.0 / FREQUENCY
    
    try:
        while True:
            loop_start = time.time()
            
            # --- A. READ LEADER (XYZ + Gripper) ---
            # This function already calculates based on the calibrated 'Candle' zero point
            target_xyz, gripper_state = leader.get_cartesian_pose()
            x, y, z = target_xyz
            
            # DEBUG: Show raw joint angles to see if leader is actually moving
            joints = leader.get_joint_angles()
            print(f"\rJoints: {[f'{j:.2f}' for j in joints[:3]]} Target: X={x:.3f} Y={y:.3f} Z={z:.3f} Grip={gripper_state:.2f}", end="")
            
            # Store original coordinates for debugging
            original_x, original_y, original_z = x, y, z
            
            # --- B. SAFETY FILTERS (Safety Policy) ---
            
            # 1. Z-Limit (Table protection)
            # If you go below 2cm with the Leader, the Follower stops at 2cm.
            SAFE_Z_MIN = 0.02 
            if z < SAFE_Z_MIN: 
                z = SAFE_Z_MIN
            
            # 2. Workspace limits (prevent collision with base)
            # X: Allow negative values, but limit extreme positions
            SAFE_X_MIN = -0.35  # Allow reaching backwards
            SAFE_X_MAX = 0.35   # Limit forward reach
            SAFE_Y_MIN = -0.35  # Left limit
            SAFE_Y_MAX = 0.35   # Right limit
            SAFE_Z_MAX = 0.60   # Height limit - increased as both robots can reach similar height
            
            # Apply safety limits
            if x < SAFE_X_MIN: x = SAFE_X_MIN
            if x > SAFE_X_MAX: x = SAFE_X_MAX
            if y < SAFE_Y_MIN: y = SAFE_Y_MIN  
            if y > SAFE_Y_MAX: y = SAFE_Y_MAX
            if z > SAFE_Z_MAX: z = SAFE_Z_MAX

            # Debug: Show coordinate changes if safety limits were applied
            if abs(x - original_x) > 0.001 or abs(y - original_y) > 0.001 or abs(z - original_z) > 0.001:
                print(f"\n[SAFETY] Coords changed: ({original_x:.3f},{original_y:.3f},{original_z:.3f}) → ({x:.3f},{y:.3f},{z:.3f})")

            # Debug print (to see what the system senses)
            #print(f"\rTarget: X={x:.3f} Y={y:.3f} Z={z:.3f} Grip={gripper_state:.2f}", end="")

            # --- C. MOVE FOLLOWER ---
            
            # 1. Gripper
            # Simple logic: If the Leader is open (>0.5), open.
            if gripper_state > 0.5:
                follower.gripper_open()
            else:
                follower.gripper_close()

            # 2. Arm (Cartesian Move)
            # move_time_ms: We give exactly as much time as the cycle (dt).
            # This makes the movement smooth, not jerky.
            move_ms = int(dt * 1000)
            
            # Move to target position - the SO100Robot handles orientation automatically
            follower.move_to_cartesian(x, y, z, time_ms=move_ms)

            # --- D. Timing ---
            # If the calculation was faster than 33ms, wait for the remaining time.
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we are running late, do not wait (to catch up)
                pass

    except KeyboardInterrupt:
        print("\n\nExiting...")
        # Safety stop
    except Exception as e:
        print(f"\n[ERROR] ERROR DURING RUN: {e}")
    finally:
        print("Turning off motors...")
        try:
            leader.close()
            follower.close()
        except:
            pass
        print("Shutted down")

if __name__ == "__main__":
    run_teleop()