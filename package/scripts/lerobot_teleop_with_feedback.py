#!/usr/bin/env python3
"""
SO100 Leader-Follower Teleoperation with Position Feedback and Correction

Enhanced version with feedback loop that:
1. Reads leader position
2. Sends command to follower
3. Reads actual follower position
4. Applies correction if there's drift

Created by Sandor Burian with the help of GitHub Copilot
"""

import os
import sys
import time
from pathlib import Path
import numpy as np

# Add package root to Python path
script_dir = Path(__file__).parent
package_root = script_dir.parent.parent
sys.path.insert(0, str(package_root))

from package.drivers.SO100_Robot.so100_core import SO100Robot as FollowerRobot
from package.drivers.SO100_Robot.leader_robot import SO100LeaderToCartesianControl as SO100Leader

def run_teleop_with_feedback():
    # 1. INITIALIZE ROBOTS
    
    # Automatic port detection
    try:
        from package.utils.input_utils import get_port_input
        print("[AUTO] Detecting SO100 ports...")
        
        leader_port = get_port_input("COM5")
        follower_port = get_port_input("COM4")
        
    except (ImportError, Exception):
        print("[MANUAL] Auto-detection failed, using manual ports...")
        leader_port = "COM14"    # Update these for your setup
        follower_port = "COM13"
    
    try:
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
        print("[FOLLOWER] Connected (Torque ON)")

    except Exception as e:
        print(f"\n [CRITICAL ERROR]: {e}")
        return

    # 2. SYNCHRONIZATION
    print("\n--- WARNING! ---")
    print("Hold the Leader arm.")
    print("The Follower will follow with position feedback correction.")
    input("Press ENTER to START!")

    print("\nTELEOP WITH FEEDBACK ACTIVE! (Ctrl+C to exit)")
    
    # Set cycle time (30 Hz = smooth video-like motion)
    FREQUENCY = 30
    dt = 1.0 / FREQUENCY
    
    # Feedback parameters
    POSITION_TOLERANCE = 0.005  # 5mm tolerance
    MAX_CORRECTION_SPEED = 0.02  # Max 2cm correction per cycle
    DRIFT_THRESHOLD = 0.015     # 15mm drift before correction
    
    # State variables
    target_position = None
    accumulated_error = np.array([0.0, 0.0, 0.0])
    correction_active = False
    
    try:
        while True:
            loop_start = time.time()
            
            # --- A. READ LEADER (XYZ + Gripper) ---
            target_xyz, gripper_state = leader.get_cartesian_pose()
            target_x, target_y, target_z = target_xyz
            
            # DEBUG: Show raw joint angles
            joints = leader.get_joint_angles()
            print(f"\rTarget: X={target_x:.3f} Y={target_y:.3f} Z={target_z:.3f} Grip={gripper_state:.2f}", end="")
            
            # --- B. SAFETY FILTERS ---
            original_x, original_y, original_z = target_x, target_y, target_z
            
            # Z-Limit (Table protection)
            SAFE_Z_MIN = 0.02 
            if target_z < SAFE_Z_MIN: 
                target_z = SAFE_Z_MIN
            
            # Workspace limits
            SAFE_X_MIN, SAFE_X_MAX = -0.35, 0.35
            SAFE_Y_MIN, SAFE_Y_MAX = -0.35, 0.35
            SAFE_Z_MAX = 0.60
            
            target_x = np.clip(target_x, SAFE_X_MIN, SAFE_X_MAX)
            target_y = np.clip(target_y, SAFE_Y_MIN, SAFE_Y_MAX)
            target_z = np.clip(target_z, 0, SAFE_Z_MAX)

            if abs(target_x - original_x) > 0.001 or abs(target_y - original_y) > 0.001 or abs(target_z - original_z) > 0.001:
                print(f"\n[SAFETY] Coords changed: ({original_x:.3f},{original_y:.3f},{original_z:.3f}) → ({target_x:.3f},{target_y:.3f},{target_z:.3f})")

            # --- C. FEEDBACK CORRECTION ---
            
            # Read actual follower position
            try:
                actual_xyz = follower.get_cartesian_position()
                actual_x, actual_y, actual_z = actual_xyz
                
                # Calculate position error
                error = np.array([target_x - actual_x, target_y - actual_y, target_z - actual_z])
                error_magnitude = np.linalg.norm(error)
                
                # Check if correction is needed
                if error_magnitude > DRIFT_THRESHOLD:
                    if not correction_active:
                        print(f"\n[FEEDBACK] Large drift detected: {error_magnitude*1000:.1f}mm")
                        correction_active = True
                    
                    # Apply correction with velocity limit
                    correction = error.copy()
                    correction_magnitude = np.linalg.norm(correction)
                    
                    if correction_magnitude > MAX_CORRECTION_SPEED:
                        correction = correction * (MAX_CORRECTION_SPEED / correction_magnitude)
                    
                    # Apply correction to target
                    corrected_x = actual_x + correction[0]
                    corrected_y = actual_y + correction[1] 
                    corrected_z = actual_z + correction[2]
                    
                    print(f" [CORRECTING] Error: {error_magnitude*1000:.1f}mm", end="")
                    
                    target_position = [corrected_x, corrected_y, corrected_z]
                    
                elif error_magnitude < POSITION_TOLERANCE and correction_active:
                    print(f"\n[FEEDBACK] Position corrected! Error: {error_magnitude*1000:.1f}mm")
                    correction_active = False
                    target_position = [target_x, target_y, target_z]
                    
                else:
                    target_position = [target_x, target_y, target_z]
                    if correction_active and error_magnitude < POSITION_TOLERANCE * 2:
                        correction_active = False
                
                # Show status
                if error_magnitude > POSITION_TOLERANCE:
                    print(f" [Err: {error_magnitude*1000:.1f}mm]", end="")
                    
            except Exception as e:
                # Fallback: use target position if feedback fails
                target_position = [target_x, target_y, target_z]
                print(f" [FEEDBACK ERROR: {e}]", end="")

            # --- D. MOVE FOLLOWER ---
            
            # 1. Gripper
            if gripper_state > 0.5:
                follower.gripper_open()
            else:
                follower.gripper_close()

            # 2. Arm movement
            move_ms = int(dt * 1000)
            
            if target_position:
                follower.move_to_cartesian(target_position[0], target_position[1], target_position[2], time_ms=move_ms)

            # --- E. Timing ---
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nExiting...")
    except Exception as e:
        print(f"\n[ERROR] ERROR DURING RUN: {e}")
    finally:
        print("Turning off motors...")
        try:
            leader.close()
            follower.close()
        except:
            pass
        print("Shutdown complete")

if __name__ == "__main__":
    run_teleop_with_feedback()