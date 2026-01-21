#!/usr/bin/env python3
"""
SO100 Leader-Follower Teleoperation with Joint Angle Feedback

Simple joint-based feedback that:
1. Reads leader joint angles
2. Sends commands to follower
3. Compares actual follower joint angles
4. Shows drift warnings if joints differ significantly

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

def run_teleop_joint_feedback():
    # 1. INITIALIZE ROBOTS
    
    # Automatic port detection
    try:
        from package.utils.input_utils import get_port_input
        print("[AUTO] Detecting SO100 ports...")
        
        leader_port = get_port_input("Leader")
        follower_port = get_port_input("Follower")
        
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
    print("The Follower will follow with joint angle drift detection.")
    input("Press ENTER to START!")

    print("\nTELEOP WITH JOINT FEEDBACK ACTIVE! (Ctrl+C to exit)")
    
    # Set cycle time (30 Hz = smooth video-like motion)
    FREQUENCY = 30
    dt = 1.0 / FREQUENCY
    
    # Joint drift parameters (in radians)
    JOINT_DRIFT_THRESHOLD = 0.2  # 11.5 degrees
    MAX_JOINT_WARNING = 0.5      # 28.6 degrees
    
    # State variables
    drift_warnings = 0
    last_drift_time = 0
    
    try:
        while True:
            loop_start = time.time()
            
            # --- A. READ LEADER (XYZ + Gripper) ---
            target_xyz, gripper_state = leader.get_cartesian_pose()
            target_x, target_y, target_z = target_xyz
            
            # Debug: Show raw joint angles
            leader_joints = leader.get_joint_angles()
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

            # --- C. JOINT DRIFT DETECTION ---
            
            # Read actual follower joint angles
            try:
                follower_joints = follower.get_joint_angles()
                
                # Calculate joint differences (first 5 joints, skip gripper)
                joint_diffs = []
                for i in range(min(5, len(leader_joints), len(follower_joints))):
                    diff = abs(leader_joints[i] - follower_joints[i])
                    joint_diffs.append(diff)
                
                # Check for significant drift
                max_diff = max(joint_diffs) if joint_diffs else 0
                max_diff_idx = joint_diffs.index(max_diff) if joint_diffs else 0
                
                current_time = time.time()
                
                if max_diff > MAX_JOINT_WARNING:
                    # Critical drift
                    if current_time - last_drift_time > 2.0:  # Don't spam warnings
                        print(f"\n[CRITICAL DRIFT] Joint {max_diff_idx+1}: {max_diff:.3f} rad ({max_diff*57.3:.1f}°)")
                        last_drift_time = current_time
                        drift_warnings += 1
                        
                elif max_diff > JOINT_DRIFT_THRESHOLD:
                    # Minor drift
                    if current_time - last_drift_time > 5.0:  # Less frequent warnings
                        print(f"\n[DRIFT] Joint {max_diff_idx+1}: {max_diff:.3f} rad ({max_diff*57.3:.1f}°)")
                        last_drift_time = current_time
                        
                # Show drift status in main output
                if max_diff > JOINT_DRIFT_THRESHOLD:
                    print(f" [J-Drift: {max_diff*57.3:.1f}°]", end="")
                    
            except Exception as e:
                # Fallback: show error but continue
                print(f" [JOINT READ ERROR: {e}]", end="")

            # --- D. MOVE FOLLOWER ---
            
            # 1. Gripper
            if gripper_state > 0.5:
                follower.gripper_open()
            else:
                follower.gripper_close()

            # 2. Arm movement (simple, no feedback correction)
            move_ms = int(dt * 1000)
            follower.move_to_cartesian(target_x, target_y, target_z, time_ms=move_ms)

            # --- E. Timing ---
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nExiting...")
        if drift_warnings > 0:
            print(f"[INFO] Total drift warnings: {drift_warnings}")
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
    run_teleop_joint_feedback()