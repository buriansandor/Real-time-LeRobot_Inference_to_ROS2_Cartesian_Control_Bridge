#!/usr/bin/env python3
"""
Debug Coordination - Analyze leader and follower coordinate mapping
Created by Copilot (Claude Sonnet 4) with the help of Sandor Burian
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

def analyze_coordination():
    print("\n=== COORDINATION ANALYSIS ===")
    
    # Connect to both robots
    print("Connecting to leader...")
    leader = SO100Leader(port="COM5")
    leader.torque_disable()
    
    print("Connecting to follower...")
    config_path = os.path.join(package_root, "package", "drivers", "SO100_Robot", "config")
    follower = FollowerRobot(port="COM4", config_dir=config_path)
    
    print("\n=== STATIC ANALYSIS ===")
    print("Reading current positions...")
    
    # Read current joint positions
    leader_joints = leader.get_joint_angles()
    follower_joints = follower.get_joint_angles()
    
    print(f"Leader joints (rad):    {[f'{j:.3f}' for j in leader_joints[:3]]}")
    print(f"Follower joints (rad):  {[f'{j:.3f}' for j in follower_joints[:3]]}")
    
    # Get cartesian coordinates
    leader_xyz, leader_grip = leader.get_cartesian_pose()
    
    print(f"\nLeader cartesian:   X={leader_xyz[0]:.3f} Y={leader_xyz[1]:.3f} Z={leader_xyz[2]:.3f}")
    print(f"Leader gripper:     {leader_grip:.3f}")
    
    print("\nSkipping follower cartesian (focusing on joint comparison)...")
    
    # Calculate difference in joint space instead
    joint_diff = [leader_joints[i] - follower_joints[i] for i in range(min(len(leader_joints), len(follower_joints)))]
    print(f"\nJoint differences (Leader - Follower):")
    for i, diff in enumerate(joint_diff[:3]):  # Only first 3 motors
        print(f"  Motor {i}: {diff:.3f} rad ({diff * 180/3.14159:.1f}°)")
    
    print("\n=== CALIBRATION COMPARISON ===")
    print("Leader offsets:", leader.offsets)
    print("Leader directions:", leader.directions)
    print("Leader adjustments:", leader.adjustments)
    
    print("\nFollower offsets:", follower.calibration_offsets)
    print("Follower directions:", follower.calibration_directions)
    print("Follower adjustments:", follower.calibration_adjustments)
    
    # Check for differences
    leader_offs = [leader.offsets[i] for i in range(6)]
    follower_offs = follower.calibration_offsets
    
    print("\nOffset differences:")
    for i in range(3):  # Only first 3 motors for position
        diff = leader_offs[i] - follower_offs[i]
        print(f"Motor {i}: Leader={leader_offs[i]}, Follower={follower_offs[i]}, Diff={diff}")
    
    print("\n=== RECOMMENDATION ===")
    max_joint_diff = max(abs(d) for d in joint_diff[:3])
    if max_joint_diff > 0.2:  # ~11 degrees
        print("❌ Large joint differences detected!")
        print("The robots are not in similar poses.")
        print("\nSuggested fixes:")
        print("1. Move both robots to similar 'candle' position")
        print("2. Re-calibrate leader zero point")
        print("3. Check offset differences in calibration files")
        
        print("\nRAW POSITION ANALYSIS:")
        print("Check if raw motor values are similar when robots are in same pose")
    else:
        print("✅ Joint angles relatively close")
        print("Problem might be in coordinate transformation")
    
    # Cleanup
    leader.close()
    follower.close()

if __name__ == "__main__":
    try:
        analyze_coordination()
    except Exception as e:
        print(f"Error: {e}")