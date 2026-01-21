#!/usr/bin/env python3
"""
Manual Calibration Helper
Help user manually calibrate leader robot to match follower
Created by Copilot (Claude Sonnet 4) with the help of Sandor Burian
"""

import sys
import os
import time
from pathlib import Path

script_dir = Path(__file__).parent
package_root = script_dir.parent.parent
sys.path.insert(0, str(package_root))

from package.drivers.SO100_Robot.so100_core import SO100Robot as FollowerRobot
from package.drivers.SO100_Robot.leader_robot import SO100LeaderToCartesianControl as SO100Leader

def manual_calibration():
    print("\n=== MANUAL CALIBRATION HELPER ===")
    print("This will help you find the correct offsets by testing raw motor positions")
    
    # Connect to robots
    leader = SO100Leader(port="COM5")
    leader.torque_disable()
    
    follower = FollowerRobot(port="COM4", config_dir=os.path.join(package_root, "package", "drivers", "SO100_Robot", "config"))
    
    print("\n🎯 GOAL: Make both robots reach EXACTLY the same physical position")
    print("📝 We'll find what raw values produce the same angles\n")
    
    # Test specific target position
    test_angles = [0.0, -1.5, -1.0]  # Reasonable test position
    
    print(f"TARGET: Move follower to joint angles: {test_angles}")
    follower.move_to_joints(test_angles + [0, 0, 0], time_ms=3000)
    time.sleep(3.5)
    
    # Read actual follower position
    actual_follower_joints = follower.get_joint_angles()
    print(f"Follower reached: {[f'{j:.3f}' for j in actual_follower_joints[:3]]}")
    
    print("\n🤖 Now manually move the LEADER robot to match the EXACT same physical position")
    print("   - Same shoulder angle")
    print("   - Same elbow angle")  
    print("   - Same wrist angle")
    print("   - Make them look identical!")
    
    input("Position the leader and press ENTER...")
    
    # Read leader raw values
    leader_joints = leader.get_joint_angles()
    print(f"Leader angles: {[f'{j:.3f}' for j in leader_joints[:3]]}")
    
    # Read raw motor values for calibration
    leader_raw = []
    for i in range(3):
        raw = leader._read_raw_position(i + 1)
        leader_raw.append(raw)
    
    # For follower, we'll use a simpler approach - just use the known calibration values
    follower_raw = []
    # Follower calibration values (from follower_calibration.csv)
    follower_offsets = [2134, 2032, 1057]
    follower_directions = [-1, 1, -1] 
    follower_adjustments = [0.0, 0.1, -0.95]
    
    for i in range(3):
        # Get current angle
        angle = actual_follower_joints[i] 
        # Reverse calculation to get raw value
        offset = follower_offsets[i]
        direction = follower_directions[i] 
        adjustment = follower_adjustments[i]
        ratio = (2 * 3.14159 / 4096)
        
        # Reverse: raw = ((angle - adjustment) / (ratio * direction)) + offset
        raw_val = ((angle - adjustment) / (ratio * direction)) + offset
        follower_raw.append(int(raw_val))
        
    print(f"\nRAW VALUES WHEN IN SAME POSITION:")
    print(f"Leader raw:    {leader_raw}")
    print(f"Follower raw:  {follower_raw}")
    
    # Calculate required offsets
    print(f"\n📊 CALIBRATION ANALYSIS:")
    current_leader_offsets = [leader.offsets[i] for i in range(3)]
    follower_offsets = [2134, 2032, 1057]  # Known follower values
    
    print(f"Current leader offsets: {current_leader_offsets}")
    print(f"Follower offsets:       {follower_offsets}")
    
    # Raw difference when robots are in same position
    raw_diffs = [leader_raw[i] - follower_raw[i] for i in range(3)]
    print(f"Raw differences:        {raw_diffs}")
    
    # Suggested new offsets
    new_offsets = []
    for i in range(3):
        # If leader raw is higher, we need to increase leader offset
        suggested = follower_offsets[i] + raw_diffs[i]
        new_offsets.append(suggested)
    
    print(f"\n✅ SUGGESTED NEW LEADER OFFSETS:")
    print(f"Motor 0: {current_leader_offsets[0]} → {new_offsets[0]} (change: {new_offsets[0] - current_leader_offsets[0]:+d})")
    print(f"Motor 1: {current_leader_offsets[1]} → {new_offsets[1]} (change: {new_offsets[1] - current_leader_offsets[1]:+d})")  
    print(f"Motor 2: {current_leader_offsets[2]} → {new_offsets[2]} (change: {new_offsets[2] - current_leader_offsets[2]:+d})")
    
    print(f"\n📝 UPDATE CSV FILE:")
    print(f"ZERO_OFFSETS,{new_offsets[0]},{new_offsets[1]},{new_offsets[2]},2020,897,1840")
    
    leader.close()
    follower.close()

if __name__ == "__main__":
    manual_calibration()