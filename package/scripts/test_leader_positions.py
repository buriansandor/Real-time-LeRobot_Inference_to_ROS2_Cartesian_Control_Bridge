#!/usr/bin/env python3
"""
Leader Position Analysis - Test specific movements
Analyze leader-follower coordination by testing predefined leader positions.

Created by Copilot (Claude Sonnet 4) with the help of Sandor Burian
Test key positions: up, forward, back, left, right
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

def test_positions():
    print("\n=== LEADER POSITION ANALYSIS ===")
    print("This script will test specific leader positions to identify coordination issues.")
    
    # Connect to robots
    print("\nConnecting to leader...")
    leader = SO100Leader(port="COM5")
    leader.torque_disable()
    
    print("Connecting to follower...")
    config_path = os.path.join(package_root, "package", "drivers", "SO100_Robot", "config")
    follower = FollowerRobot(port="COM4", config_dir=config_path)
    
    positions_to_test = [
        "1. NEUTRAL/CENTER - Robot in comfortable mid position",
        "2. UP - Reach maximum height", 
        "3. FORWARD - Extend arm forward",
        "4. BACK - Pull arm back toward body",
        "5. LEFT - Move arm to left side", 
        "6. RIGHT - Move arm to right side"
    ]
    
    print("\n=== POSITION TEST SEQUENCE ===")
    print("Move the LEADER robot to each position and press ENTER:")
    
    results = []
    
    for i, position_desc in enumerate(positions_to_test):
        print(f"\n{position_desc}")
        input("Position the leader robot and press ENTER...")
        
        # Read positions
        leader_joints = leader.get_joint_angles()
        follower_joints = follower.get_joint_angles()
        leader_xyz, leader_grip = leader.get_cartesian_pose()
        
        # Calculate what follower should do with same coordinates
        print(f"Leader XYZ: X={leader_xyz[0]:.3f} Y={leader_xyz[1]:.3f} Z={leader_xyz[2]:.3f}")
        
        # Test movement
        print("Moving follower to same coordinates...")
        try:
            follower.move_to_cartesian(leader_xyz[0], leader_xyz[1], leader_xyz[2], time_ms=2000)
            time.sleep(2.5)  # Wait for completion
            
            # Read follower position after movement
            final_follower_joints = follower.get_joint_angles()
            
            # Store results
            result = {
                'position': position_desc,
                'leader_xyz': leader_xyz,
                'leader_joints': leader_joints[:3],
                'follower_joints_before': follower_joints[:3], 
                'follower_joints_after': final_follower_joints[:3]
            }
            results.append(result)
            
            print(f"Leader joints:  {[f'{j:.3f}' for j in leader_joints[:3]]}")
            print(f"Follower after: {[f'{j:.3f}' for j in final_follower_joints[:3]]}")
            
            # Calculate joint differences
            joint_diffs = [leader_joints[i] - final_follower_joints[i] for i in range(3)]
            print(f"Joint diffs:    {[f'{d:.3f}' for d in joint_diffs]}")
            
        except Exception as e:
            print(f"Movement failed: {e}")
            
    print("\n=== ANALYSIS SUMMARY ===")
    print("Position | Leader Joints | Follower Joints | Differences")
    print("-" * 70)
    
    for result in results:
        pos_name = result['position'].split('.')[1].split(' -')[0].strip()
        leader_j = [f'{j:.2f}' for j in result['leader_joints']]
        follower_j = [f'{j:.2f}' for j in result['follower_joints_after']]
        diffs = [f'{result["leader_joints"][i] - result["follower_joints_after"][i]:.2f}' 
                for i in range(3)]
        
        print(f"{pos_name:8} | {str(leader_j):15} | {str(follower_j):16} | {str(diffs)}")
    
    print("\n=== DIAGNOSIS ===")
    # Find which motor has consistent issues
    motor_issues = [[] for _ in range(3)]
    for result in results:
        for i in range(3):
            diff = result['leader_joints'][i] - result['follower_joints_after'][i]
            motor_issues[i].append(abs(diff))
    
    for i in range(3):
        avg_error = np.mean(motor_issues[i])
        max_error = max(motor_issues[i])
        print(f"Motor {i}: Avg error={avg_error:.3f} rad ({avg_error*180/3.14:.1f}°), Max error={max_error:.3f} rad ({max_error*180/3.14:.1f}°)")
        
        if avg_error > 0.1:  # ~6 degrees
            print(f"  ❌ Motor {i} has significant coordination issues!")
        elif max_error > 0.2:  # ~11 degrees  
            print(f"  ⚠️  Motor {i} has occasional large errors")
        else:
            print(f"  ✅ Motor {i} coordination looks good")
    
    # Cleanup
    leader.close()
    follower.close()

if __name__ == "__main__":
    try:
        test_positions()
    except Exception as e:
        print(f"Error: {e}")