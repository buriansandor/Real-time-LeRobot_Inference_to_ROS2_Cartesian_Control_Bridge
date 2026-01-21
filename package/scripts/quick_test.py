#!/usr/bin/env python3
"""
Quick test after calibration fix

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

def quick_test():
    print("\n=== QUICK CALIBRATION TEST ===")
    
    # Connect to robots
    leader = SO100Leader(port="COM5")
    leader.torque_disable()
    
    follower = FollowerRobot(port="COM4", config_dir=os.path.join(package_root, "package", "drivers", "SO100_Robot", "config"))
    
    print("\nMove leader to a neutral position and press ENTER...")
    input()
    
    # Read positions
    leader_joints = leader.get_joint_angles()
    leader_xyz, _ = leader.get_cartesian_pose()
    
    print(f"Leader joints: {[f'{j:.3f}' for j in leader_joints[:3]]}")
    print(f"Leader XYZ: X={leader_xyz[0]:.3f} Y={leader_xyz[1]:.3f} Z={leader_xyz[2]:.3f}")
    
    print("Moving follower to same coordinates...")
    follower.move_to_cartesian(leader_xyz[0], leader_xyz[1], leader_xyz[2], time_ms=2000)
    time.sleep(2.5)
    
    # Check result
    follower_joints = follower.get_joint_angles()
    print(f"Follower joints: {[f'{j:.3f}' for j in follower_joints[:3]]}")
    
    diffs = [leader_joints[i] - follower_joints[i] for i in range(3)]
    print(f"Differences: {[f'{d:.3f} ({d*180/3.14:.1f}°)' for d in diffs]}")
    
    max_error = max(abs(d) for d in diffs[:3])
    if max_error < 0.2:  # <11 degrees
        print("✅ MUCH BETTER! Calibration fix worked!")
    elif max_error < 0.5:  # <29 degrees  
        print("⚠️ Improved but still needs fine-tuning")
    else:
        print("❌ Still major issues")
    
    leader.close()
    follower.close()

if __name__ == "__main__":
    quick_test()