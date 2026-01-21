#!/usr/bin/env python3
"""
Quick gripper test
"""

import sys
import os
from pathlib import Path

script_dir = Path(__file__).parent
package_root = script_dir.parent.parent
sys.path.insert(0, str(package_root))

from package.drivers.SO100_Robot.leader_robot import SO100LeaderToCartesianControl as SO100Leader

def test_gripper():
    print("\n=== GRIPPER TEST ===")
    
    leader = SO100Leader(port="COM5")
    leader.torque_disable()
    
    print("Testing gripper readings...")
    for i in range(5):
        xyz, grip = leader.get_cartesian_pose()
        raw_grip = leader._read_raw_position(6)
        print(f"Gripper: {grip:.3f} (raw: {raw_grip})")
        
        input("Manually move gripper and press ENTER...")
    
    leader.close()

if __name__ == "__main__":
    test_gripper()