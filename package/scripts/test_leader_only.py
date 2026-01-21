#!/usr/bin/env python3
"""
SO100 Leader Robot Test - Single Robot Testing
Tests if the leader robot reads joint positions correctly

Created by Copilot
"""

import sys
import os
import time
from pathlib import Path

script_dir = Path(__file__).parent
package_root = script_dir.parent.parent
sys.path.insert(0, str(package_root))

from package.drivers.SO100_Robot.leader_robot import SO100LeaderToCartesianControl as SO100Leader

def test_leader_robot():
    print("\n === SO100 LEADER ROBOT TEST ===")
    print("Testing COM4 robot as leader...")
    
    try:
        # Test with COM4
        leader = SO100Leader(port="COM4")
        leader.torque_disable()  # Allow manual movement
        print("[LEADER] Connected and motors released!")
        
        print("\n--- TEST INSTRUCTIONS ---")
        print("1. Move the robot by hand")
        print("2. Watch the joint angles and XYZ coordinates")
        print("3. Values should change as you move the robot")
        print("4. Press Ctrl+C to exit")
        
        input("\nPress ENTER to start position monitoring!")
        
        print("\nLeader position monitoring active...\n")
        
        while True:
            # Read joint angles and cartesian position
            joints = leader.get_joint_angles()
            xyz, gripper = leader.get_cartesian_pose()
            
            print(f"Joints: [{joints[0]:.2f}, {joints[1]:.2f}, {joints[2]:.2f}, {joints[3]:.2f}, {joints[4]:.2f}]")
            print(f"XYZ: ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) | Gripper: {gripper:.2f}")
            print("=" * 60)
            
            time.sleep(0.5)  # Update every 0.5 seconds
            
    except KeyboardInterrupt:
        print("\n\nTest stopped by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        print("\nTroubleshooting:")
        print("- Make sure robot is powered on")
        print("- Check USB cable connection")  
        print("- Try running: lerobot-find-port")
    finally:
        try:
            leader.close()
            print("Robot connection closed.")
        except:
            pass

if __name__ == "__main__":
    test_leader_robot()