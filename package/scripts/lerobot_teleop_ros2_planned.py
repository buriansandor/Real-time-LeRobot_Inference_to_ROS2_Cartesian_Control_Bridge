#!/usr/bin/env python3
"""
SO100 Teleoperation with ROS2 Integration Planning

This version demonstrates how the current system could be enhanced with ROS2:
1. Current: Direct serial communication to robots
2. Future: ROS2 topics for position feedback and control
3. Future: MoveIt! integration for better path planning
4. Future: Real-time pose feedback and collision detection

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

# Future ROS2 integration would import:
# import rclpy
# from geometry_msgs.msg import Pose, PoseStamped
# from sensor_msgs.msg import JointState
# from moveit_msgs.action import MoveGroup

def run_teleop_ros2_ready():
    """
    Current implementation with annotations for future ROS2 integration
    """
    
    # Future: ROS2 node initialization
    # rclpy.init()
    # node = rclpy.create_node('so100_teleop_node')
    # pose_pub = node.create_publisher(PoseStamped, '/target_pose', 10)
    # joint_sub = node.create_subscription(JointState, '/joint_states', joint_callback, 10)
    
    print("\n === SO100 ROS2-READY TELEOPERATION === ")
    print("Current: Direct serial communication")
    print("Future: ROS2 topics + MoveIt! integration\n")

    # 1. INITIALIZE ROBOTS
    try:
        from package.utils.input_utils import get_port_input
        print("[PORTS] Detecting SO100 ports...")
        
        leader_port = get_port_input("Leader")
        follower_port = get_port_input("Follower")
        
    except (ImportError, Exception):
        print("[MANUAL] Using manual ports...")
        leader_port = "COM14"
        follower_port = "COM13"

    try:
        # LEADER: Passive, reader mode
        leader = SO100Leader(port=leader_port)
        leader.torque_disable()
        print("[LEADER] Connected (Torque OFF)")
        # Future: ROS2 publisher for leader pose

        # FOLLOWER: Active, executor mode  
        config_path = os.path.join(package_root, "package", "drivers", "SO100_Robot", "config")
        follower = FollowerRobot(port=follower_port, config_dir=config_path)
        follower.torque_enable(True)
        print("[FOLLOWER] Connected (Torque ON)")
        # Future: ROS2 action client for MoveIt!

    except Exception as e:
        print(f"\n [CRITICAL ERROR]: {e}")
        return

    print("\n--- ROS2 INTEGRATION PLAN ---")
    print("✓ Direct robot control (current)")
    print("○ ROS2 pose publishing")  
    print("○ MoveIt! path planning")
    print("○ Collision detection")
    print("○ Force feedback")
    print("○ Multi-robot coordination")
    
    # 2. TELEOPERATION LOOP
    print("\n--- WARNING! ---")
    print("Hold the Leader arm.")
    print("The Follower will immediately follow the movement.")
    print("Future: ROS2 will provide position feedback and safety features.")
    input("Press ENTER to START!")

    print("\nTELEOP ACTIVE! (Ctrl+C to exit)")
    
    # Set cycle time (30 Hz)
    FREQUENCY = 30
    dt = 1.0 / FREQUENCY
    
    try:
        while True:
            loop_start = time.time()
            
            # --- A. READ LEADER ---
            target_xyz, gripper_state = leader.get_cartesian_pose()
            target_x, target_y, target_z = target_xyz
            
            # Debug output
            joints = leader.get_joint_angles()
            print(f"\rJoints: {[f'{j:.2f}' for j in joints[:3]]} Target: X={target_x:.3f} Y={target_y:.3f} Z={target_z:.3f} Grip={gripper_state:.2f}", end="")
            
            # Future ROS2: Publish target pose
            # pose_msg = PoseStamped()
            # pose_msg.header.stamp = node.get_clock().now().to_msg()
            # pose_msg.header.frame_id = "base_link"
            # pose_msg.pose.position.x = target_x
            # pose_msg.pose.position.y = target_y
            # pose_msg.pose.position.z = target_z
            # pose_pub.publish(pose_msg)

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

            # Future ROS2: MoveIt! collision checking
            # collision_result = planning_scene.isStateValid(target_state)
            # if not collision_result:
            #     print("[ROS2] Collision detected! Stopping movement.")
            #     continue

            if abs(target_x - original_x) > 0.001 or abs(target_y - original_y) > 0.001 or abs(target_z - original_z) > 0.001:
                print(f"\n[SAFETY] Coords changed: ({original_x:.3f},{original_y:.3f},{original_z:.3f}) → ({target_x:.3f},{target_y:.3f},{target_z:.3f})")

            # --- C. MOVE FOLLOWER ---
            
            # 1. Gripper
            if gripper_state > 0.5:
                follower.gripper_open()
            else:
                follower.gripper_close()

            # 2. Arm movement
            # Current: Direct control
            move_ms = int(dt * 1000)
            follower.move_to_cartesian(target_x, target_y, target_z, time_ms=move_ms)
            
            # Future ROS2: MoveIt! action call
            # move_goal = MoveGroup.Goal()
            # move_goal.request.group_name = "manipulator"
            # target_pose = Pose()
            # target_pose.position.x = target_x
            # target_pose.position.y = target_y
            # target_pose.position.z = target_z
            # move_goal.request.goal_constraints = [pose_to_constraint(target_pose)]
            # action_client.send_goal_async(move_goal)

            # --- D. Future: Position Feedback ---
            # In ROS2, we would subscribe to /joint_states topic
            # to get real robot positions and compare with targets
            # This eliminates the position drift problem completely!

            # --- E. Timing ---
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            # Future ROS2: rclpy.spin_once(node, timeout_sec=sleep_time)

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
        # Future ROS2: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    print("=" * 60)
    print("SO100 Teleoperation - ROS2 Integration Roadmap")
    print("=" * 60)
    print()
    print("CURRENT STATUS: ✅ Working direct serial control")
    print("ISSUE: Position drift detection needs improvement")
    print()
    print("ROS2 SOLUTION BENEFITS:")
    print("  🔄 Real-time position feedback via /joint_states topic")
    print("  🛡️  MoveIt! collision detection and path planning") 
    print("  📊 Built-in monitoring and diagnostics")
    print("  🤝 Multi-robot coordination capabilities")
    print("  🎯 Precise trajectory execution")
    print()
    print("RECOMMENDATION: Implement ROS2 bridge for production use")
    print("=" * 60)
    print()
    
    response = input("Run current version anyway? (y/N): ").lower()
    if response == 'y':
        run_teleop_ros2_ready()
    else:
        print("Consider implementing ROS2 integration for better feedback control!")