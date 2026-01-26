#!/usr/bin/env python3
"""
FOLLOWER NODE (Subscriber) - SWAP OFF (DIRECT MAPPING)

Created by Google Gemini Pro based on previous versions.

Reads from the Leader robot via ZMQ -> Directly controls the Follower robot.
Logs everything to 'follower_log.txt'.
"""
import sys
import time
import zmq
import numpy as np
import os
import traceback
from pathlib import Path

# --- LOGGER SETUP ---
class DualLogger(object):
    """
    Logger that writes output to both the terminal and a log file.
    """
    def __init__(self, filename="follower_log.txt"):
        self.terminal = sys.stdout
        self.log = open(filename, "w", encoding="utf-8")
    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
        self.log.flush()
    def flush(self):
        self.terminal.flush()
        self.log.flush()

# Redirect stdout and stderr to DualLogger
sys.stdout = DualLogger()
sys.stderr = sys.stdout

# --- CONFIGURATION: BACK TO BASICS ---

# 1. AXIS SWAP DISABLED
# Since moving forward went sideways, we remove this now.
SWAP_XY = False     

# 2. MIRRORING (Default state)
MIRROR_X = False   # Mirror X axis if True
MIRROR_Y = False   # Mirror Y axis if True
MIRROR_Z = False   # Mirror Z axis if True

# 3. OFFSET (Z offset)
Z_OFFSET = 0.0     # Offset added to Z axis

# --- PATHS ---
# Set up paths for importing custom modules
current_script_dir = Path(__file__).parent.resolve()
package_dir = current_script_dir.parent
drivers_root = package_dir / "drivers" / "SO100_Robot"
core_dir = drivers_root / "so100_core"
utils_dir = package_dir / "utils"

paths_to_add = [drivers_root, core_dir, utils_dir]
for p in paths_to_add:
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

# Import required modules, exit if not found
try:
    import torch
    import torch.nn as nn
    from so100_driver import SO100Robot as FollowerRobot
    from kinematics import SO100Kinematics
    from input_utils import get_port_input
except ImportError as e:
    print(f"[ERROR] IMPORT ERROR: {e}")
    sys.exit(1)

# ================= POLICY =================
class RobustPolicy(nn.Module):
    """
    Policy for mapping leader robot joint states to follower robot commands.
    Applies kinematic transformations, safety checks, and inverse kinematics.
    """
    def __init__(self, urdf_path):
        super().__init__()
        print(f"[INFO] Loading kinematics: {urdf_path}")
        self.kinematics = SO100Kinematics(urdf_path)
        self.chain_length = len(self.kinematics.chain.links)
        
        # Safety limits for Z and X axes
        self.safe_z = 0.02
        self.safe_x = -0.03

    def forward(self, leader_joints, leader_gripper, current_follower_joints):
        # 1. FK (Leader) - Compute end-effector position from leader joints
        arm_joints = leader_joints[:5]
        target_xyz = self.kinematics.forward_kinematics(arm_joints)
        x, y, z = target_xyz
        
        # Log the raw input
        print(f"[RAW] X={x:.2f} Y={y:.2f} Z={z:.2f} | ", end="")

        # 2. TRANSFORMATIONS
        # Apply axis swap and mirroring if enabled
        if SWAP_XY: x, y = y, x
        if MIRROR_X: x = -x
        if MIRROR_Y: y = -y
        if MIRROR_Z: z = -z 
        
        z += Z_OFFSET  # Apply Z offset

        # 3. SAFETY
        # Enforce minimum Z height for safety
        if z < self.safe_z: z = self.safe_z
        # if x < self.safe_x: x = self.safe_x # Safety temporarily disabled for X
        
        print(f"-> [TGT] X={x:.2f} Y={y:.2f} Z={z:.2f}")

        # 4. IK (Follower) - Compute follower joint angles from target position
        seed_state = [0.0] * self.chain_length
        if len(current_follower_joints) == 5 and self.chain_length >= 6:
            seed_state[1:6] = current_follower_joints
        
        follower_joints_full = self.kinematics.inverse_kinematics(
            target_pos=[x, y, z],
            orientation_mode=None,
            seed_state=seed_state 
        )
        
        action_joints = follower_joints_full[1:6]  # Extract relevant joints
        target_grip = 1.0 if leader_gripper > 0.5 else 0.0  # Map gripper state
        return list(action_joints), target_grip

# ================= MAIN =================
def run_follower_node():
    """
    Main loop for the follower node.
    Receives leader robot state via ZMQ and sends commands to the follower robot.
    """
    print("\n--- ZMQ FOLLOWER NODE (DIRECT MAPPING) ---")
    print(f"CONFIG: SWAP_XY={SWAP_XY}, MIRROR=[X:{MIRROR_X} Y:{MIRROR_Y} Z:{MIRROR_Z}]")
    
    # --- ZMQ SETUP ---
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)  # Only keep latest message
    socket.connect("tcp://localhost:5555") 
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics
    
    print("--- FOLLOWER SETUP ---")
    port = get_port_input("COM4")  # Get serial port for follower robot
    config_dir = drivers_root / "config"
    urdf_path = config_dir / "so100.urdf"
    
    follower = None
    try:
        # Initialize follower robot and policy
        follower = FollowerRobot(port=port, config_dir=str(config_dir))
        follower.torque_enable(True)
        policy = RobustPolicy(str(urdf_path))
    except Exception as e:
        print(f"[ERROR] Error: {e}")
        return

    print("[INFO] STARTING! (Ctrl+C to stop)")
    last_known_joints = [0.0] * 5  # Initialize last known joint states

    try:
        while True:
            try:
                # Receive leader robot state from ZMQ
                msg = socket.recv_json()
                
                # Compute target follower joints and gripper state
                target_joints, target_grip = policy.forward(
                    msg['joints'], 
                    msg['gripper'], 
                    last_known_joints
                )
                
                # Control follower gripper based on leader gripper state
                if target_grip > 0.5: follower.gripper_open()
                else: follower.gripper_close()

                # Send joint command to follower robot
                full_command = target_joints + [target_grip]
                follower.move_to_joints(full_command, time_ms=30)
                
                last_known_joints = target_joints  # Update last known joints
                
            except zmq.Again: 
                continue  # No message received, continue loop
            except KeyboardInterrupt: 
                break  # Exit on Ctrl+C
            except Exception as e:
                # Log and ignore other exceptions to keep running
                continue

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Clean up resources
        if follower: follower.close()
        socket.close()
        context.term()

if __name__ == "__main__":
    run_follower_node()