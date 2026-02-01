#!/usr/bin/env python3
"""
ZMQ HYBRID FOLLOWER NODE
Created by Google Gemini Pro and Sandor Burian based on previous versions.

This script CONTROLS BOTH:
1. The physical SO100 robot (over USB).
2. The remote ROS2 simulation (via HTTP POST).
"""
import sys
import time
import zmq
import json
import requests
import numpy as np
from pathlib import Path
from get_follower_IP import get_follower_ip

# --- CONFIGURATION ---
# 1. Physical robot settings
SWAP_XY = False
MIRROR_X = False
MIRROR_Y = False
MIRROR_Z = False
Z_OFFSET_ROBOT = -0.05  # Height correction for the physical robot

# 2. Simulation (HTTP) settings
REMOTE_API_URL = f"http://{get_follower_ip()}:8000/pose"  # Change if different IP
HTTP_SEND_RATE = 0.05  # 20 Hz (So it doesn't slow down the physical robot)

# --- PATHS ---
current_dir = Path(__file__).parent.resolve()
package_dir = current_dir.parent
drivers_root = package_dir / "drivers" / "SO100_Robot"
core_dir = drivers_root / "so100_core"
utils_dir = package_dir / "utils"

sys.path.insert(0, str(drivers_root))
sys.path.insert(0, str(core_dir))
sys.path.insert(0, str(utils_dir))

try:
    import torch
    import torch.nn as nn
    from so100_driver import SO100Robot as FollowerRobot
    from kinematics import SO100Kinematics
    from input_utils import get_port_input
except ImportError as e:
    print(f"[ERROR] IMPORT ERROR: {e}")
    sys.exit(1)

# ================= POLICY (Math) =================
class RobustPolicy(nn.Module):
    def __init__(self, urdf_path):
        super().__init__()
        self.kinematics = SO100Kinematics(urdf_path)
        self.chain_length = len(self.kinematics.chain.links)
        # Safety limits (for the physical robot)
        self.safe_z = 0.01
        self.safe_x = -0.20
        self.natural_pose = [0.0, -0.5, 1.0, -0.5, 0.0]

    def forward(self, leader_joints, current_follower_joints):
        # 1. FK (Compute where the hand is)
        arm_joints = leader_joints[:5]
        target_xyz = self.kinematics.forward_kinematics(arm_joints)
        x, y, z = target_xyz

        # Save this for HTTP sending in raw form
        raw_xyz = (x, y, z)

        # 2. Transformations (for the physical robot)
        if SWAP_XY: x, y = y, x
        if MIRROR_X: x = -x
        if MIRROR_Y: y = -y
        if MIRROR_Z: z = -z
        z += Z_OFFSET_ROBOT

        # 3. Safety (for the physical robot)
        if z < self.safe_z: z = self.safe_z
        if x < self.safe_x: x = self.safe_x

        # 4. IK (Compute for the physical robot)
        seed_state = [0.0] * self.chain_length
        # Simplified seed logic
        if len(current_follower_joints) == 5:
            seed_state[1:6] = current_follower_joints
        else:
            seed_state[1:6] = self.natural_pose

        follower_joints_full = self.kinematics.inverse_kinematics(
            target_pos=[x, y, z],
            orientation_mode=None,
            seed_state=seed_state
        )

        return list(follower_joints_full[1:6]), raw_xyz

# ================= MAIN =================
def run_hybrid_node():
    print("\n--- ZMQ HYBRID NODE (Local + HTTP) ---")

    # 1. ZMQ
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    # 2. HTTP Session
    session = requests.Session()

    # 3. Physical Robot
    port = get_port_input("COM4")
    config_dir = drivers_root / "config"
    urdf_path = config_dir / "so100.urdf"

    follower = None
    try:
        follower = FollowerRobot(port=port, config_dir=str(config_dir))
        follower.torque_enable(True)
        policy = RobustPolicy(str(urdf_path))
        print("[INFO][OK] Robot connected.")
    except Exception as e:
        print(f"[ERROR] Robot Error: {e}")
        return

    print("[INFO] Starting! Watch the robot and the simulation.")
    last_known_joints = [0.0] * 5
    last_http_time = 0

    try:
        while True:
            try:
                # Non-blocking read
                msg = socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.005)
                continue
            except KeyboardInterrupt:
                break

            try:
                # --- PROCESSING ---
                target_joints, raw_xyz = policy.forward(msg['joints'], last_known_joints)
                target_grip = msg['gripper']

                # --- 1. MOVE PHYSICAL ROBOT ---
                if target_grip > 0.5:
                    follower.gripper_open()
                else:
                    follower.gripper_close()

                full_command = target_joints + [target_grip]
                follower.move_to_joints(full_command, time_ms=30)
                last_known_joints = target_joints

                # --- 2. HTTP SEND (rate-limited) ---
                now = time.time()
                if now - last_http_time > HTTP_SEND_RATE:
                    # Send the raw XYZ to the simulation (you can also send corrected values)
                    # Here we send the Leader's raw position
                    payload = {
                        "x": round(raw_xyz[0], 3),
                        "y": round(raw_xyz[1], 3),
                        "z": round(raw_xyz[2], 3),
                        "units": "m"
                    }
                    try:
                        session.post(REMOTE_API_URL, json=payload, timeout=0.02)
                        # Do not print every successful send to avoid flooding
                    except:
                        pass  # If HTTP fails, do not stop the robot!

                    last_http_time = now

                # Minimal status
                sys.stdout.write(f"\rMotion OK | X={raw_xyz[0]:.2f} Y={raw_xyz[1]:.2f} Z={raw_xyz[2]:.2f}   ")
                sys.stdout.flush()

            except Exception:
                continue

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if follower:
            follower.torque_disable()
            follower.close()
        socket.close()
        context.term()

if __name__ == "__main__":
    run_hybrid_node()