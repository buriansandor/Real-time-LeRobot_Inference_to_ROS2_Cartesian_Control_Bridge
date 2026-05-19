#!/usr/bin/env python3
"""
SPATIAL ACCURACY (KINEMATIC ERROR) MEASUREMENT SCRIPT - TERMINATOR EDITION
Deep search throughout the entire project directory for files.
"""

import sys
import zmq
import time
import csv
import math
from pathlib import Path
import torch.nn as nn
import importlib.util
import os

# --- 1. PHYSICAL ROBOT SETTINGS ---
SWAP_XY = False
MIRROR_X = False
MIRROR_Y = False
MIRROR_Z = False
Z_OFFSET_ROBOT = -0.05

ZMQ_PORT = 5555
DURATION_SECONDS = 30
benchmark_dir = "benchmark_results/similarinsametime"
file_count = len([f for f in os.listdir(benchmark_dir) if os.path.isfile(os.path.join(benchmark_dir, f))]) if os.path.exists(benchmark_dir) else 0
OUTPUT_FILE = f"benchmark_results/similarinsametime/kinematic_error_log{file_count}.csv"

# --- 2. DEEP SEARCH FOR FILES ---
root_dir = Path(__file__).parent.resolve()

print("Searching for 'kinematics.py' and 'so100.urdf' files on your system...")
kinematics_path = None
urdf_path = None

# Sweep through the entire directory and subdirectories
for file_path in root_dir.rglob("kinematics.py"):
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
            if "SO100Kinematics" in content:
                kinematics_path = file_path
                break
    except Exception:
        pass

for file_path in root_dir.rglob("so100.urdf"):
    urdf_path = file_path
    break

if not kinematics_path:
    print(f"\n[CRITICAL ERROR] Cannot find 'kinematics.py' file in {root_dir} directory!")
    print("Either the file is named differently or it is not downloaded to the repository.")
    sys.exit(1)

if not urdf_path:
    print(f"\n[CRITICAL ERROR] Cannot find 'so100.urdf' file in {root_dir} directory!")
    sys.exit(1)

print(f"Kinematics found: {kinematics_path}")
print(f"URDF found:       {urdf_path}\n")

# --- 3. FORCE IMPORT ---
spec = importlib.util.spec_from_file_location("custom_kinematics", str(kinematics_path))
kin_module = importlib.util.module_from_spec(spec)
sys.modules["custom_kinematics"] = kin_module
spec.loader.exec_module(kin_module)

SO100Kinematics = kin_module.SO100Kinematics


# --- 4. YOUR POLICY CLASS ---
class RobustPolicy(nn.Module):
    def __init__(self, urdf_path):
        super().__init__()
        self.kinematics = SO100Kinematics(urdf_path)
        self.chain_length = len(self.kinematics.chain.links)
        self.safe_z = 0.01
        self.safe_x = -0.20
        self.natural_pose = [0.0, -0.5, 1.0, -0.5, 0.0]

    def forward(self, leader_joints, current_follower_joints):
        arm_joints = leader_joints[:5]
        target_xyz = self.kinematics.forward_kinematics(arm_joints)
        x, y, z = target_xyz
        raw_xyz = (x, y, z)

        if SWAP_XY: x, y = y, x
        if MIRROR_X: x = -x
        if MIRROR_Y: y = -y
        if MIRROR_Z: z = -z
        z += Z_OFFSET_ROBOT

        if z < self.safe_z: z = self.safe_z
        if x < self.safe_x: x = self.safe_x

        seed_state = [0.0] * self.chain_length
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


# --- 5. MEASUREMENT CYCLE ---
def calculate_3d_error(target_xyz, actual_xyz):
    dx = target_xyz[0] - actual_xyz[0]
    dy = target_xyz[1] - actual_xyz[1]
    dz = target_xyz[2] - actual_xyz[2]
    return math.sqrt(dx**2 + dy**2 + dz**2) * 1000.0

def run_kinematic_benchmark():
    print("--- SPATIAL ACCURACY (KINEMATIC ERROR) BENCHMARK ---")
    
    print("Initializing RobustPolicy...")
    policy = RobustPolicy(str(urdf_path))

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.connect(f"tcp://127.0.0.1:{ZMQ_PORT}")
    
    results = []
    last_known_joints = [0.0, -0.5, 1.0, -0.5, 0.0]
    
    print("Waiting for ZMQ data (Start the Leader!)...")
    socket.recv_string()
    print(f"Data stream active! Starting measurement for {DURATION_SECONDS} seconds...\n")
    
    start_time = time.perf_counter()
    msg_count = 0
    
    try:
        while time.perf_counter() - start_time < DURATION_SECONDS:
            try:
                message = socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.005)
                continue
                
            leader_joints = message.get("joints", [])
            
            # Run policy
            target_joints, raw_xyz = policy.forward(leader_joints, last_known_joints)
            last_known_joints = target_joints
            
            # Reconstruct target point (what the IK hunts for)
            tx, ty, tz = raw_xyz
            if SWAP_XY: tx, ty = ty, tx
            if MIRROR_X: tx = -tx
            if MIRROR_Y: ty = -ty
            if MIRROR_Z: tz = -tz
            tz += Z_OFFSET_ROBOT
            
            if tz < policy.safe_z: tz = policy.safe_z
            if tx < policy.safe_x: tx = policy.safe_x
            target_xyz = (tx, ty, tz)
            
            # Actual position from IK result
            actual_xyz = policy.kinematics.forward_kinematics(target_joints)
            
            # Error
            error_mm = calculate_3d_error(target_xyz, actual_xyz)
            
            results.append({
                "id": msg_count,
                "target_x": target_xyz[0], "target_y": target_xyz[1], "target_z": target_xyz[2],
                "actual_x": actual_xyz[0], "actual_y": actual_xyz[1], "actual_z": actual_xyz[2],
                "error_mm": error_mm
            })
            
            msg_count += 1
            if msg_count % 30 == 0:
                print(f"[{msg_count}] Target Z: {target_xyz[2]:.3f}m | Actual Z: {actual_xyz[2]:.3f}m | IK Error: {error_mm:.2f} mm")
    except KeyboardInterrupt:
        print("\nMeasurement interrupted!")
    
    print(f"\nSaving data: {OUTPUT_FILE}")
    with open(OUTPUT_FILE, 'w', newline='') as csvfile:
        fieldnames = ['id', 'target_x', 'target_y', 'target_z', 'actual_x', 'actual_y', 'actual_z', 'error_mm']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
        
    if len(results) > 0:
        avg_error = sum(r['error_mm'] for r in results) / len(results)
        max_error = max(r['error_mm'] for r in results)
        
        print("\nKINEMATIC DIAGNOSIS:")
        print(f"Average IK drift: {avg_error:.2f} mm")
        print(f"Maximum IK error:    {max_error:.2f} mm")

if __name__ == "__main__":
    run_kinematic_benchmark()
