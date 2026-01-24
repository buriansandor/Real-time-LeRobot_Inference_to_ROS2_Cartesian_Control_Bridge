#!/usr/bin/env python3
"""
FOLLOWER NODE (Subscriber) - UNLOCKED WORKSPACE
-----------------------------------------------
Fixes:
- Removed X-axis safety wall (safe_x = -0.20).
- Kept Z-offset to handle height difference.
"""
import sys
import time
import zmq
import numpy as np
import os
import traceback
from pathlib import Path

# --- 📝 LOGGER ---
class DualLogger(object):
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

sys.stdout = DualLogger()
sys.stderr = sys.stdout

# --- 🔧 CONFIGURATION ---
SWAP_XY = False     
MIRROR_X = False   
MIRROR_Y = False   
MIRROR_Z = False   

Z_OFFSET = -0.05     

# --- PATHS ---
current_script_dir = Path(__file__).parent.resolve()
package_dir = current_script_dir.parent
drivers_root = package_dir / "drivers" / "SO100_Robot"
core_dir = drivers_root / "so100_core"
utils_dir = package_dir / "utils"

paths_to_add = [drivers_root, core_dir, utils_dir]
for p in paths_to_add:
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

try:
    import torch
    import torch.nn as nn
    from so100_driver import SO100Robot as FollowerRobot
    from kinematics import SO100Kinematics
    from input_utils import get_port_input
except ImportError as e:
    print(f"❌ IMPORT ERROR: {e}")
    sys.exit(1)

# ================= POLICY =================
class RobustPolicy(nn.Module):
    def __init__(self, urdf_path):
        super().__init__()
        print(f"⚙️ Kinematics loaded: {urdf_path}")
        self.kinematics = SO100Kinematics(urdf_path)
        self.chain_length = len(self.kinematics.chain.links)
        
        # SAFETY LIMITS
        self.safe_z = 0.01 
        # UNLOCKED X: Allow robot to reach back/center
        self.safe_x = -0.20 
        
        self.natural_pose = [0.0, -0.5, 1.0, -0.5, 0.0]

    def check_posture_health(self, joints):
        if len(joints) < 4: return True
        elbow = joints[2]
        if elbow < -1.5: return False
        return True

    def forward(self, leader_joints, leader_gripper, current_follower_joints):
        arm_joints = leader_joints[:5]
        target_xyz = self.kinematics.forward_kinematics(arm_joints)
        x, y, z = target_xyz
        
        print(f"[RAW] X={x:.2f} Y={y:.2f} Z={z:.2f} | ", end="")

        if SWAP_XY: x, y = y, x
        if MIRROR_X: x = -x
        if MIRROR_Y: y = -y
        if MIRROR_Z: z = -z 
        
        z += Z_OFFSET

        # Apply Safety
        tag = "(Smooth)"
        if z < self.safe_z:
            z = self.safe_z
            tag = "(CLAMP Z)"
        
        if x < self.safe_x: 
            x = self.safe_x
            tag = "(CLAMP X)"
        
        print(f"-> [TGT] X={x:.2f} Y={y:.2f} Z={z:.2f} {tag}", end="")

        seed_state = [0.0] * self.chain_length
        is_healthy = True
        if len(current_follower_joints) == 5:
            is_healthy = self.check_posture_health(current_follower_joints)
        
        if is_healthy and len(current_follower_joints) == 5:
            seed_state[1:6] = current_follower_joints
            print("") 
        else:
            print(" [RESET POSE]")
            seed_state[1:6] = self.natural_pose
        
        follower_joints_full = self.kinematics.inverse_kinematics(
            target_pos=[x, y, z],
            orientation_mode=None,
            seed_state=seed_state 
        )
        
        action_joints = follower_joints_full[1:6]
        target_grip = 1.0 if leader_gripper > 0.5 else 0.0
        return list(action_joints), target_grip

# ================= MAIN =================
def run_follower_node():
    print("\n🧠 --- ZMQ FOLLOWER NODE (WORKSPACE UNLOCKED) ---")
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.connect("tcp://localhost:5555") 
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    
    print("--- FOLLOWER SETUP ---")
    port = get_port_input("COM4")
    config_dir = drivers_root / "config"
    urdf_path = config_dir / "so100.urdf"
    
    follower = None
    try:
        follower = FollowerRobot(port=port, config_dir=str(config_dir))
        follower.torque_enable(True)
        policy = RobustPolicy(str(urdf_path))
    except Exception as e:
        print(f"❌ Error: {e}")
        return

    print("🚀 STARTED! (Ctrl+C to stop)")
    last_known_joints = policy.natural_pose 

    try:
        while True:
            try:
                msg = socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.005) 
                continue
            except KeyboardInterrupt:
                raise 
            
            try:
                target_joints, target_grip = policy.forward(
                    msg['joints'], 
                    msg['gripper'], 
                    last_known_joints
                )
                
                if target_grip > 0.5: follower.gripper_open()
                else: follower.gripper_close()

                full_command = target_joints + [target_grip]
                follower.move_to_joints(full_command, time_ms=30)
                
                last_known_joints = target_joints
                
            except Exception as e:
                continue

    except KeyboardInterrupt:
        print("\n🛑 STOPPING...")
    finally:
        try:
            if follower: 
                follower.torque_disable()
                follower.close()
        except: pass
        socket.close()
        context.term()
        sys.exit(0)

if __name__ == "__main__":
    run_follower_node()