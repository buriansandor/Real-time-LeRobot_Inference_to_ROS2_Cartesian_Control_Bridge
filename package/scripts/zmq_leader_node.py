#!/usr/bin/env python3
"""
LEADER NODE (Publisher) + LOGGING

Created by Google Gemini Pro based on previous versions.

Reads from the Leader robot -> Publishes via ZMQ.
Logs everything to 'leader_log.txt'.
"""
import sys
import time
import zmq
import json
import numpy as np
import os
from pathlib import Path

# --- LOGGER CLASS ---
class DualLogger(object):
    def __init__(self, filename="leader_log.txt"):
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

# --- IMPORTS ---
try:
    try:
        from leader_robot import SO100LeaderToCartesianControl as SO100Leader
    except ImportError:
        from leader_robot import SO100Leader
    from input_utils import get_port_input
except ImportError as e:
    print(f"[ERROR] IMPORT ERROR: {e}")
    sys.exit(1)

def run_leader_node():
    print("\n--- ZMQ LEADER NODE (PUBLISHER) ---")
    
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    try:
        socket.bind("tcp://*:5555")
        print("Topic published: tcp://localhost:5555")
    except zmq.ZMQError as e:
        print(f"[ERROR] Error opening port: {e}")
        return

    print("--- LEADER SETUP ---")
    port = get_port_input("COM5")
    
    try:
        config_dir = drivers_root / "config"
        leader = SO100Leader(port=port, config_dir=str(config_dir))
        leader.torque_disable()
    except Exception as e:
        print(f"[ERROR] Error starting Leader: {e}")
        return

    print("[INFO] BROADCAST STARTED! (Ctrl+C to stop)")
    
    try:
        while True:
            joints = leader.get_joint_angles()
            _, gripper = leader.get_cartesian_pose()
            
            msg = {
                "joints": joints[:5],
                "gripper": gripper,
                "timestamp": time.time()
            }
            
            socket.send_json(msg)
            time.sleep(0.033)
            
            sys.stdout.write(f"\rSending: {msg['joints'][0]:.2f}, Grip: {msg['gripper']:.2f}   ")
            sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n[INFO] Broadcast ended.")
    finally:
        try: leader.close()
        except: pass
        socket.close()
        context.term()

if __name__ == "__main__":
    run_leader_node()