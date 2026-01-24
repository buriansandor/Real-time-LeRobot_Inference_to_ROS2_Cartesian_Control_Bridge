#!/usr/bin/env python3
"""
SO100 LeRobot-Style Policy Demo (LOGGING + FIXED MOVEMENT)
----------------------------------------------------------
Ez a script bemutatja, hogyan néz ki a logika "Policy" formában.
A Policy nem nyúl a hardverhez, csak matematikát végez.

VÁLTOZÁSOK:
- Automatikus logolás fájlba (teleop_log.txt)
- Javított 6-tengelyes vezérlés (Gripper bugfix)
- Javított import útvonalak
"""

import sys
import os
import time
import numpy as np
from pathlib import Path

# ==============================================================================
# 📝 0. RÉSZ: LOGGER BEÁLLÍTÁS (FÁJLBA MENTÉS)
# ==============================================================================
class DualLogger(object):
    def __init__(self, filename="package/drivers/log/teleop_log.txt"):
        self.terminal = sys.stdout
        self.log = open(filename, "w", encoding="utf-8")
        print(f"📝 [LOG] A kimenet mentése ide is történik: {filename}")

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
        self.log.flush() # Azonnali írás, hogy összeomlásnál is meglegyen

    def flush(self):
        self.terminal.flush()
        self.log.flush()

# Átirányítjuk a print-et a Logger osztályba
sys.stdout = DualLogger()
sys.stderr = sys.stdout # A hibákat is mentjük

# ==============================================================================
# 🛠️ ÚTVONALAK ÉS IMPORTOK
# ==============================================================================

# 1. Megkeressük a script helyét
current_script_dir = Path(__file__).parent.resolve()
package_root = current_script_dir.parent.parent

# 2. Definiáljuk a lehetséges driver mappákat
drivers_root = current_script_dir.parent / "drivers" / "SO100_Robot"
core_dir = drivers_root / "so100_core"
utils_dir = current_script_dir.parent / "utils"

# 3. Hozzáadjuk őket a sys.path-hoz
paths_to_add = [str(drivers_root), str(core_dir), str(utils_dir), str(package_root)]

print("\n🔍 [DEBUG] Keresési útvonalak frissítése...")
for p in paths_to_add:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

# --- DEPENDENCY ELLENŐRZÉS ---
try:
    import torch
    import torch.nn as nn
except ImportError:
    print("\n❌ HIBA: A 'torch' modul hiányzik!")
    print("Kérlek telepítsd: pip install torch numpy")
    sys.exit(1)

# --- IMPORTOK ---
try:
    # Follower Robot
    try:
        from so100_driver import SO100Robot as FollowerRobot
        print("✅ Siker: Follower driver betöltve (so100_driver)")
    except ImportError:
        from robot import SO100Robot as FollowerRobot
        print("✅ Siker: Follower driver betöltve (robot.py)")

    # Leader Robot (Hosszú név kezelése)
    try:
        from leader_robot import SO100LeaderToCartesianControl as SO100Leader
        print("✅ Siker: Leader driver betöltve (SO100LeaderToCartesianControl)")
    except ImportError:
        try:
            from leader_robot import SO100Leader
            print("✅ Siker: Leader driver betöltve (SO100Leader)")
        except ImportError:
            from package.drivers.SO100_Robot.leader_robot import SO100LeaderToCartesianControl as SO100Leader

    # Egyéb
    from kinematics import SO100Kinematics
    from input_utils import get_port_input
    
except ImportError as e:
    print(f"\n❌ IMPORT KRITIKUS HIBA: {e}")
    sys.exit(1)

# ==============================================================================
# 🧠 1. RÉSZ: A POLICY (AZ AGY)
# ==============================================================================

class CartesianTeleopPolicy(nn.Module):
    def __init__(self, urdf_path):
        super().__init__()
        self.kinematics = SO100Kinematics(urdf_path)
        
        # Paraméterek
        self.safe_z = 0.02
        self.safe_x = 0.03
        self.gripper_threshold = 0.5

    def forward(self, observation):
        leader_joints = observation.cpu().numpy()[0]
        
        # FK
        arm_joints = leader_joints[:5]
        target_xyz = self.kinematics.forward_kinematics(arm_joints)
        x, y, z = target_xyz

        # Safety
        if z < self.safe_z: z = self.safe_z
        if x < self.safe_x: x = self.safe_x
        
        # Gripper
        leader_gripper = leader_joints[5]
        target_gripper = 1.0 if leader_gripper > self.gripper_threshold else 0.0

        # IK
        follower_joints = self.kinematics.inverse_kinematics(
            target_pos=[x, y, z],
            orientation_mode=None
        )
        
        action_joints = follower_joints[1:6] 
        full_action = np.append(action_joints, target_gripper)

        return torch.from_numpy(full_action).float().unsqueeze(0)


# ==============================================================================
# 🤖 2. RÉSZ: A KÖRNYEZET (A TEST)
# ==============================================================================

def run_lerobot_session():
    print("\n🧠 --- LeROBOT STYLE POLICY DEMO --- 🧠")
    
    # Portok
    print("--- LEADER SETUP ---")
    leader_port = get_port_input("COM5")
    
    print("\n--- FOLLOWER SETUP ---")
    follower_port = get_port_input("COM4")
    
    # Config keresés
    urdf_path = None
    possible_config_dirs = [
        drivers_root / "config",
        core_dir / "config"
    ]
    
    final_config_dir = None
    
    for d in possible_config_dirs:
        test_urdf = d / "so100.urdf"
        if test_urdf.exists():
            urdf_path = test_urdf
            final_config_dir = d
            print(f"[INIT] URDF found at: {urdf_path}")
            break
            
    if urdf_path is None:
        print("❌ HIBA: Nem találom a so100.urdf fájlt!")
        return

    try:
        # Driverek indítása
        leader = SO100Leader(port=leader_port, config_dir=str(final_config_dir))
        follower = FollowerRobot(port=follower_port, config_dir=str(final_config_dir))
        
        leader.torque_disable()
        follower.torque_enable(True)
        
        # Policy
        policy = CartesianTeleopPolicy(str(urdf_path))
        
        print("\n✅ Rendszer kész. Nyomj ENTER-t az indításhoz!")
        input()
        print("🚀 POLICY FUTTATÁSA... (Ctrl+C leállítás)")

        FREQUENCY = 30
        dt = 1.0 / FREQUENCY

        while True:
            start_time = time.time()

            # 1. OBSZERVÁCIÓ
            leader_angles = leader.get_joint_angles()
            _, gripper_val = leader.get_cartesian_pose()
            
            state_vector = np.array(leader_angles[:5] + [gripper_val], dtype=np.float32)
            observation_tensor = torch.from_numpy(state_vector).unsqueeze(0)

            # 2. POLICY DÖNTÉS
            with torch.no_grad():
                action_tensor = policy(observation_tensor)

            # 3. CSELEKVÉS
            action_np = action_tensor.squeeze(0).cpu().numpy()
            
            # --- JAVÍTÁS: Nem vágjuk le 5 elemre! ---
            # target_joints = action_np[:5] # <-- EZ VOLT A HIBA
            
            target_gripper = action_np[5]

            # 4. VÉGREHAJTÁS
            if target_gripper > 0.5:
                follower.gripper_open()
            else:
                follower.gripper_close()
            
            move_ms = int(dt * 1000)
            
            # --- JAVÍTÁS: A teljes 6 elemű tömböt küldjük ---
            follower.move_to_joints(action_np, time_ms=move_ms)

            # Időzítés
            elapsed = time.time() - start_time
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\nLeállítás...")
    except Exception as e:
        print(f"\n❌ HIBA: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            leader.close()
            follower.close()
        except: pass

if __name__ == "__main__":
    run_lerobot_session()