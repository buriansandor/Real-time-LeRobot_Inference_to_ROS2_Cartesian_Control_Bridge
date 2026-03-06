#!/usr/bin/env python3
"""
ZMQ LEADER -> HTTP POST BRIDGE

Reads from the Leader robot via ZMQ -> Sends HTTP POST requests to the Follower robot's web server.
"""
import sys
import time
import zmq
import json
import requests
import numpy as np
from pathlib import Path
from get_follower_IP import get_follower_ip


# --- PATHS (To find the kinematics) ---
current_dir = Path(__file__).parent.resolve()
package_dir = current_dir.parent
drivers_root = package_dir / "drivers" / "SO100_Robot"
core_dir = drivers_root / "so100_core"
utils_dir = package_dir / "utils"

sys.path.insert(0, str(drivers_root))
sys.path.insert(0, str(core_dir))
sys.path.insert(0, str(utils_dir))


REMOTE_API_URL = get_follower_ip()

# --- CONFIGURATION (Coordinate Transformations) ---
# 1. AXIS SWAP
SWAP_XY = False     # Swap X and Y axes if mechanical mounting differs

# 2. MIRRORING
MIRROR_X = False   # Mirror X axis if True
MIRROR_Y = False   # Mirror Y axis if True
MIRROR_Z = False   # Mirror Z axis if True

# 3. OFFSET
Z_OFFSET = -0.05   # Height adjustment (table-to-base offset)

# 4. SAFETY
SAFE_Z_MIN = 0.01  # Minimum Z height to prevent collisions

REMOTE_API_URL = "http://" + REMOTE_API_URL + ":8000/pose"

try:
    from kinematics import SO100Kinematics
except ImportError as e:
    print(f"❌ Hiba: Nem találom a kinematikai modult. {e}")
    sys.exit(1)

def run_http_bridge():
    print(f"\n🌐 --- ZMQ -> HTTP BRIDGE ---")
    print(f"📡 Célpont: {REMOTE_API_URL}")
    print(f"🔧 CONFIG: SWAP_XY={SWAP_XY}, MIRROR=[X:{MIRROR_X} Y:{MIRROR_Y} Z:{MIRROR_Z}], Z_OFFSET={Z_OFFSET}")
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)
    
    # Fontos: Ha ugyanazon a gépen fut a Leader, akkor localhost!
    LEADER_ADDRESS = "tcp://localhost:5555"
    print(f"👂 Csatlakozás a Leaderhez: {LEADER_ADDRESS} ...")
    
    socket.connect(LEADER_ADDRESS)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    
    urdf_path = drivers_root / "config" / "so100.urdf"
    try:
        kinematics = SO100Kinematics(str(urdf_path))
        print("✅ Kinematika betöltve.")
    except Exception as e:
        print(f"❌ URDF Hiba: {e}")
        return

    session = requests.Session()
    last_send_time = 0
    SEND_RATE = 0.05 # 20 Hz

    print("🚀 Híd aktív! Várakozás adatra...\n")

    # Állapotjelző változó, hogy ne floodoljuk a konzolt a "Várakozás" üzenettel
    waiting_printed = False 

    try:
        while True:
            try:
                # Adat fogadása
                msg = socket.recv_json(flags=zmq.NOBLOCK)
                
                # Ha kaptunk adatot, reseteljük a várakozás jelzőt
                waiting_printed = False 

                if time.time() - last_send_time < SEND_RATE:
                    continue
                
                leader_joints = msg['joints'][:5]
                
                # 1. FK (Leader) - Compute end-effector position
                xyz = kinematics.forward_kinematics(leader_joints)
                x, y, z = xyz
                
                # 2. SWAP_XY - Apply axis swap if enabled
                if SWAP_XY:
                    x, y = y, x
                
                # 3. MIRROR - Apply axis mirroring
                if MIRROR_X:
                    x = -x
                if MIRROR_Y:
                    y = -y
                if MIRROR_Z:
                    z = -z
                
                # 4. Z_OFFSET - Apply height adjustment
                z += Z_OFFSET
                
                # 5. SAFETY - Enforce minimum Z height
                if z < SAFE_Z_MIN:
                    z = SAFE_Z_MIN

                payload = {
                    "x": round(x, 3),
                    "y": round(y, 3),
                    "z": round(z, 3),
                    "units": "m"
                }
                
                try:
                    response = session.post(REMOTE_API_URL, json=payload, timeout=0.2)
                    
                    status = "✅" if response.status_code == 200 else f"❌ {response.status_code}"
                    # flush=True nagyon fontos, hogy azonnal lásd!
                    print(f"\r{status} Küldve: X={x:.2f} Y={y:.2f} Z={z:.2f} | Válasz: {response.status_code}   ", end="", flush=True)
                    
                    last_send_time = time.time()
                    
                except requests.exceptions.RequestException as e:
                    print(f"\r⚠️ HTTP Hiba: Nem érem el a szervert ({REMOTE_API_URL})", end="", flush=True)

            except zmq.Again:
                # Ha még nem írtuk ki, hogy várunk, akkor most kiírjuk
                if not waiting_printed:
                    print(f"\r⏳ Várakozás a Leader ZMQ jeleire... (Fut a másik ablakban?)", end="", flush=True)
                    waiting_printed = True
                
                time.sleep(0.005)
                continue
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"\n❌ Váratlan hiba: {e}")
                continue

    except KeyboardInterrupt:
        print("\nLeállítás...")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    run_http_bridge()