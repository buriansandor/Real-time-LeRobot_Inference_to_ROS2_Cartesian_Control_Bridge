#!/usr/bin/env python3
"""
SO-100 'Follower' Robot Calibration for Cartesian Calculations Transfer Script 

Created by Sandor Burian with the help of Google Gemini Pro.

This script transfers calibration data from a well-calibrated 'Leader' SO-100 robot
to a 'Follower' SO-100 robot by guiding the user through manual alignment
"""

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
import time
import csv
from so100_control_driver import SO100ControlDriver

# Try to import PortFinder, use fallback if not available
try:
    from lerobot_functions import PortFinder
    print("=== SO-100 'SZOBOR' KALIBRÁCIÓS ÁTVITEL ===")
    print("Set the port of leader arm")
    LEADER_PORT = PortFinder.find_port_with_lerobot()
    print("Set the port of the follower arm")
    FOLLOWER_PORT = PortFinder.find_port_with_lerobot() 
except ImportError:
    print("Warning: PortFinder not available, using manual port configuration.\n Please set LEADER_PORT and FOLLOWER_PORT manually, use 'lerobot-find-port' to find them in a separate console.")
    LEADER_PORT = input("Enter the port of the leader arm (e.g., COM5 or /dev/ttyUSB0): ").strip()
    FOLLOWER_PORT = input("Enter the port of the follower arm (e.g., COM6 or /dev/ttyUSB0): ").strip()
    PortFinder = None

# A Leaderhez használt (már létező) kalibrációs fájl
LEADER_CALIB_FILE = 'callibration_data.csv' 
# Ide fogjuk menteni a Follower új értékeit
OUTPUT_FILE = 'follower_calibration.csv'

def main():
    print("=== SO-100 'SZOBOR' KALIBRÁCIÓS ÁTVITEL ===")
    print("Ez a script átmásolja a Leader pontosságát a Followerra.")
    
    # 1. LEADER ELŐKÉSZÍTÉSE (Referencia)
    print(f"\n1. Csatlakozás a Mesterhez (Leader: {LEADER_PORT})...")
    try:
        # A Leadert a saját jó kalibrációjával töltjük be
        leader = SO100ControlDriver(port=LEADER_PORT, calibration_file=LEADER_CALIB_FILE, simulation=False)
        
        print("   Mester beállítása 'Gyertya' (Függőleges) pozícióba...")
        # Beállítjuk a motorokat a szoftveres 0 pontra (ami a fizikai függőleges)
        # Feltételezzük, hogy a driver már tud írni (set_target_angle)
        # Ha a driveredben nincs set_target_angle, akkor ezt a lépést a 'main_viz.py'-vel is megcsinálhatod előtte.
        
        # Ideiglenes megoldás, ha a driver még nem tud írni:
        # A script itt megáll, és megkér, hogy a Leadert ne mozgasd.
        print("   TARTOM a pozíciót (Torque ON).")
        
        # Ha a drivered már tud írni, akkor itt küldd el a 0 pozíciót:
        for i in range(6):
            # 0.0 radián = URDF szerinti függőleges
            leader.set_target_angle(i, 0.0, move_time_ms=2000)
        time.sleep(2.5) # Várunk, hogy odaérjen
        
    except Exception as e:
        print(f"HIBA a Leaderrel: {e}")
        print("Tipp: Ellenőrizd a portot és hogy a 'callibration_data.csv' létezik-e.")
        return

    # 2. FOLLOWER ELŐKÉSZÍTÉSE (Tanuló)
    print(f"\n2. Csatlakozás a Tanulóhoz (Follower: {FOLLOWER_PORT})...")
    try:
        # Kalibráció nélkül töltjük be (hogy a nyers értékeket lássuk)
        follower = SO100ControlDriver(port=FOLLOWER_PORT, simulation=False)
        
        print("   Tanuló lazítása (Torque OFF)...")
        # Kikapcsoljuk a motorokat, hogy kézzel tudd mozgatni
        for i in range(1, 7):
            follower._write_packet(i, 0x03, [0x28, 0x00]) # 0x28=Torque Enable, 0=OFF
            
    except Exception as e:
        print(f"HIBA a Followerrel: {e}")
        return

    # 3. AZ EMBERI BEAVATKOZÁS
    print("\n" + "="*60)
    print("                           UTASÍTÁS")
    print("="*60)
    print("A Leader robot most tartja a tökéletes függőleges pozíciót.")
    print("A Follower robot motorjai lazák.")
    print("\nFELADAT:")
    print("1. Fogd meg a Follower robotot.")
    print("2. Állítsd be a karjait úgy, hogy PONTOSAN PÁRHUZAMOSAK legyenek a Leaderrel.")
    print("   (Lásd a 'Mester' és a 'Tanítvány' viszonyát).")
    print("3. Figyelj, hogy a csuklók iránya is egyezzen!")
    print("="*60)
    
    input("\nHa a Follower is tökéletesen áll, nyomj ENTER-t a mentéshez...")

    # 4. ÉRTÉKEK MENTÉSE
    print("\nAdatok olvasása a Followerről...")
    offsets = []
    
    # Kiolvassuk, hogy a Follower motorjai mit mutatnak ebben a pozícióban
    # Ez lesz az ő "Null pontja"
    for i in range(1, 7):
        raw = follower.read_raw_position(i)
        if raw is None:
            print(f"HIBA: Nem sikerült olvasni a {i}. motort!")
            return
        offsets.append(raw)
        print(f"   Motor {i} Nyers értéke: {raw}")

    # Fájl írása
    print(f"\nMentés ide: {OUTPUT_FILE}...")
    with open(OUTPUT_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        # Ezek lesznek az új offsetek
        writer.writerow(['ZERO_OFFSETS'] + offsets)
        
        # Az irányok és korrekciók általában hardver-specifikusak (SO-100 standard)
        # Ha a Leadernek más értékei vannak, azokat érdemes átmásolni, de általában ez a standard:
        writer.writerow(['DIRECTIONS', 1, 1, 1, -1, 1, 1]) 
        writer.writerow(['CALIBRATION_POSE_ADJUSTMENTS', 0, 0, 0, 0, 0, 0])

    print("\nSIKERES KALIBRÁCIÓ!")
    print("Most már használhatod a 'follower_calibration.csv'-t a Follower robothoz.")
    
    # 5. Lazítás (Takarítás)
    print("Motorok elengedése...")
    for i in range(1, 7):
        leader._write_packet(i, 0x03, [0x28, 0x00])

if __name__ == "__main__":
    main()