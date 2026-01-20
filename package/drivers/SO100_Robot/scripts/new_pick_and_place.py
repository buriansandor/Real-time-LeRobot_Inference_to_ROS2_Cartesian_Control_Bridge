"""
Slow demo script for pick and place operation using SO-100 Robot.
Created by Sándor Burian with the help of Google Gemini Pro.
"""

import sys
import os
import time
from pathlib import Path

# Add the package root to Python path
script_dir = Path(__file__).parent
package_root = script_dir.parent.parent.parent
sys.path.insert(0, str(package_root))

from drivers.SO100_Robot import SO100Robot

def main():
    print("--- SO-100 DEMO ---")
    
    # 1. Connection (Reads CSVs from the config folder!)
    port = input("Port (e.g., COM4): ")
    robot = SO100Robot(port=port, config_dir="config")
    
    print("Waking up motors...")
    robot.torque_enable(True)
    
    # Coordinates (From your test)
    PICK_POS = [0.25, -0.05, 0.03]
    PLACE_POS = [0.25, 0.05, 0.03]
    SAFE_Z = 0.12
    
    try:
        print("1. Start: Gripper OPEN")
        robot.gripper_open()
        
        print(f"2. Move above the object: {PICK_POS}")
        robot.move_to_cartesian(PICK_POS[0], PICK_POS[1], SAFE_Z)
        
        print("3. Descending...")
        robot.move_to_cartesian(PICK_POS[0], PICK_POS[1], PICK_POS[2])
        
        print("4. GRASP the object")
        robot.gripper_close()
        
        print("5. Lifting")
        robot.move_to_cartesian(PICK_POS[0], PICK_POS[1], SAFE_Z)
        
        print(f"6. Move to placement: {PLACE_POS}")
        robot.move_to_cartesian(PLACE_POS[0], PLACE_POS[1], SAFE_Z)
        
        print("7. Placing")
        robot.move_to_cartesian(PLACE_POS[0], PLACE_POS[1], PLACE_POS[2])
        
        print("8. Release")
        robot.gripper_open()
        
        print("9. Done! Resetting...")
        robot.move_to_cartesian(PLACE_POS[0], PLACE_POS[1], SAFE_Z)
        
    except KeyboardInterrupt:
        print("Stopped!")
    finally:
        robot.close()
        print("Connection closed.")

if __name__ == "__main__":
    main()