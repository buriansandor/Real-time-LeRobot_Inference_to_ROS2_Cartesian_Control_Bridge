#!/usr/bin/env python3
"""
SO-100 to Annin AR4 Connector Launcher demo
Created by Sandor Burian with the help of Google Gemini Pro
This script automatically detects the connected SO-100 robot arm
and starts the visualization and control program.
"""

import sys
import os
import serial.tools.list_ports

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from visualisation.main_viz import main as start_visualization # Importáljuk a fő programot

def find_robot_port():
    return 'COM5'

def check_files():
    """Checking the files"""
    required = ["SO100/URDF/so100.urdf", "SO100/so100_driver.py", "../kinematics/kinematics_bridge.py", "../visualisation/main_viz.py"]
    missing = [f for f in required if not os.path.exists(f)]
    
    if missing:
        print("ERROR: Missing files!")
        for f in missing:
            print(f" - {f}")
        return False
    return True

if __name__ == "__main__":
    print("==========================================")
    print("   SO-100 -> AR4 Control System v1.0    ")
    print("==========================================")
    
    if not check_files():
        input("Press Enter to continue (exit)...")
        sys.exit()
        
    # 1. Find port
    port = find_robot_port()
    
    # Ask user for mode selection
    print("\nChoose mode:")
    print("1. Real hardware (port: {})".format(port))
    print("2. Simulation mode")
    choice = input("Enter your choice (1/2): ").strip()
    
    if choice == "2":
        # Simulation mode
        print("\nStarting simulation mode...")
        import time
        time.sleep(1)
        try:
            start_visualization(simulation=True)
        except KeyboardInterrupt:
            print("\nStopped.")
    elif choice == "1" and port:
        # Hardware mode
        print(f"\nStarting hardware mode with port: {port}")
        import time
        time.sleep(1)
        try:
            start_visualization(port_name=port, simulation=False)
        except KeyboardInterrupt:
            print("\nStopped.")
    else:
        print("Invalid choice or no available port.")
             
    print("Exiting...")