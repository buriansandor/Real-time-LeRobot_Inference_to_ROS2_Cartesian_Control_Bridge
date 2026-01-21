#!/usr/bin/env python3
"""
SO100 Leader Robot calibration

Created by Sandor Burian, with the help of Copilot (Claude Sonnet 4)
"""

import csv
import sys
import os
import time
from pathlib import Path
import ikpy.chain

# Add the package root to Python path 
script_dir = Path(__file__).parent
package_root = script_dir.parent.parent
sys.path.insert(0, str(package_root))

from drivers.SO100_Robot import SO100Robot
from utils.input_utils import get_port_input

class calibrateSO100Leader:

    def __init__(self):
        pass

    def calibrate_candle(self):
        print("\n\n=== LEADER CANDLE CALIBRATION ===")
        
        port = get_port_input("COM5")
        config_file_path = self.createConfigurationFile()
        if config_file_path is not None:        
            try:
                driver = SO100Robot(port=port, config_dir="config", calibration_file="SO100leader_to_cartesian_calibration.csv")
                
                # Check if we're in simulation mode (connection failed)
                if getattr(driver, 'simulation', False):
                    print("\n[ERROR] Robot connection FAILED!")
                    print(f"[ERROR] Could not connect to port: {port}")
                    print("\n[ERROR] Possible causes:")
                    print("  - Robot is not powered ON")
                    print("  - Another program is using the port")
                    print("  - Wrong port selected")
                    print("  - USB cable disconnected")
                    print("\n[SOLUTION] Please:")
                    print("  1. Make sure robot is powered ON")
                    print("  2. Check USB cable connection") 
                    print("  3. Close any other robot programs")
                    print("  4. Try running 'lerobot-find-port' again")
                    return
                    
                print("Connected, release the motors...")
                driver.torque_enable(False)
                    
            except Exception as e:
                print(f"[Error]: {e}")
                return

            print("\n--- INSTRUCTIONS ---")
            print("1. Grip the robot gently.")
            print("2. Set to PERFECT CANDLE POSITION.")
            print("   (All joints should be straight, vertically upwards, the gripper should also be straight.)")
            print("3. This will be the mathematical [0, 0, 0, 0, 0] point.")
            
            input("\nWhen you are ready and holding the position, press ENTER!")
            
            print("Reading values...")
            offsets = []
            
            # Read 6 motors
            for i in range(1, 7):
                raw = driver._read_raw_position(i)
                if raw is None:
                    print(f"ERROR: Could not read motor {i}!")
                    return
                offsets.append(raw)
                print(f"Motor {i}: {raw}")
                
            print("\nValues recorded!\n")
            
            self.saveCalibrationData(config_file_path, offsets=offsets)
        print("You can now start the Teleoperation script to test the calibration.")
        driver.close()

    def createConfigurationFile(self):
        """Create a default configuration file for the leader robot calibration"""
        
        config_dir = os.path.join(package_root, "package", "drivers", "SO100_Robot", "config")
        # Ensure config directory exists
        os.makedirs(config_dir, exist_ok=True)
        csv_path = os.path.join(config_dir, "SO100leader_to_cartesian_calibration.csv")
        path =self.saveCalibrationData(csv_path, offsets=[0,0,0,0,0,0])
        if path is not None:
            print(f"[DEBUG] Configuration file created with default offsets.")
            return path
        else:
            print(f"[ERROR] Failed to create configuration file!")
        return None
    
    def saveCalibrationData(self, path, offsets, directions=[1,1,1,1,1,1], adjustments=[0,0,0,0,0,0]):
        try:
            # Ensure parent directory exists
            os.makedirs(os.path.dirname(path), exist_ok=True)
            
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['ZERO_OFFSETS'] + offsets)
                writer.writerow(['DIRECTIONS'] + directions)
                writer.writerow(['CALIBRATION_POSE_ADJUSTMENTS'] + adjustments)    
                print(f"\n[DEBUG] Calibration data saved to: {path}")
                return path
        except Exception as e:
            print(f"[ERROR] Failed to save calibration data: {e}")
            return None

if __name__ == "__main__":
    calibrator = calibrateSO100Leader()
    calibrator.calibrate_candle()
