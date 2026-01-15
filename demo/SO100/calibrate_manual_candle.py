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

def main():
    FOLLOWER_PORT = input("Enter the port of the follower arm (e.g., COM4 or /dev/ttyUSB0): ").strip()
    if FOLLOWER_PORT == '':
        print("Setting port to the default: COM4")
        FOLLOWER_PORT = 'COM4'  # Change this to your follower's port if needed
    OUTPUT_FILE = input("Enter the output file name (default: follower_calibration.csv): ").strip()
    if OUTPUT_FILE == '':
        print("Setting output file to default: follower_calibration.csv")
        OUTPUT_FILE = 'follower_calibration.csv'


    print("=== MANUAL CANDLE CALIBRATION OF SO100 FOLLOWER ARM ===")
    print("This script will help you calibrate the Follower SO-100 robot, based on your setting it to the perfect 'Candle' (vertical) position.")
    
    print(f"\nConnecting to the Follower ({FOLLOWER_PORT})...")
    # Without calibration (raw mode)
    follower = SO100ControlDriver(port=FOLLOWER_PORT, simulation=False)
    
    # 2. Relaxation (Torque OFF)
    print("Relaxing motors (Torque OFF)...")
    for i in range(1, 7):
        follower._write_packet(i, 0x03, [0x28, 0x00])

    # 3. INSTRUCTIONS
    print("\n" + "="*60)
    print("                           TASK")
    print("="*60)
    print("1. Grab the Follower robot.")
    print("2. Manually set it to the perfect 'CANDLE' position.")
    print("   -> Every arm should point STRAIGHT UP.")
    print("   -> The wrist should be straight.")
    print("   -> The waist should be centered.")
    print("3. Hold this position (support if necessary).")
    print("="*60)
    
    input("When the robot is in the PERFECT CANDLE position, press ENTER...")

    # 4. Saving values
    print("\nReading current (Candle) values...")
    candle_offsets = []
    
    for i in range(1, 7):
        # Reading values as the current positiion is the candle (0.0 radians),
        # The raw value will be the OFFSET!
        # Raw = ((0 - 0) / ...) + Offset  => Raw = Offset
        raw = follower.read_raw_position(i)
        
        if raw is None:
            print(f"ERROR: Failed to read Motor {i}!")
            return
            
        candle_offsets.append(raw)
        print(f"   Motor {i} (Candle): {raw}")

    # Writing file
    print(f"\nSaving to: {OUTPUT_FILE}...")
    with open(OUTPUT_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['ZERO_OFFSETS'] + candle_offsets)
        # Standard directions (if something moves reversed on your robot, change it to -1 here)
        writer.writerow(['DIRECTIONS', 1, 1, 1, -1, 1, 1]) 
        writer.writerow(['CALIBRATION_POSE_ADJUSTMENTS', 0, 0, 0, 0, 0, 0])

    print("\nDONE! Calibration saved.")
    print("Now run 'simple_move_test.py' (Candle test)!")
    print("If you ask for 0.0 now, it will return EXACTLY here.")

if __name__ == "__main__":
    main()