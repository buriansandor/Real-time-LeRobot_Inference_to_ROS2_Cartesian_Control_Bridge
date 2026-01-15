#!/usr/bin/env python3

"""
SO100 Robot Limits Backup and Unlock Script

Created by Sandor Burian with the help of Google Gemini Pro.

This script backs up the current motor angle limits of the SO100 robot arm
and provides an option to unlock (remove) these limits for unrestricted movement.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
import time
import csv
from so100_control_driver import SO100ControlDriver

def main():
    try:
        from lerobot_functions import PortFinder
        print("Set the port of arm")
        PORT = PortFinder.find_port_with_lerobot()
    except:
        print("Warning: PortFinder not available, using manual port configuration.\n Please set PORT manually, use 'lerobot-find-port' to find it in a separate console.")
        PORT = input("Enter the port of the arm (e.g., COM4 or /dev/ttyUSB0): ").strip()

    BACKUP_FILE = input("Name of the safety backup file (default: 'factory_limits_backup.csv'): ").strip()
    if BACKUP_FILE == '':
        BACKUP_FILE = 'factory_limits_backup.csv'

    action = input("\nChoose action - (1) Backup and Unlock limits, (2) Restore limits from backup [1/2]: ").strip()
    if action == '1':   
        backup_and_unlock_limits(PORT, BACKUP_FILE)
    elif action == '2':
        restore_limits(PORT, BACKUP_FILE)
    else:
        print("Invalid option. Exiting.")

def backup_and_unlock_limits(PORT, BACKUP_FILE):
    print(f"Connecting to: {PORT}...")
    driver = SO100ControlDriver(port=PORT, simulation=False)
    
    print("\n--- 1. PHASE: BACKING UP CURRENT LIMITS ---")
    
    limits_data = []
    success_count = 0
    
    for i in range(1, 7):
        print(f"Reading Motor {i}...", end="")
        
        # Reg 9 = Min Angle Limit
        min_limit = driver.read_register(i, 9)
        # Reg 11 = Max Angle Limit
        max_limit = driver.read_register(i, 11)
        
        if min_limit is not None and max_limit is not None:
            print(f" SUCCESS! Min: {min_limit}, Max: {max_limit}")
            limits_data.append([i, min_limit, max_limit])
            success_count += 1
        else:
            print(" ERROR! Failed to read.")
            
    if success_count < 6:
        print("\nERROR: Failed to read all motors!")
        print("Process ABORTED. No changes made.")
        return

    # Save to CSV
    with open(BACKUP_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Motor_ID', 'Min_Limit', 'Max_Limit'])
        writer.writerows(limits_data)
        
    print(f"\nBackup saved to: {os.path.abspath(BACKUP_FILE)}")
    print("It is now safe to make changes.")
    
    confirm = input("\nDo you want to UNLOCK the limits now? (yes/no): ")
    if confirm.lower() != 'yes':
        print("Exiting without changes.")
        return

    print("\n--- 2. PHASE: UNLOCKING LIMITS ---")
    for i in range(1, 7):
        print(f"Unlocking Motor {i} (0-4095)...")
        # Min -> 0
        driver._write_packet(i, 0x03, [0x09, 0x00, 0x00])
        time.sleep(0.01)
        # Max -> 4095
        driver._write_packet(i, 0x03, [0x0B, 0xFF, 0x0F])
        time.sleep(0.01)
        
    print("\nDONE! The robot is now unrestricted. The old settings have been saved.")

def restore_limits(PORT, BACKUP_FILE):
    print("--- RESTORING LIMITS ---")
    
    try:
        driver = SO100ControlDriver(port=PORT, simulation=False)
        
        with open(BACKUP_FILE, 'r') as f:
            reader = csv.DictReader(f)
            print(f"'{BACKUP_FILE}' betöltése...")
            
            for row in reader:
                motor_id = int(row['Motor_ID'])
                min_lim = int(row['Min_Limit'])
                max_lim = int(row['Max_Limit'])
                
                print(f"Restoring Motor {motor_id} -> Min: {min_lim}, Max: {max_lim}...", end="")
                
                # Writing back Min Limit
                p_min_low = min_lim & 0xFF
                p_min_high = (min_lim >> 8) & 0xFF
                driver._write_packet(motor_id, 0x03, [0x09, p_min_low, p_min_high])
                time.sleep(0.02)

                # Writing back Max Limit
                p_max_low = max_lim & 0xFF
                p_max_high = (max_lim >> 8) & 0xFF
                driver._write_packet(motor_id, 0x03, [0x0B, p_max_low, p_max_high])
                time.sleep(0.02)
                
                print(" OK.")
                
        print("\nRestoration complete! The robot is now 'locked' again.")
        
    except FileNotFoundError:
        print(f"ERROR: Could not find the file {BACKUP_FILE}!")

if __name__ == "__main__":
    main()