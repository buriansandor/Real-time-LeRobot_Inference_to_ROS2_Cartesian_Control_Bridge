#!/usr/bin/env python3

"""
SO100 Robot Driver Demo
Created by Sandor Burian with the help of Google Gemini Pro.

This script demonstrates simple movements of the SO100 robotic arm using the SO100ControlDriver.
"""

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from demo.SO100.so100_control_driver import SO100ControlDriver
import time
import math


def wavetest():
    print("--- 1. Test: wave ---")
    driver.set_target_angle(0, 0.5, 1000) 
    time.sleep(1.5)
    driver.set_target_angle(0, -0.5, 1000)
    time.sleep(1.5)
    driver.set_target_angle(0, 0.0, 1000)
    time.sleep(1.5)

def shouldertest():
    print("--- 2. Test: Nod (Shoulder) ---")
    # Slightly forward
    driver.set_target_angle(1, 0.2, 1000)
    time.sleep(1.5)
    # Back to Raptor/Middle position
    driver.set_target_angle(1, 0.0, 1000)
    time.sleep(1.0)

    print("Test finished. Relaxing motors...")
    for i in range(1, 7):
        driver._write_packet(i, 0x03, [0x28, 0x00])

def basicdemo():
    print("Movement 1: Left")
    # Move the 1st motor (index 0) to 0.5 radians (approx 30 degrees) in 1 second
    driver.set_target_angle(motor_index=0, angle_radians=0.5, move_time_ms=1000)
    time.sleep(1.5)

    print("Movement 2: Right")
    # Move to -0.5 radians
    driver.set_target_angle(motor_index=0, angle_radians=-0.5, move_time_ms=1000)
    time.sleep(1.5)

    print("Finished. Relaxing.")
    # Turn off torque (Torque Off - Reg 40 = 0)
    for i in range(1, 7):
        driver._write_packet(i, 0x03, [0x28, 0x00])

def candle():
    move_time = 2000 # 2000 ms = 2 seconds (slow, safe)
    
    for i in range(6): # Motor 0..5
        print(f" -> Motor {i+1} set to 0.0...")
        driver.set_target_angle(i, 0.0, move_time_ms=move_time)
        
    print("\nWaiting for the movement to complete...")
    time.sleep(2.5)
    
    print("\nNow the arm should be vertical.")
    print("If tilted, the offset values in 'follower_calibration.csv' need adjustment!")

def candletest():
    print("--- 4. Test: Candle (Vertical) ---")
    print("1. Step: TORQUE ON...")
    
    # Safety wake-up for all motors
    for i in range(1, 7):
        # 0x28 (40) = Torque Enable regiszter, 1 = ON
        driver._write_packet(i, 0x03, [0x28, 0x01])
        time.sleep(0.05)
        
    print(" -> Motors activated")
    time.sleep(1.0)
    
    print("2. Step: Move to Candle (SLOWLY)...")
    
    # 5 seconds for safety
    move_time = 5000 
    
    for i in range(6): 
        # The driver indexes from 0, motor IDs start from 1 (handled by the driver)
        target_raw = driver.ZERO_OFFSETS[i]
        print(f" Motor {i+1} starting -> Target: {target_raw}")
        
        driver.set_target_angle(i, 0.0, move_time_ms=move_time)
        time.sleep(0.2)
        
    print("\nWaiting for the movement to complete...")
    time.sleep(5.5)
    
    print("\nNow the arm should be vertical.")
    print("(If it still doesn't stand up, there is a mechanical jam, but no software issue.)")

def limittest():
    for i in range(1, 7):
        print(f"\n[Motor {i}] Inspection...")
        
        # 1. LOCK CHECK - Reg 48 (55)
        # 0 = Open, 1 = Closed
        lock_val = driver.read_register(i, 48, length=1)
        print(f"   -> Current Lock state (Reg 48): {lock_val}")
        
        if lock_val == 1:
            print("   -> LOCKED! Unlocking...")
            driver._write_packet(i, 0x03, [48, 0]) # Lock = 0
            time.sleep(0.05)
            # Re-read to confirm
            if driver.read_register(i, 48, length=1) == 0:
                print("   -> Success! Lock unlocked.")
            else:
                print("   -> ERROR: Could not unlock the lock!")

        # 2. READ LIMITS
        old_min = driver.read_register(i, 9)
        old_max = driver.read_register(i, 11)
        print(f"   -> Current Limits: Min={old_min}, Max={old_max}")
        
        # 3. OVERRIDE (If necessary)
        if old_min != 0 or old_max < 4000:
            print("   -> Expanding limits (0 - 4095)...")
            
            # Min -> 0
            driver._write_packet(i, 0x03, [9, 0, 0])
            time.sleep(0.02)
            # Max -> 4095 (0x0FFF)
            driver._write_packet(i, 0x03, [11, 0xFF, 0x0F])
            time.sleep(0.02)
            
            # 4. RE-READ (THE MOMENT OF TRUTH)
            new_min = driver.read_register(i, 9)
            new_max = driver.read_register(i, 11)
            
            if new_min == 0 and new_max > 4000:
                print(f"   -> SUCCESS! New limits applied: {new_min} - {new_max}")
            else:
                print(f"   -> FAILURE! The motor did not remember: {new_min} - {new_max}")
        else:
            print("   -> Limits are already open.")
    print("\n------------------------------------------------")
    print("If everywhere was 'SUCCESS' or 'already open', then:")
    print("Run the 'simple_move_test.py' -> Candle test (option 4)!")

def raptor():
    print("--- Move to RAPTOR (Parking) position ---")
    
    # These are your old "Raptor" values:
    # (Waist, Shoulder, Elbow, Wrist1, Wrist2, Gripper)
    positions = [2110, 1105, 2857, 2508, 1997, 1854]
    move_time = 3000 
    
    # 1. Safety "Muscle" activation (Torque ON)
    for i in range(1, 7):
        driver._write_packet(i, 0x03, [0x28, 0x01])
    time.sleep(0.5)

    # 2. Sending raw values
    for i in range(6): 
        motor_id = i + 1
        target_raw = positions[i]
        
        print(f" Motor {motor_id} -> Raptor Target: {target_raw}")
        
        # --- PACKET ASSEMBLY (Bypassing Math) ---
        # Position (Little Endian)
        p_low = target_raw & 0xFF
        p_high = (target_raw >> 8) & 0xFF
        
        # Time (Little Endian)
        t_low = move_time & 0xFF
        t_high = (move_time >> 8) & 0xFF
        
        # [PosL, PosH, TimeL, TimeH, SpdL, SpdH]
        # 0x2A is the Goal Position address
        params = [0x2A, p_low, p_high, t_low, t_high, 0x00, 0x00]
        
        driver._write_packet(motor_id, 0x03, params)
        time.sleep(0.1) # Small pause between packets
        
    print("Commands sent. The robot is now moving to the Raptor pose.")
    time.sleep(3.5)


# Initialize the driver

port_id = input("Enter the COM port ID (e.g., COM4 or /dev/ttyUSB0) (If you are not sure, use the 'lerobot-find-port'): ").strip()
if port_id == "":
    port_id = "COM4"  # Default port

calibration_file = 'follower_calibration.csv'
driver = SO100ControlDriver(port=port_id, urdf_path="URDF/so100.urdf", simulation=False,  calibration_file=calibration_file)

print("\n\nCurrent Calibration Parameters:")
driver.show_current_calibration()
print("\n\n")

print("Starting...")
time.sleep(2)

exiting = False
while not exiting:
    menu = input("Press 1 to start the basic demo movements \nPress 2 to wave test\nPress 3 to shoulder test\nPress 4 to candle test\n---------------------\nPress 5 to limit test\nPress 6 to raptor test\n\nPress x to exit: \n")
    if menu == "1":
        basicdemo()
    elif menu == "2":
        wavetest()
    elif menu == "3":
        shouldertest()
    elif menu == "4":
        candletest()
    elif menu == "5":
        limittest()
    elif menu == "6":
        raptor()
    elif menu.lower() == "x":
        exiting = True
        print("Exiting...")
    else:
        print("Invalid option.")

choice = input("\nDo you want to release the motors (Relax)? (y/n): ")
if choice.lower() == 'y' or choice == '':
    print("Releasing motors...")
    for i in range(1, 7):
        driver._write_packet(i, 0x03, [0x28, 0x00])