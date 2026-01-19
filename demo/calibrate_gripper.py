import sys
import os
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from demo.SO100.so100_control_driver import SO100ControlDriver

def calibrate_gripper():
    PORT = input("Enter the port of the follower arm (e.g., COM4 or /dev/ttyUSB0) or get the porty with port detection(detect): ").strip()
    if PORT == '':
        print("Setting port to the default: COM4")
        PORT = 'COM4'  # Change this to your follower's port if needed
    elif PORT.lower() == 'detect':
        try:
            from lerobot_functions import PortFinder
            print("Set the port of leader arm")
            PORT = PortFinder.find_port_with_lerobot()
        except ImportError:
            print("Warning: PortFinder not available, using manual port configuration.\n Please set LEADER_PORT and FOLLOWER_PORT manually, use 'lerobot-find-port' to find them in a separate console.")
    CALIB_FILE = input("Enter the calibration file name (default: follower_calibration.csv): ").strip()
    if CALIB_FILE == '':
            print("Setting calibration file to default: follower_calibration.csv")
            CALIB_FILE = 'follower_calibration.csv'
    URDF_PATH = input("Enter the URDF file path (default: URDF/so100.urdf): ").strip()
    if URDF_PATH == '':
        print("Setting URDF path to default: URDF/so100.urdf")
        URDF_PATH = "URDF/so100.urdf"
    
    
    driver = SO100ControlDriver(port=PORT, urdf_path=URDF_PATH, calibration_file=CALIB_FILE, simulation=False)


    print("--- GRIPPER (FOGÓ) KALIBRÁLÓ ---")
    
    # 6-os motor (Index 5)
    GRIPPER_ID = 5 
    
    print("\nNyomaték engedélyezése a fogón...")
    driver.torque_enable(GRIPPER_ID + 1) # Motor ID 6
    
    # Jelenlegi pozíció lekérése
    current_pos = driver.read_raw_position(GRIPPER_ID + 1)
    print(f"Kezdő pozíció: {current_pos}")
    
    print("\n--- KEZELÉS ---")
    print(" [W] NYITÁS (Érték növelése)")
    print(" [S] ZÁRÁS (Érték csökkentése)")
    print(" [Space] Jelenlegi érték mentése listába")
    print(" [Q] KILÉPÉS")
    
    saved_values = {}
    
    while True:
        print(f"\rJelenlegi érték: {current_pos} ", end="")
        
        # Windows-specifikus billentyűolvasás (hogy ne kelljen entert nyomni)
        import msvcrt
        key = msvcrt.getch().decode('utf-8').lower()
        
        step = 20 # Lépésköz
        
        if key == 'q':
            break
        elif key == 'w':
            current_pos += step
        elif key == 's':
            current_pos -= step
        elif key == ' ':
            name = input(f"\nMinek nevezzük el a {current_pos} értéket? (pl. 'open', 'close'): ")
            saved_values[name] = current_pos
            print(f"Mentve: {name} -> {current_pos}")
            continue
            
        # Határok (nehogy túlhajtsuk)
        current_pos = max(0, min(4095, current_pos))
        
        # Küldés nyers parancsként (megkerülve a kinematikát)
        # STS3215 raw position write
        driver._write_packet(GRIPPER_ID + 1, 0x03, [0x2A, 0x00, current_pos & 0xFF, (current_pos >> 8) & 0xFF, 0x00, 0x00, 0x00, 0x00])

    print("\n" + "="*40)
    print("EREDMÉNYEK:")
    for name, val in saved_values.items():
        print(f"{name}: {val}")
    print("="*40)
    
    driver.close()

if __name__ == "__main__":
    calibrate_gripper()