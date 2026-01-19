import sys
import os
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from demo.SO100.so100_control_driver import SO100ControlDriver

def tune_wrist():
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
    
    print("--- CSUKLÓ FINOMHANGOLÓ ---")
    
    driver = SO100ControlDriver(port=PORT, urdf_path=URDF_PATH, calibration_file=CALIB_FILE, simulation=False)

    # 1. Beállunk a teszt pozícióba (kinyúlás)
    print("\nMozgás a teszt pozícióba (0.2, 0.0, 0.1)...")
    # Kiszámoljuk a szögeket manuálisan, hogy ne kelljen az IK-val bajlódni itt
    # Ezeket a logodból vettem, amik működtek:
    driver.set_target_angle(0, 0.22, 1000)   # J1
    driver.set_target_angle(1, 1.38, 1000)   # J2
    driver.set_target_angle(2, 1.06, 1000)   # J3
    # A csuklót (J4) a jelenlegi beállítással küldjük
    target_wrist = -2.32
    driver.set_target_angle(3, target_wrist, 1000) 
    
    time.sleep(2.0)
    
    adjustment = 0.0
    print("\n--- HANGOLÁS ---")
    print("Használd a billentyűket a csukló forgatásához:")
    print(" [W] Felfelé emel (korrigál)")
    print(" [S] Lefelé dönt (korrigál)")
    print(" [Q] MENTÉS és KILÉPÉS (Kiírja a kódot)")
    
    while True:
        cmd = input(f"Jelenlegi korrekció: {adjustment:.2f} | Parancs (w/s/q): ").lower()
        
        if cmd == 'q':
            break
        elif cmd == 'w':
            adjustment += 0.1  # Kicsit emelünk rajta
        elif cmd == 's':
            adjustment -= 0.1  # Kicsit döntünk rajta
        else:
            print("Ismeretlen gomb. Csak w, s, q.")
            continue
            
        # Újraszámoljuk és küldjük a parancsot a korrekcióval
        # A driver logikáját imitáljuk: (szög - adjustment)
        # Így látjuk, mi történik
        final_angle = target_wrist - adjustment
        driver.set_target_angle(3, final_angle, move_time_ms=200)
        
    print("\n" + "="*40)
    print("A ROBOT BEÁLLÍTVA!")
    print("Most nyisd meg a 'follower_calibration.csv'-t,")
    print("és a 'CALIBRATION_POSE_ADJUSTMENTS' sort írd át erre:")
    print(f"CALIBRATION_POSE_ADJUSTMENTS,0,0,0,{adjustment:.2f},0,0")
    print("="*40)

    driver.close()

if __name__ == "__main__":
    tune_wrist()