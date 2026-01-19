import sys
import os
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from demo.SO100.so100_control_driver import SO100ControlDriver

def sculpt_robot():
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

    print("\n--- ROBOT SZOBRÁSZ ---")
    print("Most helyretesszük az egész kart!")
    port = input("Add meg a portot (pl. COM4): ")
    
    
    # Cél: Kinyúlás
    target_angles = {
        0: 0.22,  # Forgás
        1: 1.38,  # Váll (J2)
        2: 1.06,  # Könyök (J3)
        3: -2.32, # Csukló (J4)
        4: 0.0,
        5: 0.0
    }
    
    print("\nBeállás a teszt pozícióba (0.20, 0.0, 0.10)...")
    for i, angle in target_angles.items():
        driver.set_target_angle(i, angle, 1500)
    time.sleep(2.0)
    
    # Korrekciók tárolása (Radiánban)
    adjustments = [0.0] * 6
    
    current_motor = 1 # Kezdjük a Vállal (J2 -> index 1)
    
    while True:
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"\n--- ÉLŐ HANGOLÁS ---")
        print(f"Jelenleg kiválasztott motor: {current_motor + 1}. (J{current_motor+1})")
        print(f"Jelenlegi korrekciók: {[round(a, 2) for a in adjustments]}")
        print("-" * 30)
        print(" [1, 2, 3, 4]  -> Motor kiválasztása (Váll, Könyök, Csukló...)")
        print(" [W] / [S]     -> Mozgatás (+ / - irány)")
        print(" [Q]           -> MENTÉS és KILÉPÉS")
        print("-" * 30)
        
        key = input("Parancs: ").lower()
        
        if key == 'q':
            break
        elif key in ['1', '2', '3', '4']:
            # Motor váltás (input 1-based, index 0-based)
            # De a J1 (index 0) a forgás, az jó volt. Mi a J2, J3, J4-el foglalkozunk.
            idx = int(key) - 1
            current_motor = idx
            print(f"Átváltva a {key}. motorra.")
            continue
            
        step = 0.05 # Finom lépésköz
        
        if key == 'w':
            adjustments[current_motor] += step
        elif key == 's':
            adjustments[current_motor] -= step
        else:
            continue
            
        # Parancs küldése a módosított értékkel
        # A driver logikája: target - adjustment. 
        # Tehát ha mi hozzáadunk az adjustmenthez, a driver levonja a célból -> változik a pozíció.
        base_target = target_angles.get(current_motor, 0.0)
        
        # FIGYELEM: Itt "fordítva" gondolkodunk kicsit.
        # Ha a 'calibration_pose_adjustments'-et állítjuk, az egy eltolás.
        # Most csak simán küldjük a módosított parancsot, hogy lásd a hatást.
        # A végén ezt az 'eltérést' mentjük el korrekciónak.
        
        # Küldés
        # A driver set_target_angle függvénye NEM adja hozzá a korrekciót automatikusan a paraméterhez,
        # hanem a CSV-ből olvassa. Mivel mi most felülbíráljuk a CSV-t manuálisan:
        # Közvetlenül a drivert használjuk? Nem, az bonyolult.
        # Inkább: kiszámoljuk, mi lenne a jó szög, és a különbséget mentjük adjustmentnek.
        
        # Egyszerűsítve:
        # Azt akarjuk, hogy a robot pozíciója változzon.
        # Küldünk egy új targetet: (eredeti_cel + eltolas)
        new_target = base_target + adjustments[current_motor]
        driver.set_target_angle(current_motor, new_target, move_time_ms=100)
        
    print("\n" + "="*40)
    print("A ROBOT BEÁLLÍTVA! Mentsd el ezt a sort:")
    
    # A driver logikája: raw = ((angle - CALIB) / ...)
    # Ha mi azt láttuk jónak, amikor a motor (target + adj) szögben állt,
    # akkor a korrekciós értéknek pont ennek az ellentettjének (vagy magának az értéknek) kell lennie.
    # Ha (Target + X) a jó pozíció, akkor a CALIB értéknek X-nek kell lennie?
    # Képlet: raw = ((Target - Calib) ...).
    # Ha Calib = -X, akkor (Target - (-X)) = Target + X.
    # Tehát: Ha pozitív irányba toltad (W), akkor a Calib értéke NEGATÍV lesz.
    
    final_calib_values = [-1 * a for a in adjustments]
    # Kerekítés és formázás
    calib_str = ",".join([f"{x:.2f}" for x in final_calib_values])
    
    print(f"CALIBRATION_POSE_ADJUSTMENTS,{calib_str}")
    print("="*40)
    print("Másold ezt be a 'follower_calibration.csv' utolsó sorába!")

    driver.close()

if __name__ == "__main__":
    sculpt_robot()