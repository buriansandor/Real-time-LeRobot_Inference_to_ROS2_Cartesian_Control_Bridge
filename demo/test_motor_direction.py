import time
import sys
import os

# Mivel a demo mappában vagyunk, a driver importja így néz ki:
try:
    from SO100.so100_control_driver import SO100ControlDriver
except ImportError:
    # Ha mégis máshonnan futtatnád
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    from demo.SO100.so100_control_driver import SO100ControlDriver

def test_direction():
    print("--- MOTOR IRÁNY TESZT ---")
    port = input("Add meg a portot (pl. COM4): ")
    driver = SO100ControlDriver(port=port)
    
    print("\nKérlek, tedd a robotot 'L' alakba (könyök 90 fok)!")
    print("Ha kész, nyomj Entert!")
    input()

    # 1. Nyomaték bekapcsolása (A drivered függvényét használva)
    print("Motorok rögzítése...")
    for i in range(1, 7):
        driver.torque_enable(i)
        
    # --- 2-es MOTOR (Váll) TESZT ---
    print("\n--- 2-es MOTOR (Váll) TESZT ---")
    current_angle = driver.read_angle(2)
    print(f"Jelenlegi szög (rad): {current_angle:.2f}")
    
    # Hozzáadunk +0.3 radiánt (kb 17 fok)
    # Az URDF-ben a '0 1 0' tengely miatt a POZITÍV iránynak ELŐRE/LEFELÉ kell vinnie a kart.
    target = current_angle + 0.3
    print(f"Mozgás ide: {target:.2f} (Pozitív irány)...")
    
    driver.set_target_angle(2, target, move_time_ms=1500)
    time.sleep(2.0)
    
    print("\n>>> KÉRDÉS: MERRE MOZDULT A KAR?")
    print(" [A] LEFELÉ / ELŐRE (Bólintott az asztal felé)")
    print(" [B] FELFELÉ / HÁTRA (Ágaskodott)")
    valasz2 = input("Írd be a betűt (a/b): ").lower()
    
    # Visszaállás
    driver.set_target_angle(2, current_angle, move_time_ms=1000)
    time.sleep(1.5)

    # --- 3-as MOTOR (Könyök) TESZT ---
    print("\n--- 3-as MOTOR (Könyök) TESZT ---")
    current_angle = driver.read_angle(3)
    print(f"Jelenlegi szög (rad): {current_angle:.2f}")
    
    # Hozzáadunk +0.3 radiánt
    target = current_angle + 0.3
    print(f"Mozgás ide: {target:.2f} (Pozitív irány)...")
    
    driver.set_target_angle(3, target, move_time_ms=1500)
    time.sleep(2.0)
    
    print("\n>>> KÉRDÉS: MERRE MOZDULT AZ ALKAR?")
    print(" [A] LEFELÉ / KINYÚJT (Kiegyenesedett a kar)")
    print(" [B] FELFELÉ / BEHAJLÍT (Maga alá húzta a kezét)")
    valasz3 = input("Írd be a betűt (a/b): ").lower()
    
    # Visszaállás
    driver.set_target_angle(3, current_angle, move_time_ms=1000)
    time.sleep(1.0)
    
    # Kikapcsolás
    driver.torque_disable()
    driver.close()

    print("\n--- EREDMÉNY ÉS TEENDŐ ---")
    print("Nyisd meg a 'follower_calibration.csv'-t!")
    
    print(f"Motor 2 (Shoulder): ", end="")
    if valasz2 == 'a':
        print("MOZGÁS JÓ -> Írj 1-est a CSV-be!")
    else:
        print("MOZGÁS FORDÍTOTT -> Írj -1-et a CSV-be!")

    print(f"Motor 3 (Elbow):    ", end="")
    if valasz3 == 'a':
        print("MOZGÁS JÓ -> Írj 1-est a CSV-be!")
    else:
        print("MOZGÁS FORDÍTOTT -> Írj -1-et a CSV-be!")

if __name__ == "__main__":
    test_direction()