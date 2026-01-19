import os
import sys

# Próbáljuk importálni a drivert
try:
    from SO100.so100_control_driver import SO100ControlDriver
except ImportError:
    try:
        sys.path.append(os.path.join(os.path.dirname(__file__), 'demo'))
        from so100.so100_control_driver import SO100ControlDriver
    except:
        print("Nem találom a drivert, de most a CSV a lényeg...")

print("\n--- NYOMOZÁS INDUL ---")

# 1. Hol vagyunk most?
current_dir = os.getcwd()
print(f"1. A Python ebben a mappában dolgozik éppen:\n   -> {current_dir}")

# 2. Látja-e a fájlt itt?
expected_file = os.path.join(current_dir, 'follower_calibration.csv')
if os.path.exists(expected_file):
    print(f"2. IGEN! Látom a fájlt itt: {expected_file}")
    
    # 3. Mi van benne SZERINTE?
    print("3. A fájl tartalma a Python szerint:")
    print("-" * 30)
    with open(expected_file, 'r') as f:
        content = f.read()
        print(content.strip())
    print("-" * 30)
    
    # Ellenőrzés: Benne van-e a módosításod?
    if "DIRECTIONS,1,-1,1,1,1,1" in content:
        print("   -> LÁTOM a (1, -1, 1, 1, 1, 1) beállítást a szövegben.")
    else:
        print("   -> NEM LÁTOM a módosításodat! Ez a fájl nem az, amit szerkesztettél!")

else:
    print(f"2. HOHÓ! NEM LÁTOM a fájlt itt: {expected_file}")
    print("   A Python nem találja a CSV-t, ezért valószínűleg alapértelmezett értékekkel dolgozik!")

print("\n--- DRIVER BETÖLTÉSE ---")
# 4. Mit tölt be a Driver?
try:
    # Direkt nem adunk meg fájlnevet, hadd keresse a magáét
    # Portot se kell megadni, ha csak a config betöltést teszteljük (ha a driver engedi)
    # Ha a driver csatlakozni akar, add meg a portodat!
    port = input("Add meg a COM portot a teszthez (pl. COM4): ")
    driver = SO100ControlDriver(port=port) 
    
    print("\n4. A Driver memóriájában lévő irányok:")
    print(f"   {driver.motor_directions}")
    
    if driver.motor_directions == [1, -1, 1, 1, 1, 1]:
        print("   -> A Driver JÓL töltötte be az adatokat.")
    else:
        print("   -> A Driver MÁS adatokat használ! (Valószínűleg nem olvasta be a fájlt).")
        
    driver.close()

except Exception as e:
    print(f"Hiba a driver indításakor: {e}")