import os
import sys

# Adjuk hozzá a package könyvtárat a path-hoz
package_dir = os.path.join(os.getcwd(), 'package')
sys.path.insert(0, package_dir)

# Próbáljuk importálni a drivert
try:
    from package.drivers.SO100_Robot import SO100Robot
    print("✅ SIKER: A SO100Robot osztály importálható.")
except ImportError as e:
    print(f"❌ HIBA: Nem tudom importálni a drivert! {e}")
    sys.exit(1)

# Ellenőrizzük a fájlokat
current_dir = os.getcwd()
print(f"\nJelenlegi mappa: {current_dir}")

expected_config = os.path.join(current_dir, "package", "drivers", "SO100_Robot", "config", "follower_calibration.csv")
print(f"Itt keresem a kalibrációt: {expected_config}")

if os.path.exists(expected_config):
    print("✅ A fájl LÉTEZIK a lemezen.")
    with open(expected_config, 'r') as f:
        print(f"   --> Tartalom első sora: {f.readline().strip()}")
else:
    print("❌ A fájl NEM LÉTEZIK ezen az útvonalon!")

# Robot példányosítás teszt (kapcsolódás nélkül)
print("\n--- Robot Osztály Teszt ---")
try:
    # Direkt rossz config dir-t adunk meg alapból, hogy lássuk mit csinál
    robot = SO100Robot(port="DUMMY", config_dir="drivers/SO100_Robot/config")
    print(f"Robot Offsetek: {robot.offsets}")
    if robot.offsets.get(0) == 2048:
        print("❌ HIBA: A robot az alapértelmezett (2048) offseteket töltötte be! (Ezért áll ferdén)")
    else:
        print(f"✅ SIKER: Egyedi offset betöltve: {robot.offsets.get(0)}")
except Exception as e:
    print(f"Hiba a robot létrehozásakor: {e}")