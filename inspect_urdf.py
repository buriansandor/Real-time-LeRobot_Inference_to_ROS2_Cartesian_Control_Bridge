import sys
import os
import ikpy.chain

# Beállítjuk az útvonalakat
current_dir = os.getcwd()
# Feltételezzük, hogy a driver mappában van az URDF (a logod alapján)
urdf_path = os.path.join(current_dir, "package", "drivers", "SO100_Robot", "config", "so100.urdf")

print(f"--- 🕵️ URDF INSPECTOR ---")
print(f"Vizsgált fájl: {urdf_path}")

if not os.path.exists(urdf_path):
    print("❌ HIBA: Nem találom az URDF fájlt ezen az útvonalon!")
    print("Kérlek, ellenőrizd, hogy a 'package/drivers/SO100_Robot/config' mappában van-e.")
    sys.exit(1)

try:
    # Betöltjük a láncot úgy, ahogy a kinematics.py teszi
    chain = ikpy.chain.Chain.from_urdf_file(
        urdf_path,
        base_elements=["base"], 
        active_links_mask=[False, True, True, True, True, True, False]
    )
    
    print("\n✅ SIKER: A lánc betöltődött!")
    print(f"Összes link száma: {len(chain.links)}")
    
    print("\n🔗 LINK LISTA (És hogy aktív-e):")
    print("-" * 50)
    print(f"{'Index':<5} | {'Link Neve':<20} | {'Típus':<10} | {'Aktív?'}")
    print("-" * 50)
    
    for i, link in enumerate(chain.links):
        active_status = "✅ IGEN" if i in chain.active_links_mask else "❌ NEM"
        # Próbáljuk kitalálni a joint típusát (revolute/fixed)
        j_type = "Unknown"
        if hasattr(link, 'joint_type'):
             j_type = link.joint_type
             
        print(f"{i:<5} | {link.name:<20} | {j_type:<10} | {active_status}")
        
    print("-" * 50)
    print("\nELEMZÉS:")
    active_count = sum(1 for x in chain.active_links_mask if x)
    print(f"Aktív motorok száma a maszk szerint: {active_count}")
    
    if active_count == 5:
        print("✅ Ez HELYES egy 5-DOF robotkarhoz (plusz gripper külön).")
    else:
        print(f"⚠️ FIGYELEM: {active_count} aktív motort találtam. Biztos ez kell?")

except Exception as e:
    print(f"\n❌ KRITIKUS HIBA az URDF betöltésekor:\n{e}")
    print("\nTipp: Lehet, hogy a 'base_elements' nem 'base', hanem 'base_link'?")