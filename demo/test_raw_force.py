import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from demo.SO100.so100_control_driver import SO100ControlDriver
import time

# Írd be a portot!
PORT = 'COM4'

print("Csatlakozás...")
# Nem számít a kalibráció, mert most "nyersen" írunk
driver = SO100ControlDriver(port=PORT, simulation=False)

print("\nFIGYELEM! Most KÖZVETLENÜL küldjük a Gyertya értékeket.")
print("Ha a robot most sem áll fel, akkor a kommunikációval/motorral van baj.")
time.sleep(2)

# 1. Nyomaték bekapcsolása (Mindkét lehetséges címen)
print("Nyomaték bekapcsolása (Cím: 40 és 0)...")
for i in range(1, 7):
    # Cím 40 (0x28) - Standard STS
    driver._write_packet(i, 0x03, [0x28, 0x01])
    time.sleep(0.02)
    
print("Várakozás...")
time.sleep(1.0)

# 2. Mozgatás teszt (1-es motor, Derék)
# Először kicsit balra, aztán jobbra, hogy lássuk, él-e
motor_id = 1
raw_target = 2500 # Kicsit elfordítva a közép (2048)-tól

print(f"TESZT 1: Küldés a 0x2A (42) címre (Standard)...")
p_low = raw_target & 0xFF
p_high = (raw_target >> 8) & 0xFF
# Idő nélkül, csak pozíció + sebesség (néha ez a baj)
# [PosL, PosH, TimeL, TimeH, SpdL, SpdH]
params = [0x2A, p_low, p_high, 0xE8, 0x03, 0x00, 0x00] # Time = 1000 (0x03E8)
driver._write_packet(motor_id, 0x03, params)

time.sleep(2)

print(f"TESZT 2: Küldés a 0x1E (30) címre (Alternatív)...")
# Ha a fenti nem ment, ez a régi SMS/SCS protokoll címe
params_alt = [0x1E, p_low, p_high, 0xE8, 0x03, 0x00, 0x00]
driver._write_packet(motor_id, 0x03, params_alt)

print("\nMozgott valamelyikre? (Figyeld a derekát!)")