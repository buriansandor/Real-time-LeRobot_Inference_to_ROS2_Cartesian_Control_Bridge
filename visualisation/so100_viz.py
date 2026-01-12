#!/usr/bin/env python3
 
"""
Created by Sandor Burian with the help of Gemini3 PRO and GitHub Copilot (Claude Sonnet 4)
Live visualization of SO100 leader robot arm using IKpy and Matplotlib
"""

import struct
import ikpy.chain
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import time
import numpy as np

# Simple 3D plot initialization
def init_3d_figure():
    """Initialize a 3D matplotlib figure for robot visualization"""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    return fig, ax

# Simple port detection using lerobot's pyserial dependency
def find_port():
    """Find available serial port using pyserial (lerobot dependency)"""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if ports:
            return ports[0].device
    except:
        pass
    
    # Fallback
    import platform
    return "COM5" if platform.system() == "Windows" else "/dev/ttyUSB0"

# --- Settings ---

URDF_FILE = "../demo/SO100/URDF/so100.urdf"
my_chain = ikpy.chain.Chain.from_urdf_file(
    URDF_FILE,
    base_elements=["base"]  # Specify the correct base element name
)
MOTOR_IDS = [1, 2, 3, 4, 5, 6]  # STS3215 servo IDs for SO-100
print("--- SO-100 URDF loaded ---")
print(f"Number of joints: {len(my_chain.links)}")
print("Press enter to continue or s to run in simulation mode...")
user_input = input().strip().lower()
SIMULATION_MODE = (user_input == 's')
SERIAL_PORT = 'COM5'     # On Windows e.g. COM5, on Linux /dev/ttyUSB0
BAUD_RATE = 1000000      # Default speed of the STS3215
# ----------------

print("\nStarting SO-100 real-time visualization...")
print("We asssume you have a Waveshare/Feetech driver for the SO100 connected via USB.")
print("---------------------------------------")

if not SIMULATION_MODE:
    import serial
    SERIAL_PORT = 'COM5'
    print(f"Using port: {SERIAL_PORT}, baud rate: {BAUD_RATE}")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    

my_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE, base_elements=["base"])
n_motors = len(my_chain.links)
print(f"Robot loaded. Number of links: {n_motors}")

plt.ion()  # Enable interactive mode (this is key for movement!)
fig, ax = init_3d_figure()
ax.set_xlim(-0.3, 0.3)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(0, 0.4)
ax.set_title("SO-100 Real-Time Digital Twin")


# --- STS3215 Communication ---

def checksum(data):
    return (~sum(data)) & 0xFF

def disable_torque(ser, motor_id):
    """Disables the motor torque to allow manual movement (Leader mode)"""
    # Instruction: Write (0x03), Address: 40 (Torque Enable), Value: 0
    msg = [0xFF, 0xFF, motor_id, 0x04, 0x03, 0x28, 0x00]
    msg.append(checksum(msg[2:]))
    ser.write(bytearray(msg))
    time.sleep(0.005) # Small pause for the bus

def read_position(ser, motor_id):
    """Reads the current position (0-4096)"""
    # Instruction: Read (0x02), Address: 56 (Present Position), Length: 2
    msg = [0xFF, 0xFF, motor_id, 0x04, 0x02, 0x38, 0x02]
    msg.append(checksum(msg[2:]))
    
    ser.reset_input_buffer() # Clear the buffer
    ser.write(bytearray(msg))
    
    response = ser.read(8)
    
    if len(response) == 8:
        pos_raw = struct.unpack('<H', response[5:7])[0]
        return pos_raw
    return None

def raw_to_radians(raw_value, offset=2048):
    """Converts the 0-4096 value to radians (between -PI and +PI)"""
    # 2048 is the center position (0 degrees)
    # 1 step = 360 / 4096 degrees
    if raw_value is None: return 0
    return (raw_value - offset) * (2 * math.pi / 4096)

# -------------------------------------------------------

def get_real_servo_positions():
    """
    This function needs to be written for your specific driver.
    """
    if SIMULATION_MODE:
        # Generate a slow oscillating movement for testing
        t = time.time()
        pan = math.sin(t) * 0.5
        lift = math.cos(t) * 0.5
        # [Base, Pan, Lift, Elbow, Wrist, Roll, Gripper]
        # Important: The 0th element is the Base (fixed), we always keep it at 0.
        return [0, pan, lift, -1.0, -0.5, 0, 0]
    else:
        angles = [0] # The Base (0th link) is always fixed
        
        for mid in MOTOR_IDS:
            raw = read_position(ser, mid)
            # Calibration note: 
            # The SO-100 motors are sometimes reversed, or 2048 is not zero.
            # Here we use a basic conversion. If it looks strange, adjust the +/- sign here.
            rad = raw_to_radians(raw)
            
            # Example correction (if, for example, the elbow moves in reverse):
            # if mid == 3: rad = -rad 
            
            angles.append(rad)

        # If we found fewer motors, fill up with 0s
        while len(angles) < len(my_chain.links):
            angles.append(0)

        return angles

def update_plot(frame_idx):
    """
    This loop runs continuously.
    1. Read
    2. Calculate (FK)
    3. Draw
    """

    current_angles = get_real_servo_positions()
    
    if len(current_angles) < n_motors:
        current_angles += [0] * (n_motors - len(current_angles))

    real_frame = my_chain.forward_kinematics(current_angles)
    tcp_pos = real_frame[:3, 3] # X, Y, Z
    ax.clear()
    
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(0, 0.4)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"TCP: X={tcp_pos[0]:.2f}, Y={tcp_pos[1]:.2f}, Z={tcp_pos[2]:.2f}")

    my_chain.plot(current_angles, ax, target=tcp_pos)
    
    fig.canvas.draw()
    fig.canvas.flush_events()


print("\n\nStarting visualization... (Close the visualisation window then press Ctrl+C to stop)")
try:
    frame = 0
    while True:
        update_plot(frame)
        frame += 1
        time.sleep(0.05) 
except KeyboardInterrupt:
    print("\nStopped.")
    plt.close()