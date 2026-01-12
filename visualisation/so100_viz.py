#!/usr/bin/env python3
 
"""
Created by Sandor Burian with the help of Gemini3 PRO and GitHub Copilot (Claude Sonnet 4)
Live visualization of SO100 leader robot arm using IKpy and Matplotlib
"""

import argparse
import platform
import struct
import ikpy.chain
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import time
import numpy as np
from matplotlib.widgets import Button
from datetime import datetime

def init_3d_figure():
    """Initialize a 3D matplotlib figure for robot visualization"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Add view buttons
    button_height = 0.04
    button_width = 0.08
    button_left = 0.02
    button_spacing = 0.05
    
    
    # Create buttons for different views
    ax_top = plt.axes([button_left, 0.95 - button_spacing, button_width, button_height])
    ax_front = plt.axes([button_left, 0.95 - 2*button_spacing, button_width, button_height])
    ax_right = plt.axes([button_left, 0.95 - 3*button_spacing, button_width, button_height])
    ax_iso = plt.axes([button_left, 0.95 - 4*button_spacing, button_width, button_height])
    ax_left = plt.axes([button_left, 0.95 - 5*button_spacing, button_width, button_height])
    ax_back = plt.axes([button_left, 0.95 - 6*button_spacing, button_width, button_height])
    
    btn_top = Button(ax_top, 'Top')
    btn_front = Button(ax_front, 'Front')
    btn_right = Button(ax_right, 'Right')
    btn_iso = Button(ax_iso, 'Iso')
    btn_left = Button(ax_left, 'Left')
    btn_back = Button(ax_back, 'Back')
    
    # Define view functions
    def view_top(event):
        ax.view_init(elev=90, azim=-90)
        fig.canvas.draw_idle()
    
    def view_front(event):
        ax.view_init(elev=0, azim=-90)
        fig.canvas.draw_idle()
    
    def view_right(event):
        ax.view_init(elev=0, azim=0)
        fig.canvas.draw_idle()
    
    def view_iso(event):
        ax.view_init(elev=30, azim=-60)
        fig.canvas.draw_idle()
    
    def view_left(event):
        ax.view_init(elev=0, azim=180)
        fig.canvas.draw_idle()
    
    def view_back(event):
        ax.view_init(elev=0, azim=90)
        fig.canvas.draw_idle()
    
    # Connect buttons to functions
    btn_top.on_clicked(view_top)
    btn_front.on_clicked(view_front)
    btn_right.on_clicked(view_right)
    btn_iso.on_clicked(view_iso)
    btn_left.on_clicked(view_left)
    btn_back.on_clicked(view_back)
    
    return fig, ax

def find_port():
    """Find available serial port using pyserial or manually if pyserial is not available"""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if ports:
            return ports[0].device
    except:
        pass
    
    # Fallback
    print(f"Error finding port with PySerial: {e}")
    print("Falling back to manual method...")
    print("Available ports:")
    for p in ports:
        print(f" - {p.device}\t {p.description}")
    portnumber = input("Enter the port number for your device (e.g., COM5 or /dev/ttyUSB0): ")
    return portnumber

def find_port_with_lerobot():
    """Use lerobot's 'lerobot-find-port' to find the serial port"""
    import subprocess
    try:
        subprocess.call(['lerobot-find-port'])
        print("\nConnect again the robot in the same USB port!")
        port = input("Enter the port number for your device: ")
        if port:
            return port
    except Exception as e:
        print(f"Error finding port with LeRobot: {e}")
        print("Falling back to pyserial method...")
    return find_port()

def parse_arguments():
    """Parse command line arguments with interactive fallback"""
    parser = argparse.ArgumentParser(description='SO-100 Robot Arm Visualization')
    parser.add_argument('--port', '-p', type=str, help='Serial port (e.g., COM5 or /dev/ttyUSB0)')
    parser.add_argument('--baud-rate', '-b', type=int, default=1000000, help='Baud rate (default: 1000000)')
    parser.add_argument('--simulation', '-s', action='store_true', help='Run in simulation mode')
    parser.add_argument('--loose_motors', '-l', action='store_true', help='Loosen motors for manual movement, default: false')
    parser.add_argument('--urdf', '-u', type=str, default='../demo/SO100/URDF/so100.urdf', help='Path to URDF file, default: ../demo/SO100/URDF/so100.urdf')
    parser.add_argument('--motor-ids', '-m', nargs='+', type=int, default=[1, 2, 3, 4, 5, 6], help='Motor IDs (default: 1 2 3 4 5 6)')
    parser.add_argument('--logging_on', '-log', action='store_true', help='Enable movement logging, default: false')
    parser.add_argument('--logging_to_file', '-logf', action='store_true', help='Enable movement logging to file, default: false')

    return parser.parse_args()

def disable_torque(ser, motor_id):
    """Disables the motor torque to allow 1nual movement (Leader mode)"""
    # Instruction: Write (0x03), Address: 40 (Torque Enable), Value: 0
    msg = [0xFF, 0xFF, motor_id, 0x04, 0x03, 0x28, 0x00]
    msg.append(checksum(msg[2:])) 
    ser.write(bytearray(msg)) 
    time.sleep(0.005) # Small pause for the bus

# --- STS3215 Communication ---

def checksum(data):
    return (~sum(data)) & 0xFF

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

def raw_to_radians(raw_value, offset=2048, direction=1):
    """Converts the 0-4096 value to radians (between -PI and +PI)"""
    # 2048 is the center position (0 degrees)
    # 1 step = 360 / 4096 degrees
    if raw_value is None: return 0
    return (raw_value - offset) * (2 * math.pi / 4096) * direction

# -------------------------------------------------------

def get_real_servo_positions():
    """
    Reads the current servo positions from the robot and converts to radians.
    Returns a list of angles in radians for each joint.
    """
    if SIMULATION_MODE:
        # Generate a slow oscillating movement for testing
        t = time.time()
        pan = math.sin(t) * 0.5
        lift = math.cos(t) * 0.5
        # [Base, Pan, Lift, Elbow, Wrist, Roll, Gripper]
        # Important: The 0th element is the Base (fixed), we always keep it at 0.
        base_anim = [0, math.sin(t)*0.5, math.cos(t)*0.5, -1.0, -0.5, 0]
        #return [0, pan, lift, -1.0, -0.5, 0, 0]
        return [b + a for b, a in zip(base_anim, CALIBRATION_POSE_ADJUSTMENTS)] + [0]
    else:
        angles = [0] # The Base (0th link) is always fixed
        
        for i, mid in enumerate(MOTOR_IDS):
            raw = read_position(ser, mid)
            # Calibration note: 
            # The SO-100 motors are sometimes reversed, or 2048 is not zero.
            # Here we use a basic conversion. If it looks strange, adjust the +/- sign here.
            
            offset = ZERO_OFFSETS[i]
            direction = DIRECTIONS[i]
            relative_rad = raw_to_radians(raw, offset, direction)

            final_rad = relative_rad + CALIBRATION_POSE_ADJUSTMENTS[i]
            angles.append(final_rad)

        # If we found fewer motors, fill up with 0s
        while len(angles) < len(my_chain.links):
            angles.append(0)
        
        # Logging
        if args.logging_on:
            debug_deg = [round(math.degrees(a), 1) for a in angles]
            print(f"DEBUG OUT: {debug_deg}")
        return angles

def update_plot(frame_idx):
    """
    Updates the plot with current robot positions.
    Detects and logs movement when joints change by more than 1 degree.
    Generated by Gemini Pro.
    """
    global last_angles  # So we can write to the outer variable
    
    # 1. Get current angles
    current_angles = get_real_servo_positions()
    
    # Fill with 0s if needed
    if len(current_angles) < n_motors:
        current_angles += [0] * (n_motors - len(current_angles))

    # --- LOGGING: MOVEMENT DETECTION ---
    if last_angles is not None:
        # Calculate the difference (Delta) for each motor
        # Convert to degrees, as it's easier to read than radians
        diffs = []
        moved = False
        
        for i in range(len(current_angles)):
            # Difference in radians
            delta_rad = current_angles[i] - last_angles[i]
            # Convert to degrees
            delta_deg = math.degrees(delta_rad)
            diffs.append(delta_deg)
            
            # If it moved more than 1 degree, we consider it "movement"
            if abs(delta_deg) > 1.0:
                moved = True

        # Only print to console if there was actual movement!
        if moved:
            # Formatted output (only 1 decimal place)
            diff_str = ", ".join([f"{d:5.1f}°" for d in diffs])
            if(args.logging_on):
                print(f"MOVEMENT: [{diff_str}]")
            if(args.logging_to_file):
                with open(f"so100_movement_log_{timestamp}.txt", "a") as f:
                    f.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - MOVEMENT: [{diff_str}]\n")

    # Save the current one for the next iteration
    last_angles = current_angles
    # -----------------------------------

    # 2. FK Calculation and 3. Drawing (REMAINS THE SAME)
    real_frame = my_chain.forward_kinematics(current_angles)
    tcp_pos = real_frame[:3, 3] 
    
    ax.clear()
    ax.set_xlim(-0.3, 0.3); ax.set_ylim(-0.3, 0.3); ax.set_zlim(0, 0.4)
    ax.set_title(f"TCP: X={tcp_pos[0]:.2f}, Y={tcp_pos[1]:.2f}, Z={tcp_pos[2]:.2f}")
    my_chain.plot(current_angles, ax, target=tcp_pos)
    
    fig.canvas.draw()
    fig.canvas.flush_events()

def update_plot_old(frame_idx):
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

# --- Configuration ---

# Parse command line arguments
args = parse_arguments()
# Temp data for the logging
last_angles = None
if (args.logging_to_file):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    print(f"Logging movements to file: so100_movement_log_{timestamp}.txt")

print("Initializing SO-100 real-time visualization settings...")

# Load URDF file
URDF_FILE = args.urdf
my_chain = ikpy.chain.Chain.from_urdf_file(
    URDF_FILE,
    base_elements=["base"]                  # Specify the correct base element name
)
MOTOR_IDS = args.motor_ids

print("SO-100 URDF loaded...")
print(f"Number of joints: {len(my_chain.links)}")

# Setup the offsets of the robotic arm
ZERO_OFFSETS = [2197, 1994, 1091, 2045, 2928, 1675]
DIRECTIONS = [-1, 1, 1, 1, 1, 1]
CALIBRATION_POSE_ADJUSTMENTS = [0, 1.45, -2.8, -0.5, 1.5, 0] #[0, 1.45, -2.7, -1.57, 1.5, 0]


# Determine simulation mode
if args.simulation:
    SIMULATION_MODE = True
    print("Running in simulation mode (from command line)")
else:
    if args is None:
        # Interactive fallback if not specified
        print("Press enter to continue or s to run in simulation mode...")
        user_input = input().strip().lower()
        SIMULATION_MODE = (user_input == 's')
    else:
        SIMULATION_MODE = False
    print(f"Running in {'simulation' if SIMULATION_MODE else 'real'} mode")

# Determine serial port
if args.port:
    SERIAL_PORT = args.port
    print(f"Using port: {SERIAL_PORT}")
else:
    # Interactive fallback
    if not SIMULATION_MODE:
        print("Press enter to continue with COM5 on Windows, /dev/ttyUSB0 on Linux, or c to configure another port.")
        user_input = input().strip().lower()
        if user_input == 'c':
            SERIAL_PORT = find_port_with_lerobot()     # On Windows e.g. COM5, on Linux /dev/ttyUSB0
        else:
            SERIAL_PORT = 'COM5' if platform.system() == 'Windows' else '/dev/ttyUSB0'
    else:
        SERIAL_PORT = None  # Not needed in simulation mode

BAUD_RATE = args.baud_rate

print(f"Configuration: Simulation={SIMULATION_MODE}, Port={SERIAL_PORT}, Baud={BAUD_RATE}")
# ----------------

print("\nStarting SO-100 real-time visualization...")
print("Assuming a Waveshare/Feetech driver for the SO100 connected via USB.")
print("---------------------------------------")

if not SIMULATION_MODE:
    import serial
    print(f"Using port: {SERIAL_PORT}, baud rate: {BAUD_RATE}")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    if(args.loose_motors):
        print("Loosing motors in the robotic arm...")
        for mid in MOTOR_IDS:
            disable_torque(ser, mid)
        print("Now the arm can be moved!")
    

my_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE, base_elements=["base"])
n_motors = len(my_chain.links)
print(f"Robot loaded. Number of links: {n_motors}")

plt.ion()  # Enable interactive mode (this is key for movement!)
fig, ax = init_3d_figure()
ax.set_xlim(-0.3, 0.3)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(0, 0.4)
ax.set_title("SO-100 Real-Time Digital Twin")

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


