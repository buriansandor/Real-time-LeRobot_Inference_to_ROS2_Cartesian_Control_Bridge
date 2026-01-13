#!/usr/bin/env python3
"""
Main Visualization for SO100 to Annin AR4 Mapping

Created by Sandor Burian with the help of Google Gemini Pro and Copilot (Claude Sonnet 4)
"""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import sys
import os
import keyboard
import time
import argparse
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from demo.SO100.so100_driver import SO100Driver
from kinematics.kinematics_bridge import KinematicsBridge, MappingConfig
from kinematics.urdf_based_kinematics_bridge import URDFBasedKinematicBridge, MappingConfig

def init_3d_figure():
    """Initialize a 3D matplotlib figure for robot visualization"""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    return fig, ax

# --- Config ---
def parse_arguments():
    """Parse command line arguments for visualization"""
    parser = argparse.ArgumentParser(description='SO-100 to AR4 Visualization')
    parser.add_argument('--port', '-p', type=str, default='COM5', help='Serial port (e.g., COM5 or /dev/ttyUSB0)')
    parser.add_argument('--urdf', '-u', type=str, default="../demo/SO100/URDF/so100.urdf", help='Path to URDF file')
    parser.add_argument('--simulation', '-s', action='store_true', help='Run in simulation mode')
    return parser.parse_args()

def draw_safety_zone(ax, config):
    """
    Work zone of the follower arm
    """
    # Resolution for drawing the circle
    theta = np.linspace(0, 2*np.pi, 50)
    z = np.linspace(config.min_z, config.max_z, 10)
    
    # Create grids (Meshgrid)
    theta_grid, z_grid = np.meshgrid(theta, z)
    
    # --- 1. OUTER CYLINDER (Max Reach) ---
    x_outer = config.max_radius * np.cos(theta_grid)
    y_outer = config.max_radius * np.sin(theta_grid)
    
    ax.plot_surface(x_outer, y_outer, z_grid, 
                    color='gray', alpha=0.1, rstride=5, cstride=5)

    # --- 2. INNER CYLINDER (Dead Zone) ---
    # This shows why the robot cannot stand "vertically"
    x_inner = config.min_radius * np.cos(theta_grid)
    y_inner = config.min_radius * np.sin(theta_grid)
    
    ax.plot_surface(x_inner, y_inner, z_grid, 
                    color='red', alpha=0.15, rstride=5, cstride=5) # Slightly reddish to be intimidating

    # --- 3. FLOOR and CEILING (Wireframe circles) ---
    # We only draw with lines so it doesn't cover everything
    ax.plot(x_outer[0, :], y_outer[0, :], config.min_z, 'k--', alpha=0.3) # Floor outer circle
    ax.plot(x_inner[0, :], y_inner[0, :], config.min_z, 'r--', alpha=0.3) # Floor inner circle
    
    ax.plot(x_outer[-1, :], y_outer[-1, :], config.max_z, 'k--', alpha=0.3) # Ceiling outer circle

args = parse_arguments()
PORT = args.port
URDF = args.urdf

def main(port_name=None, urdf_path=None, simulation=False, hardware=1):
    print("Hardware init...")
    
    # Determine the correct URDF path based on working directory
    if urdf_path is None:
        # Try different possible paths
        possible_paths = [
            args.urdf,                     # from command line argument
            "SO100/URDF/so100.urdf",        # from demo directory
            "../demo/SO100/URDF/so100.urdf", # from visualisation directory
            "URDF/so100.urdf"              # from SO100 directory
        ]
        
        urdf_path = None
        for path in possible_paths:
            if os.path.exists(path):
                urdf_path = path
                print(f"Found URDF at: {urdf_path}")
                break
        
        if urdf_path is None:
            raise FileNotFoundError("Could not find so100.urdf file")
    
    # Set up sys.argv for SO100Driver
    original_argv = sys.argv.copy()
    sys.argv = ['main_viz.py', '--urdf', urdf_path]
    if simulation:
        sys.argv.append('--simulation')
    if port_name:
        sys.argv.extend(['--port', port_name])
    
    try:
        leader = SO100Driver()
    finally:
        sys.argv = original_argv  # Restore original argv

    # Create the "Bridge" configuration
    if (hardware == 2):
        config = MappingConfig(
            scale_factor=1.8,
            min_z=0.05,
            max_radius=0.6,
            offset_x=0.0, # No offset at the start
            name="Arm B"
        )
        bridge = URDFBasedKinematicBridge(config, leader_urdf=urdf_path)
    else:
        config = MappingConfig(
            scale_factor=1.8,
            min_z=0.05,
            max_radius=0.6,
            offset_x=0.0, # No offset at the start
            name="AR4"
        )
        bridge = KinematicsBridge(config)
    
    # 2. Graphics
    plt.ion()
    fig, ax = init_3d_figure()
    
    print("\n--- Control ---")
    print("Move the SO-100 manually.")
    print("Keys for tuning the MAPPING:")
    print("  [W/S] : Move AR4 target forward/backward (Offset X)")
    print("  [A/D] : Move AR4 target right/left (Offset Y)")
    print("  [Q/E] : Increase/decrease scaling")
    print("  [ESC] : Exit")
    
    last_follower_angles = [0] * len(bridge.follower_chain.links) if bridge.follower_chain else [0]*7
    try:
        while True:
            # --- INPUT ---
            angles = leader.get_joint_angles()
            so100_pos = leader.get_tcp_position(angles)

            # the follower arm
            target_pos = bridge.transform(so100_pos)
            if bridge.follower_chain:
                # Az ikpy IK solverét használjuk
                follower_angles = bridge.follower_chain.inverse_kinematics(
                    target_position=target_pos, 
                    initial_position=last_follower_angles  
                )
                last_follower_angles = follower_angles
                # Kiszámoljuk, hova sikerült ténylegesen eljutni (ellenőrzés)
                actual_ar4_pos = bridge.follower_chain.forward_kinematics(follower_angles)[:3, 3]
            else:
                follower_angles = [0] * 7 # Ha nincs URDF, maradjon alaphelyzetben
                actual_ar4_pos = target_pos
                last_follower_angles = follower_angles
            
            # --- mapping changes with keyboard ---
            # live callibration of AR4
            step = 0.01 # 1 cm step
            if keyboard.is_pressed('w'): bridge.update_config(offset_x = config.offset_x + step)
            if keyboard.is_pressed('s'): bridge.update_config(offset_x = config.offset_x - step)
            if keyboard.is_pressed('a'): bridge.update_config(offset_y = config.offset_y + step)
            if keyboard.is_pressed('d'): bridge.update_config(offset_y = config.offset_y - step)
            
            if keyboard.is_pressed('q'): bridge.update_config(scale_factor = config.scale_factor + 0.05)
            if keyboard.is_pressed('e'): bridge.update_config(scale_factor = config.scale_factor - 0.05)
            
            if keyboard.is_pressed('esc'): break

            # --- TRANSFORMATION ---
            ar4_target = bridge.transform(so100_pos)
            
            # --- VISUALIZATION ---
            ax.clear()
            ax.set_xlim(-0.8, 0.8); ax.set_ylim(-0.8, 0.8); ax.set_zlim(0, 0.8)
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            
            draw_safety_zone(ax, bridge.config)

            # SO-100 (Blue)
            leader.chain.plot(angles, ax, target=so100_pos)

            # follower arm
            bridge.follower_chain.plot(follower_angles, ax, target=target_pos)
            # Change color based on if it's on limit
            target_color = 'orange' if bridge.limit_reached else 'red'

            # Target
            if (hardware==1):
                ax.scatter(ar4_target[0], ar4_target[1], ar4_target[2], c=target_color, s=100, label='AR4 Target')
            elif (hardware==2):
                ax.scatter(ar4_target[0], ar4_target[1], ar4_target[2], c=target_color, s=100, label=bridge.config.name)
            else:
                ax.scatter(ar4_target[0], ar4_target[1], ar4_target[2], c=target_color, s=100, label='Target')

            # Set robot arm text
            ax.text(so100_pos[0], so100_pos[1], so100_pos[2] + 0.05, "SO-100 (Leader)", color='blue', fontsize=10, fontweight='bold')
            if(bridge.config.name):
                label_text = f"{bridge.config.name} (Limit!)" if bridge.limit_reached else f"{bridge.config.name}"
            else:
                label_text = "AR4 (Limit!)" if bridge.limit_reached else "AR4 Target"
            ax.text(ar4_target[0], ar4_target[1], ar4_target[2] + 0.05, label_text, color=target_color, fontsize=10, fontweight='bold')
            
            # Visual helper lines
            ax.plot([so100_pos[0], ar4_target[0]], 
                    [so100_pos[1], ar4_target[1]], 
                    [so100_pos[2], ar4_target[2]], 'k:', alpha=0.3)

            # Info display in the title
            title_str = (f"Scale: {config.scale_factor:.1f}x | "
                         f"Offset X: {config.offset_x:.2f}m | "
                         f"Target Z: {ar4_target[2]:.2f}m")
            if bridge.limit_reached:
                ax.set_title(f"!!! LIMIT REACHED !!! Z={ar4_target[2]:.2f}m", color='red')
            elif simulation:
                ax.set_title(f"SIMULATION MODE - {title_str}", color='blue')
            else:
                ax.set_title(title_str)
            
            fig.canvas.draw()
            fig.canvas.flush_events()

    except KeyboardInterrupt:
        pass
    finally:
        plt.close()
        print("Program stopped.")

if __name__ == "__main__":
    main()