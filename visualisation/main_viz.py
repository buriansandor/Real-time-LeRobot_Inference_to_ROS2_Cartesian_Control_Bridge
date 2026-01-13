#!/usr/bin/env python3
"""
Main Visualization for SO100 to Annin AR4 Mapping

Created by Sandor Burian with the help of Google Gemini Pro
"""

import matplotlib.pyplot as plt
from ikpy.utils import plot_utils
import sys
import os
import keyboard
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from SO100.so100_driver import SO100Driver
from kinematics.kinematic_bridge import KinematicBridge, MappingConfig

# --- Config ---
PORT = SO100Driver.args.port
URDF = SO100Driver.args.urdf if SO100Driver.args.urdf else "../SO100/URDF/so100.urdf"

def main():
    print("Hardware init...")
    leader = SO100Driver(port=PORT, urdf_path=URDF, simulation=False)
    
    # Create the "Bridge" configuration
    config = MappingConfig(
        scale_factor=1.8,
        min_z=0.05,
        max_radius=0.6,
        offset_x=0.0 # No offset at the start
    )
    bridge = KinematicBridge(config)
    
    # 2. Graphics
    plt.ion()
    fig, ax = plot_utils.init_3d_figure()
    
    print("\n--- Control ---")
    print("Move the SO-100 manually.")
    print("Keys for tuning the MAPPING:")
    print("  [W/S] : Move AR4 target forward/backward (Offset X)")
    print("  [A/D] : Move AR4 target right/left (Offset Y)")
    print("  [Q/E] : Increase/decrease scaling")
    print("  [ESC] : Exit")
    
    try:
        while True:
            # --- INPUT ---
            angles = leader.get_joint_angles()
            so100_pos = leader.get_tcp_position(angles)
            
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
            
            # SO-100 (Blue)
            leader.chain.plot(angles, ax, target=so100_pos)
            
            # AR4 Target (Red Sphere)
            ax.scatter(ar4_target[0], ar4_target[1], ar4_target[2], c='red', s=100, label='AR4 Target')
            
            # Visual helper lines
            ax.plot([so100_pos[0], ar4_target[0]], 
                    [so100_pos[1], ar4_target[1]], 
                    [so100_pos[2], ar4_target[2]], 'k:', alpha=0.3)

            # Info display in the title
            title_str = (f"Scale: {config.scale_factor:.1f}x | "
                         f"Offset X: {config.offset_x:.2f}m | "
                         f"Target Z: {ar4_target[2]:.2f}m")
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