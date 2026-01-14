import ikpy.chain
import numpy as np
import time
from so100_control_driver import SO100ControlDriver

class SmartSO100:
    def __init__(self, port, urdf_path, calibration_file="calibration.csv"):
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["base"])
        self.hw = SO100ControlDriver(port, calibration_file)
        self.current_angles = [0] * len(self.chain.links)
        print("Smart SO-100 Ready!")

    def move_to_xyz(self, x, y, z, duration=1.0):
        """
        Cartesian movement.
        duration: seconds
        """
        target_pos = [x, y, z]
        
        # A. Inverse Kinematics calculation
        target_angles_rad = self.chain.inverse_kinematics(
            target_position=target_pos,
            initial_position=self.current_angles
        )
        
        # B. IK Validation (Can we reach the target?)
        real_pos = self.chain.forward_kinematics(target_angles_rad)[:3, 3]
        error = np.linalg.norm(real_pos - target_pos)
        
        if error > 0.05:
            print(f"ERROR: Target point ({x},{y},{z}) unreachable! Error: {error:.3f}m")
            return False
            
        # C. Hardware Execution
        # ikpy returns a list where the first element (index 0) is the 'base' (dummy).
        # The motors only need J1..J6.
        # Check the URDF structure, but usually the [1:] slice is needed.
        
        motor_commands = target_angles_rad[1:] # Remove base
        
        # Safety check
        if len(motor_commands) != 6:
             # If the gripper is included in the URDF, we may have 7 angles.
             print(f"WARNING: Expected 6 motor commands, got {len(motor_commands)}. Truncating to 6.")
             motor_commands = motor_commands[:6]
        
        print(f"Starting movement... (Duration: {duration}s)")
        self.hw.set_target_joints(motor_commands, move_time_ms=int(duration*1000))
        
        # D. Update memory
        self.current_angles = target_angles_rad
        return True