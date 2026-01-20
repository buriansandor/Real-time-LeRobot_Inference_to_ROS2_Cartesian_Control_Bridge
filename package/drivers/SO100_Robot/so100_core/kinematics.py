"""
Kinematics module for SO100 Robot using ikpy

Created by Sandor Burian based on the summary of Gemini Pro about the urdf_based_kinematics_bridge.py.
Corrected with the help of Copilot (Claude Sonnet 4)
"""

import ikpy.chain
import numpy as np
import os

class SO100Kinematics:
    def __init__(self, urdf_path):
        # Load the chain
        # active_links_mask: In your driver it was [False, True, True, True, True, True, False]
        # This is critical for correct operation!
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["base"],  # SO100 URDF uses "base" not "base_link"
            active_links_mask=[False, True, True, True, True, True, False] 
        )

    def inverse_kinematics(self, target_pos, target_orient=[0, 0, -1], orientation_mode="Z", seed_state=None):
        """
        Calculates the angles (in Radians)
        """
        return self.chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_orient,
            orientation_mode=orientation_mode,
            initial_position=seed_state  # Important for continuous movement!
        )

    def forward_kinematics(self, joint_angles):
        """
        Calculates where the hand is based on the angles (XYZ)
        """
        # The 0th element is the base (always 0), the last is the gripper end
        padded_joints = [0] + list(joint_angles) + [0]
        return self.chain.forward_kinematics(padded_joints)[:3, 3]