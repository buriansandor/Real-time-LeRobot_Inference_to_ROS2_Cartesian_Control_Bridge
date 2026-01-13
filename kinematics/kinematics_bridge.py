#!/usr/bin/env python3
"""
Kinematics Bridge between Leader and Follower Robots
Created by Sandor Burian with the help of Google Gemini Pro

This module transforms the end-effector positions from the Leader robot's
coordinate system to the Follower robot's coordinate system,
applying scaling, rotation, translation, and safety limits.
"""
import numpy as np
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class MappingConfig:
    """Store of configurations"""
    scale_factor: float = 1.0           # How much to scale the movement?
    offset_x: float     = 0.0           # How much to offset in X direction? (meters)
    offset_y: float     = 0.0           # How much to offset in Y direction?
    offset_z: float     = 0.0           # How much to offset in Z direction?
    rotation_z: float   = 0.0           # Rotation around the vertical axis (degrees)
    
    # safety limits of follower robot
    min_z: float = 0.05             # Should not go below this (table protection)
    max_z: float = 0.90             # Should not go above this (ceiling protection)
    min_radius: float = 0.02        # Should not go closer than this (singularity level)
    max_radius: float = 0.8         # Should not reach beyond this (cylindrical workspace)

class KinematicBridge:
    def __init__(self, config: MappingConfig):
        self.config = config
        self.limit_reached = False

    def transform(self, leader_pos: List[float]) -> List[float]:
        """
        Input:  [x, y, z] in the Leader robot's coordinate system
        Output: [x, y, z] in the Follower robot's coordinate system
        """
        self.limit_reached = False

        lx, ly, lz = leader_pos
        c = self.config
        
        # 1. SCALING
        x = lx * c.scale_factor
        y = ly * c.scale_factor
        z = lz * c.scale_factor
        
        # 2. ROTATION (around Z axis)
        # If the two robots are not facing the same direction
        if c.rotation_z != 0:
            rad = np.radians(c.rotation_z)
            x_new = x * np.cos(rad) - y * np.sin(rad)
            y_new = x * np.sin(rad) + y * np.cos(rad)
            x, y = x_new, y_new

        # 3. TRANSLATION (Offset)
        # This allows you to move the "workspace" on the big robot's table
        x += c.offset_x
        y += c.offset_y
        z += c.offset_z
        
        # 4. SAFETY LIMITS
        # Floor protection
        if z < c.min_z: 
            z = c.min_z
            self.limit_reached = True
        elif z > c.max_z:
            z = c.max_z
            self.limit_reached = True
            
        # Range protection (cylindrical workspace)
        radius = np.sqrt(x**2 + y**2)
        if radius > c.max_radius:
            scale_back = c.max_radius / radius
            x *= scale_back
            y *= scale_back
            self.limit_reached = True
        elif radius < c.min_radius:
            if radius < 0.001:   # in 0,0  can't scale back
                x = c.min_radius # push in X direction
                y = 0
            else:
                scale_out = c.min_radius / radius
                x *= scale_out
                y *= scale_out
            self.limit_reached = True
        return [x, y, z]

    def update_config(self, **kwargs):
        """Modify settings at runtime (e.g., from GUI or config file)"""
        for key, value in kwargs.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)