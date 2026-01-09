#!/usr/bin/env python3
"""
Simple IKpy demo for SO100 robot arm
Created by Sandor Burian with the help of GitHub Copilot (Claude Sonnet 4)
"""

import ikpy.chain
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

print("IKpy SO100 Robot Demo")
print("=====================")

try:
    # Try to load the URDF file with proper base elements
    print("Loading SO100 URDF...")
    my_chain = ikpy.chain.Chain.from_urdf_file(
        "../SO100/URDF/so100.urdf",
        base_elements=["base"]  # Specify the correct base element
    )
    print(f"✓ URDF loaded successfully!")
    print(f"Chain has {len(my_chain.links)} links")
    
    # Print chain information
    print("\nChain links:")
    for i, link in enumerate(my_chain.links):
        print(f"  {i}: {link.name}")
    
except Exception as e:
    print(f"✗ Error loading URDF: {e}")
    print("Creating a manual chain instead...")
    
    # Create a simple manual chain for demonstration
    my_chain = Chain(name='so100_manual', links=[
        OriginLink(),  # Fixed base
        URDFLink(name="shoulder", origin_translation=[0, 0, 0.1], origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
        URDFLink(name="upper_arm", origin_translation=[0, 0, 0.15], origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
        URDFLink(name="lower_arm", origin_translation=[0, 0, 0.15], origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
        URDFLink(name="wrist", origin_translation=[0, 0, 0.1], origin_orientation=[0, 0, 0], rotation=[1, 0, 0]),
        URDFLink(name="gripper", origin_translation=[0, 0, 0.05], origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
        OriginLink()  # End effector
    ])
    print(f"✓ Manual chain created with {len(my_chain.links)} links")

# Set target joint angles (in radians)
print("\nSetting target angles...")
target_angles = [0, math.radians(30), math.radians(-45), math.radians(60), math.radians(-20), math.radians(0), 0]

# Ensure we have the right number of angles for the chain
if len(target_angles) != len(my_chain.links):
    print(f"Adjusting angles count from {len(target_angles)} to {len(my_chain.links)}")
    target_angles = target_angles[:len(my_chain.links)]
    while len(target_angles) < len(my_chain.links):
        target_angles.append(0)

print(f"Target angles: {[math.degrees(a) for a in target_angles]} degrees")

# Calculate forward kinematics
print("\nCalculating forward kinematics...")
try:
    transformation_matrix = my_chain.forward_kinematics(target_angles)
    end_effector_position = transformation_matrix[:3, 3]
    print(f"✓ End-effector position: [{end_effector_position[0]:.3f}, {end_effector_position[1]:.3f}, {end_effector_position[2]:.3f}] meters")
    
    # Test inverse kinematics
    print("\nTesting inverse kinematics...")
    target_position = [0.1, 0.1, 0.3]  # Simple target position
    ik_angles = my_chain.inverse_kinematics(target_position)
    print(f"✓ IK solution: {[math.degrees(a) for a in ik_angles]} degrees")
    
    # Verify the IK solution
    verify_matrix = my_chain.forward_kinematics(ik_angles)
    verify_position = verify_matrix[:3, 3]
    error = np.linalg.norm(np.array(target_position) - verify_position)
    print(f"✓ Position error: {error:.6f} meters")
    
except Exception as e:
    print(f"✗ Kinematics error: {e}")
    target_angles = [0] * len(my_chain.links)
    transformation_matrix = my_chain.forward_kinematics(target_angles)
    end_effector_position = transformation_matrix[:3, 3]

# Visualization
print("\nCreating visualization...")
try:
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the robot
    my_chain.plot(target_angles, ax, target=end_effector_position)
    
    # Customize the plot
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('SO100 Robot Arm Visualization')
    
    # Set reasonable limits
    limit = 0.5
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(0, limit)
    
    # Add grid
    ax.grid(True)
    
    print("✓ Showing plot...")
    plt.show()
    
except Exception as e:
    print(f"✗ Visualization error: {e}")
    print("You might need to install additional packages for visualization.")

print("\nDemo completed!")