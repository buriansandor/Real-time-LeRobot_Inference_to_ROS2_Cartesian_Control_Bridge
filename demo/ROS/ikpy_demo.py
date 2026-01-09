#!/usr/bin/env python3
 
"""
Created by Sandor Burian with the help of Gemini3 PRO and GitHub Copilot (Claude Sonnet 4)
IKpy demo for SO100 robot arm using URDF file
"""

import ikpy.chain
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

my_chain = ikpy.chain.Chain.from_urdf_file(
    "../SO100/URDF/so100.urdf",
    base_elements=["base"]  # Specify the correct base element name
)
print("--- SO-100 URDF loaded ---")
print(f"Number of joints: {len(my_chain.links)}")

print("\nJoints:")
for i, link in enumerate(my_chain.links):
    print(f"{i}. {link.name}")

print("\n\nIn the case of SO100 it should be: [Base(fix), Shoulder_Pan, Shoulder_Lift, Elbow_Flex, Wrist_Flex, Wrist_Roll, Gripper]")
print("--------------------------")
# 2. set the target angles for each joint as SO100 is a 6 DOF robot
# [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]

target_angles = [
    0,                  # Base (fixed)
    math.radians(45),   # Shoulder Pan 
    math.radians(-20),  # Shoulder Lift 
    math.radians(-45),  # Elbow Flex 
    math.radians(-30),  # Wrist Flex 
    math.radians(0),    # Wrist Roll 
    0                   # Gripper (no change in TCP position, as this is just the gripper's angle) 
]

# Ensure the target_angles list matches the number of links in the chain
if len(target_angles) < len(my_chain.links):
    target_angles += [0] * (len(my_chain.links) - len(target_angles))

# 3. Calculate the position (just for verification)
real_frame = my_chain.forward_kinematics(target_angles)
print("Calculated end-effector position: ", real_frame[:3, 3])

# 4. VISUALIZATION
# Create a 3D figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Draw the robot in the given pose
my_chain.plot(target_angles, ax, target=real_frame[:3, 3])

# Set the limits so everything is visible (in meters)
ax.set_xlim(-0.3, 0.3)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(0, 0.4)

# Add labels and title
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('SO100 Robot Arm - IKpy Demo')

print("Plotting the robot...")
plt.show()
print("The plot should have appeared!")