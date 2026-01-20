#!/usr/bin/env python3
"""
Example script showing different ways to use the input utility functions
for SO100 Robot configuration.
Created by Copilot based on Sandor Burian's code.
"""


import sys
import os
import time
from pathlib import Path
import ikpy.chain

# Add the package root to Python path 
script_dir = Path(__file__).parent
package_root = script_dir.parent.parent.parent
sys.path.insert(0, str(package_root))

from drivers.SO100_Robot import SO100Robot
from utils.input_utils import get_robot_configuration
from utils.input_utils import get_port_input, get_calibration_file_input, get_urdf_path_input, get_port_with_detection


def example_full_configuration():
    """Example using the full configuration function."""
    print("=== Example 1: Full Configuration ===")
    config = get_robot_configuration()
    print(f"Configuration received:")
    print(f"  Port: {config['port']}")
    print(f"  Calibration file: {config['calib_file']}")
    print(f"  URDF path: {config['urdf_path']}")
    
    # Initialize robot with configuration
    robot = SO100Robot(port=config['port'], config_dir="config")
    return robot


def example_individual_inputs():
    """Example using individual input functions."""
    print("\n=== Example 2: Individual Inputs ===")
    
    # Get each configuration separately
    port = get_port_input()
    calib_file = get_calibration_file_input()
    urdf_path = get_urdf_path_input()
    
    print(f"Individual inputs received:")
    print(f"  Port: {port}")
    print(f"  Calibration file: {calib_file}")
    print(f"  URDF path: {urdf_path}")
    
    # Initialize robot
    robot = SO100Robot(port=port, config_dir="config")
    return robot


def example_auto_detection():
    """Example using automatic port detection."""
    print("\n=== Example 3: Auto Detection ===")
    
    # Try automatic port detection
    port = get_port_with_detection()
    
    if port is None:
        print("Auto detection failed, falling back to manual input...")
        port = get_port_input()
    
    # Use defaults for other settings
    calib_file = "follower_calibration.csv"
    urdf_path = "URDF/so100.urdf"
    
    print(f"Auto detection result:")
    print(f"  Port: {port}")
    print(f"  Calibration file: {calib_file} (default)")
    print(f"  URDF path: {urdf_path} (default)")
    
    # Initialize robot
    robot = SO100Robot(port=port, config_dir="config")
    return robot


def main():
    """Main function demonstrating different configuration methods."""
    print("SO100 Robot Configuration Examples")
    print("This script shows different ways to configure your robot.")
    
    method = input("\nChoose configuration method (1=full, 2=individual, 3=auto): ").strip()
    
    try:
        if method == "1":
            robot = example_full_configuration()
        elif method == "2":
            robot = example_individual_inputs()
        elif method == "3":
            robot = example_auto_detection()
        else:
            print("Invalid choice, using full configuration...")
            robot = example_full_configuration()
        
        print(f"\nRobot initialized successfully!")
        print(f"Robot port: {robot.port}")
        
        # Test basic functionality
        test = input("\nTest robot connection? (y/n): ").strip().lower()
        if test == 'y':
            print("Testing robot connection...")
            robot.torque_enable(True)
            print("Motors enabled successfully!")
            
            input("Press Enter to disable motors and exit...")
            robot.torque_enable(False)
            print("Motors disabled. Goodbye!")
        else:
            print("Skipping test. Goodbye!")
            
    except Exception as e:
        print(f"Error: {e}")
        print("Please check your configuration and try again.")


if __name__ == "__main__":
    main()