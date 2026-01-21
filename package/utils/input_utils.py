#!/usr/bin/env python3
"""
Input utility functions for SO100 Robot configuration
Handles common input prompts for port, calibration file, and URDF path
Created by Sandor Burian based Copilot's code with the help of Google Gemini Pro.
"""


def get_robot_configuration():
    """
    Get robot configuration from user input.
    
    Returns:
        dict: Configuration dictionary containing:
            - 'port': Serial port for robot communication
            - 'calib_file': Calibration file name
            - 'urdf_path': URDF file path
    """
    config = {}
    
    # Get port configuration
    config['port'] = get_port_input()
    
    # Get calibration file
    config['calib_file'] = get_calibration_file_input()
    
    # Get URDF path
    config['urdf_path'] = get_urdf_path_input()
    
    return config


def get_port_input(default_port='COM4'):
    """
    Get port configuration from user input.
    Supports default port, manual input, and automatic detection.
    
    Returns:
        str: Port name (e.g., 'COM4', '/dev/ttyUSB0')
    """
    port = input("Enter the port of the follower arm (e.g., COM4 or /dev/ttyUSB0) or get the port with port detection (detect): ").strip()
    
    if port == '':
        print(f"Setting port to the default: {default_port}")
        port = default_port  # Change this to your follower's port if needed
    elif port.lower() == 'detect':
        port = None
        port = get_port_with_detection()
        if port is None:
            print("Port detection failed. Please set the port manually.")
            port = input("Enter the port of the follower arm (e.g., COM4 or /dev/ttyUSB0): ").strip()
    return port


def get_calibration_file_input(default_file='follower_calibration.csv'):
    """
    Get calibration file name from user input.
    
    Returns:
        str: Calibration file name
    """
    calib_file = input(f"Enter the calibration file name (default: {default_file}): ").strip()
    
    if calib_file == '':
        print(f"Setting calibration file to default: {default_file}")
        calib_file = default_file
    
    return calib_file


def get_urdf_path_input(default_urdf='so100.urdf'):
    """
    Get URDF file path from user input.
    
    Returns:
        str: URDF file path
    """
    urdf_path = input(f"Enter the URDF file path (default: {default_urdf}): ").strip()
    
    if urdf_path == '':
        print(f"Setting URDF path to default: {default_urdf}")
        urdf_path = default_urdf
    
    return urdf_path


def get_port_with_detection(defaultPort='COM4'):
    """
    Get port using automatic detection only.
    This is a convenience function for scripts that prefer automatic detection.
    
    Returns:
        str: Detected port name or None if detection fails
    """
    try:
        import sys
        import os
        # Add the project root directory to Python path
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        if project_root not in sys.path:
            sys.path.insert(0, project_root)
        
        from lerobot_functions import PortFinder
        print("Set the port of robotic arm")
        port = PortFinder.find_port_with_lerobot()
        
    except ImportError:
        print("Warning: PortFinder not available, using manual port configuration.")
        print("Please set LEADER_PORT and FOLLOWER_PORT manually, use 'lerobot-find-port' to find them in a separate console.")
        # Fallback to manual input
        port = input("Enter the port manually: ").strip()
        if port == '':
            port = defaultPort
    return port