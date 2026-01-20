#!/usr/bin/env python3
"""
Input utility functions for SO100 Robot configuration
Handles common input prompts for port, calibration file, and URDF path
Created by Copilot based on Sandor Burian's code with the help of Google Gemini Pro.
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


def get_port_input():
    """
    Get port configuration from user input.
    Supports default port, manual input, and automatic detection.
    
    Returns:
        str: Port name (e.g., 'COM4', '/dev/ttyUSB0')
    """
    port = input("Enter the port of the follower arm (e.g., COM4 or /dev/ttyUSB0) or get the port with port detection (detect): ").strip()
    
    if port == '':
        print("Setting port to the default: COM4")
        port = 'COM4'  # Change this to your follower's port if needed
    elif port.lower() == 'detect':
        port = None
        port = get_port_with_detection()
        if port is None:
            print("Port detection failed. Please set the port manually.")
            port = input("Enter the port of the follower arm (e.g., COM4 or /dev/ttyUSB0): ").strip()
    return port


def get_calibration_file_input():
    """
    Get calibration file name from user input.
    
    Returns:
        str: Calibration file name
    """
    calib_file = input("Enter the calibration file name (default: follower_calibration.csv): ").strip()
    
    if calib_file == '':
        print("Setting calibration file to default: follower_calibration.csv")
        calib_file = 'follower_calibration.csv'
    
    return calib_file


def get_urdf_path_input():
    """
    Get URDF file path from user input.
    
    Returns:
        str: URDF file path
    """
    urdf_path = input("Enter the URDF file path (default: so100.urdf): ").strip()
    
    if urdf_path == '':
        print("Setting URDF path to default: so100.urdf")
        urdf_path = "so100.urdf"
    
    return urdf_path


def get_port_with_detection():
    """
    Get port using automatic detection only.
    This is a convenience function for scripts that prefer automatic detection.
    
    Returns:
        str: Detected port name or None if detection fails
    """
    try:
        from lerobot_functions import PortFinder
        print("Set the port of leader arm")
        port = PortFinder.find_port_with_lerobot()
    except ImportError:
        print("Warning: PortFinder not available, using manual port configuration.")
        print("Please set LEADER_PORT and FOLLOWER_PORT manually, use 'lerobot-find-port' to find them in a separate console.")
        # Fallback to manual input
        port = input("Enter the port manually: ").strip()
        if port == '':
            port = 'COM4'