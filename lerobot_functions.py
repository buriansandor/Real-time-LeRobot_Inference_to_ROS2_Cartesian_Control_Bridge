#!/usr/bin/env python3
 
"""
Created by Sandor Burian
Help functions to the usage of LeRobot-based robots
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import time
import numpy as np
from matplotlib.widgets import Button
from datetime import datetime

class PortFinder:
    """Class to find available serial port for LeRobot-based robots"""
    def __init__(self):
        pass

    def find_port(self):
        """Find available serial port using pyserial or manually if pyserial is not available"""
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            if ports:
                return ports[0].device
        except Exception as e:
            pass
        
        # Fallback
        print(f"Error finding port with PySerial: {e}")
        print("Falling back to manual method...")
        print("Available ports:")
        for p in ports:
            print(f" - {p.device}\t {p.description}")
        portnumber = input("Enter the port number for your device (e.g., COM5 or /dev/ttyUSB0): ").strip()
        print(f"Using port: {portnumber}")
        return portnumber

    def find_port_with_lerobot(self):
        """Use lerobot's 'lerobot-find-port' to find the serial port"""
        import subprocess
        try:
            subprocess.call(['lerobot-find-port'])
            print("\nConnect again the robot in the same USB port!")
            port = input("Enter the mentioned port number for your device: ").strip()
            print(f"Using port: {port}")
            if port:
                return port
        except Exception as e:
            print(f"Error finding port with LeRobot: {e}")
            print("Falling back to pyserial method...")
        return self.find_port()
    
    @staticmethod
    def find_port():
        """Find available serial port using pyserial or manually if pyserial is not available"""
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            if ports:
                return ports[0].device
        except Exception as e:
            pass
        
        # Fallback
        print(f"Error finding port with PySerial: {e}")
        print("Falling back to manual method...")
        print("Available ports:")
        for p in ports:
            print(f" - {p.device}\t {p.description}")
        portnumber = input("Enter the port number for your device (e.g., COM5 or /dev/ttyUSB0): ").strip()
        print(f"Using port: {portnumber}")
        return portnumber
    
    @staticmethod
    def find_port_with_lerobot():
        """Use lerobot's 'lerobot-find-port' to find the serial port"""
        import subprocess
        try:
            subprocess.call(['lerobot-find-port'])
            print("\nConnect again the robot in the same USB port!")
            port = input("Enter the mentioned port number for your device: ").strip()
            print(f"Using port: {port}")
            if port:
                return port
        except Exception as e:
            print(f"Error finding port with LeRobot: {e}")
            print("Falling back to pyserial method...")
        return PortFinder.find_port()
    
