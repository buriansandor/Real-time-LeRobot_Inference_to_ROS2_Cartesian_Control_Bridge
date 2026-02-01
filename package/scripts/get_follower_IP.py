#!/usr/bin/env python3
"""
GET FOLLOWER IP ADDRESS

Created by Sandor Burian, with assistance of Copilot.
Reads the Follower robot's IP address from 'follower_ip.txt' from the configuration directory or enter it manually.
"""
import os
from pathlib import Path
import sys
import time
import socket
import re
import errno
import traceback


# --- PATHS ---
current_script_dir = Path(__file__).parent.resolve()
package_dir = current_script_dir.parent
config_dir = package_dir / "config"
follower_ip_file = config_dir / "follower_ip.txt"

# --- FUNCTION TO GET FOLLOWER IP ---
def get_follower_ip(config_dir=config_dir, follower_ip_file=follower_ip_file):
    """
    Reads the Follower robot's IP address from 'follower_ip.txt' or prompts the user to enter it manually.
    Returns the IP address as a string.
    """
    ip_address = None

    # Try to read the IP address from the file
    try:
        with open(follower_ip_file, "r", encoding="utf-8") as f:
            ip_address = f.read().strip()
            # Validate the IP address format
            if not re.match(r"^\d{1,3}(\.\d{1,3}){3}$", ip_address):
                print(f"[WARNING] Invalid IP address format in {follower_ip_file}.")
                ip_address = None
    except FileNotFoundError:
        print(f"[INFO] {follower_ip_file} not found.")
    except Exception as e:
        print(f"[ERROR] Could not read {follower_ip_file}: {e}")

    # If not found or invalid, prompt the user to enter it manually
    while not ip_address:
        ip_address = input("Enter the Follower robot's IP address (e.g., 192.168.1.100): ").strip()
        # Validate the IP address format
        if not re.match(r"^\d{1,3}(\.\d{1,3}){3}$", ip_address):
            print("[WARNING] Invalid IP address format. Please try again.")
            ip_address = None

    # Save the valid IP address to the file
    try:
        with open(follower_ip_file, "w", encoding="utf-8") as f:
            f.write(ip_address)
        print(f"[INFO] Follower IP address {ip_address} saved to {follower_ip_file}.")
    except Exception as e:
        print(f"[ERROR] Could not write to {follower_ip_file}: {e}")

    return ip_address