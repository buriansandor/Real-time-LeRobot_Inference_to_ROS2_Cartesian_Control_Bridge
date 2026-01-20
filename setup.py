#!/usr/bin/env python3
"""
Cross-platform setup script for HFSO100_to_AnninAR4_connector project.
This script handles virtual environment creation and dependency installation
across Windows, macOS, and Linux.

Created by Sandor Burian with the help of GitHub Copilot (Claude Sonnet 4)
"""

import os
import sys
import subprocess
import venv
import platform
from pathlib import Path
from setuptools import setup, find_packages
setup(
    name='universal_robot_control', 
    version='0.1', 
    packages=find_packages(where='package') + ['so100_core'],
    package_dir={
        '': 'package',
        'so100_core': 'package/drivers/SO100_Robot/so100_core'
    },
    install_requires=[
        'numpy',
        'ikpy',
        'matplotlib',
        'lerobot'
    ]
)

def get_venv_path():
    """Get the virtual environment path based on the operating system."""
    if platform.system() == "Windows":
        return Path("venv") / "Scripts" / "python.exe"
    else:
        return Path("venv") / "bin" / "python"


def get_pip_path():
    """Get the pip executable path based on the operating system."""
    if platform.system() == "Windows":
        return Path("venv") / "Scripts" / "pip.exe"
    else:
        return Path("venv") / "bin" / "pip"


def create_virtual_environment():
    """Create a virtual environment in the current directory."""
    print("Creating virtual environment...")
    venv_dir = Path("venv")
    
    if venv_dir.exists():
        print("Virtual environment already exists. Skipping creation.")
        return True
    
    try:
        venv.create("venv", with_pip=True)
        print("[OK] Virtual environment created successfully.")
        return True
    except Exception as e:
        print(f"[ERROR] Failed to create virtual environment: {e}")
        return False


def upgrade_pip():
    """Upgrade pip to the latest version."""
    print("Upgrading pip...")
    pip_path = get_pip_path()
    
    try:
        subprocess.run([str(pip_path), "install", "--upgrade", "pip"], 
                      check=True, capture_output=True, text=True)
        print("[OK] pip upgraded successfully.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to upgrade pip: {e}")
        return False


def install_requirements():
    """Install requirements from requirements.txt if it exists."""
    requirements_file = Path("requirements.txt")
    
    if not requirements_file.exists():
        print("No requirements.txt found. Skipping package installation.")
        return True
    
    print("Installing requirements from requirements.txt...")
    pip_path = get_pip_path()
    
    try:
        subprocess.run([str(pip_path), "install", "-r", "requirements.txt"], 
                      check=True, capture_output=True, text=True)
        print("[OK] Requirements installed successfully.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to install requirements: {e}")
        return False


def install_specific_packages(packages):
    """Install specific packages."""
    if not packages:
        return True
        
    print(f"Installing packages: {', '.join(packages)}")
    pip_path = get_pip_path()
    
    try:
        subprocess.run([str(pip_path), "install"] + packages, 
                      check=True, capture_output=True, text=True)
        print("[OK] Packages installed successfully.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to install packages: {e}")
        return False


def generate_requirements():
    """Generate requirements.txt with current packages."""
    print("Generating requirements.txt...")
    pip_path = get_pip_path()
    
    try:
        result = subprocess.run([str(pip_path), "freeze"], 
                              capture_output=True, text=True, check=True)
        
        with open("requirements.txt", "w") as f:
            f.write(result.stdout)
        
        print("[OK] requirements.txt generated successfully.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to generate requirements.txt: {e}")
        return False


def show_activation_instructions():
    """Show platform-specific activation instructions."""
    print("\n" + "="*50)
    print("VIRTUAL ENVIRONMENT SETUP COMPLETE!")
    print("="*50)
    
    if platform.system() == "Windows":
        print("To activate the virtual environment on Windows:")
        print("  PowerShell: .\\venv\\Scripts\\Activate.ps1")
        print("  CMD:        venv\\Scripts\\activate.bat")
    else:
        print("To activate the virtual environment on Unix/macOS:")
        print("  source venv/bin/activate")
    
    print("\nTo deactivate:")
    print("  deactivate")
    
    print(f"\nPython executable: {get_venv_path()}")
    print(f"Pip executable: {get_pip_path()}")


def main():
    """Main setup function."""
    print("Real-time LeRobot Inference to ROS2 Cartesian Control Bridge - Cross-platform Setup")
    print(f"Platform: {platform.system()} {platform.release()}")
    print(f"Python: {sys.version}")
    print("-" * 50)
    
    # Check if Python version is supported
    if sys.version_info < (3, 7):
        print("[ERROR] Python 3.7 or higher is required.")
        return 1
    
    # Create virtual environment
    if not create_virtual_environment():
        return 1
    
    # Upgrade pip
    if not upgrade_pip():
        print("Warning: Could not upgrade pip, continuing anyway...")
    
    # Install from requirements.txt or install default packages
    if not install_requirements():
        print("Installing default packages (numpy, ikpy)...")
        if not install_specific_packages(["numpy", "ikpy"]):
            return 1
        
        # Generate requirements.txt with installed packages
        if not generate_requirements():
            print("Warning: Could not generate requirements.txt")
    
    show_activation_instructions()
    return 0


if __name__ == "__main__":
    # Only run setup when executed directly (python setup.py), not during pip install
    import os
    if 'SETUPTOOLS_USE_DISTUTILS' in os.environ or len(sys.argv) > 1:
        # Skip setup when being run by setuptools/pip
        setuptools_commands = ['egg_info', 'bdist_wheel', 'build', 'install', 'dist_info', 'develop', 'sdist', 'editable_wheel']
        if any(cmd in sys.argv for cmd in setuptools_commands):
            pass
        else:
            sys.exit(main())
    else:
        sys.exit(main())