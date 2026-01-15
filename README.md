# Real-time LeRobot Inference to ROS2 Cartesian Control Bridge
<!--
Created by Sandor Burian with the help of GitHub Copilot (Claude Sonnet 4)
-->

[![Pylint](https://github.com/buriansandor/Real-time-LeRobot_Inference_to_ROS2_Cartesian_Control_Bridge/actions/workflows/pylint.yml/badge.svg)](https://github.com/buriansandor/Real-time-LeRobot_Inference_to_ROS2_Cartesian_Control_Bridge/actions/workflows/pylint.yml)
[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![License: MPL 2.0](https://img.shields.io/badge/License-MPL_2.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
[![Code style: pylint](https://img.shields.io/badge/code%20style-pylint-blue)](https://pylint.pycqa.org/)

<!-- not yet
[![Model on HF](https://huggingface.co/datasets/huggingface/badges/resolve/main/model-on-hf-sm-dark.svg)]()
[![Paper page](https://huggingface.co/datasets/huggingface/badges/resolve/main/paper-page-sm-dark.svg)]()-->

----
[Hungarian Readme](documentation_translations/hu-HU/README.md)
----



> **Real-time Robot-to-Robot Control Bridge with Multiple Operation Modes**
>
> This project creates a sophisticated connector between Hugging Face's LeRobot ecosystem and ROS2, demonstrated using two open-source robotic arms: the **HF SO100** (leader) and the **Annin AR4** (follower).
>
> ## 🎯 **Project Goals:**
> - Enable real-time control of AR4 using SO100 as leader arm
> - Make HFSO100 trained models directly usable on AR4 
> - Support multiple operation modes: hardware with hard-coded kinematics, hardware with URDF-based kinematics, and full simulation
> - Provide robust cross-platform compatibility (Windows, macOS, Linux)
> - Implement comprehensive safety limits and workspace mapping

## 🚀 **Quick Start**

### **Prerequisites**
- Python 3.7+ (recommended: Python 3.11)
- Git
- Compatible with Windows, macOS, and Linux

### **1. Installation**
```bash
git clone https://github.com/buriansandor/HFSO100_to_AnninAR4_connector.git
cd HFSO100_to_AnninAR4_connector
python setup.py
```

The setup script automatically:
- Creates a Python virtual environment (`.venv`)
- Installs all required dependencies including ikpy, lerobot, matplotlib
- Generates `requirements.txt` with current package versions

### **2. Launch the System**
```bash
cd demo
python launcher.py
```

The launcher provides **three operation modes**:

#### **Mode 1: Hardware with Hard-coded Kinematics** 
- Direct hardware control using pre-defined kinematic transformations
- Fastest performance, minimal computational overhead
- Requires SO100 connected via serial (typically COM5)

#### **Mode 2: Hardware with URDF-based Kinematics**
- Advanced mode using URDF files for both robots
- Dynamic inverse kinematics with workspace validation
- Full safety boundary checking (z-limits, radial limits)
- Supports different robot configurations

#### **Mode 3: Simulation Mode**
- Complete simulation without hardware requirements
- Ideal for development, testing, and demonstration
- Full 3D visualization with real-time kinematic feedback

#### **Cartrasian Mode**
> read about setup of carttasian in: [SETUP_GUIDE](SETUP_GUIDE.md)

## 📁 **Project Structure**

```
HFSO100_to_AnninAR4_connector/
├── .venv/                     # Python virtual environment
├── demo/                      # Main application entry point
│   ├── launcher.py           # Multi-mode system launcher
│   ├── SO100/               # SO100 robot configuration
│   │   ├── URDF/           # Robot URDF files
│   │   │   ├── so100.urdf  # SO100 kinematic model
│   │   │   └── so101.urdf  # Alternative configuration
│   │   ├── so100_driver.py # Hardware interface driver
│   │   └── callibration_data.csv # Motor calibration data
│   ├── AR4/                 # AR4 robot configuration  
│   │   └── URDF/
│   │       └── ar4.urdf    # AR4 kinematic model
│   └── ROS/                # ROS integration examples
├── kinematics/             # Kinematic transformation engines
│   ├── kinematics_bridge.py          # Basic hard-coded kinematics
│   ├── URDF_based_kinematics_bridge.py # Advanced URDF kinematics
│   └── urdf_based_kinematics_bridge.py # Alternative URDF implementation
├── visualisation/          # 3D visualization and UI
│   ├── main_viz.py        # Real-time 3D visualization system
│   └── so100_viz.py       # SO100-specific visualization
├── setup.py              # Cross-platform environment setup
├── requirements.txt      # Python dependencies
└── README.md            # This file
```

## ⚙️ **System Architecture**

### **Core Components:**

1. **Launcher System (`demo/launcher.py`)**
   - Automatic hardware detection
   - Mode selection interface
   - Cross-platform port management
   - Error handling and validation

2. **Robot Drivers:**
   - **SO100 Driver** (`SO100/so100_driver.py`): Serial communication with STS3215 servos
   - **Calibration System**: Load motor-specific adjustments from CSV data
   - **URDF Support**: Dynamic loading of robot models

3. **Kinematic Engines:**
   - **Basic Bridge**: Fast hard-coded transformations
   - **URDF Bridge**: Dynamic IK/FK with safety validation
   - **Workspace Mapping**: Scaling, rotation, translation between robot coordinate systems

4. **Visualization System:**
   - **Real-time 3D Plotting**: Live robot arm positions
   - **Safety Indicators**: Visual feedback for workspace limits
   - **Multi-mode Support**: Hardware and simulation rendering

### **Key Features:**

#### **🔒 Safety Systems**
- **Cylindrical Workspace Validation**: Enforces z-limits (table level protection) and radial boundaries
- **IK Solver Verification**: Ensures target positions are physically reachable
- **Real-time Limit Monitoring**: Visual and programmatic feedback when approaching boundaries
- **Automatic Position Clamping**: Safe fallback when limits are exceeded

#### **🔧 Hardware Integration**
- **Serial Communication**: STS3215 servo protocol implementation
- **Motor Calibration**: Per-motor offset compensation from calibration files
- **Port Auto-detection**: Automatic discovery of connected SO100 devices
- **Cross-platform Compatibility**: Windows (COM), macOS/Linux (ttyUSB) support

#### **📊 URDF-based Kinematics**
- **Dynamic Model Loading**: Support for different robot configurations
- **Inverse Kinematics**: ikpy-based solver for precise positioning
- **Forward Kinematics**: Validation and error calculation
- **Multi-robot Support**: Different base link conventions (SO100: "base", AR4: "base_link")

## 🎮 **Usage Guide**

### **Starting the System:**
```bash
# Navigate to demo directory
cd demo

# Launch the multi-mode selector
python launcher.py

# Select your preferred mode:
# 1 - Hardware with hard-coded kinematics (fastest)
# 2 - Hardware with URDF-based kinematics (most accurate) 
# 3 - Simulation mode (no hardware required)
```

### **Mode-Specific Instructions:**

#### **Hardware Modes (1 & 2):**
1. Connect SO100 robot to USB port
2. Ensure proper power supply to robot
3. Launcher will auto-detect port (typically COM5 on Windows)
4. Calibration data automatically loaded from `SO100/callibration_data.csv`

#### **Simulation Mode (3):**
- No hardware required
- Full 3D visualization active
- Interactive testing environment
- Ideal for development and demonstrations

### **Real-time Operation:**
- **3D Visualization Window**: Shows both leader and follower robot positions
- **Title Bar Status**: Displays current mode and safety limit status
- **Interactive Control**: Move SO100 leader arm to control AR4 follower
- **Safety Feedback**: Visual indicators when approaching workspace limits

## 🔧 **Technical Specifications**

### **Supported Robots:**
- **Leader**: HF SO100 (6-DOF, STS3215 servos, compact workspace)
- **Follower**: Annin AR4 (6-DOF, larger workspace, ROS2 compatible)

### **Communication Protocols:**
- **SO100**: Serial over USB, STS3215 servo protocol
- **AR4**: ROS2 integration (planned/configurable)

### **Dependencies:**
- **ikpy 3.4.2**: Inverse kinematics solver with URDF support
- **lerobot 0.4.2**: Hugging Face robotics ecosystem integration
- **matplotlib**: Real-time 3D visualization
- **numpy**: Numerical computing for transformations
- **pyserial**: Serial communication for hardware interface

### **Workspace Mapping:**
- **Scale Factor**: 1.8x (AR4 is 1.8x larger workspace than SO100)
- **Safety Margins**: Configurable dead zones and maximum reach limits
- **Coordinate Systems**: Automatic transformation between robot bases
- **Real-time Validation**: Continuous safety boundary checking

## 🛠️ **Development**

### **Environment Setup:**
```bash
# Activate virtual environment
# Windows PowerShell:
.venv\Scripts\Activate.ps1

# macOS/Linux:
source .venv/bin/activate

# Install additional packages
pip install <package-name>

# Update requirements
pip freeze > requirements.txt
```

### **Adding New Features:**
1. **New Kinematic Bridge**: Extend base classes in `kinematics/`
2. **Robot Support**: Add URDF files and driver configurations
3. **Visualization Enhancements**: Modify `visualisation/main_viz.py`
4. **Safety Features**: Update workspace validation in kinematic bridges

### **Testing:**
- **Hardware Testing**: Use Mode 1 for basic functionality, Mode 2 for advanced features
- **Simulation Testing**: Mode 3 provides safe environment for development
- **Cross-platform**: Test setup.py on different operating systems

## 🚨 **Safety Considerations**

### **Hardware Safety:**
- Always ensure proper power supply connections before operation
- Verify workspace clearance before enabling hardware modes
- Monitor motor temperatures during extended operation
- Keep emergency stop accessible

### **Software Safety:**
- URDF-based mode (Mode 2) includes comprehensive safety boundary checking
- Automatic position clamping prevents dangerous movements
- Real-time limit monitoring with visual feedback
- Calibration data validation prevents motor damage

### **Workspace Limits:**
- **Z-axis**: Minimum 0.05m (table level protection), Maximum 0.90m
- **Radial**: Minimum 0.15m (inner deadzone), Maximum 0.60m
- **Safety Factor**: 90% of maximum reach for additional margin

## 📊 **Performance Notes**

- **Mode 1 (Hard-coded)**: ~100 Hz update rate, minimal latency
- **Mode 2 (URDF-based)**: ~50-80 Hz update rate, includes validation overhead  
- **Mode 3 (Simulation)**: ~60 Hz visualization, no hardware limitations
- **Memory Usage**: ~50-100 MB depending on mode and visualization complexity

## 🤝 **Contributing**

1. Fork the repository
2. Create feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📜 **License**

This project is licensed under the Mozilla Public License Version 2.0 - see the `LICENSE` file for details.

## 🔗 **Related Projects**

- [Hugging Face LeRobot](https://github.com/huggingface/lerobot)
- [SO100 Robot Documentation](https://github.com/huggingface/lerobot/tree/main/lerobot/configs/robot/so100.yaml)
- [Annin AR4 Robot](https://www.anninrobotics.com/)
- [ikpy - Inverse Kinematics Python](https://github.com/Phylliade/ikpy)

## 📞 **Support**

For questions, issues, or contributions:
- GitHub Issues: Report bugs and request features
- Discussions: General questions and community support
- Documentation: Comprehensive guides in project wiki

---
*Created with <3 by Sandor Burian with the assistance of GitHub Copilot*
