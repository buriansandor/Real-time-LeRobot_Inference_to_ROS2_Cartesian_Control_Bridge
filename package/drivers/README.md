# SO100 Robot Driver

High-performance Python driver for SO100 robotic arm with **direct serial communication** and **cartesian coordinate control**.

## 🚀 Features

- **⚡ Ultra-fast performance** - Direct serial protocol bypassing wrapper layers
- **📐 Cartesian coordinates** - Simple XYZ positioning with automatic inverse kinematics  
- **🎯 Precise control** - Motor-by-motor programming with calibrated movements
- **🤖 Full gripper support** - Optimized 8-byte protocol for gripper operations
- **🔧 Auto-calibration** - CSV-based motor offset and direction compensation
- **📊 Real-time feedback** - Position reading and joint angle monitoring

## 📦 Installation

### Prerequisites
```bash
pip install pyserial ikpy numpy
```

### Package Installation
```bash
# Install in development mode
pip install -e .

# Or install from package root
python setup.py install
```

### Hardware Setup
1. Connect SO100 robot via USB/Serial (default: COM4)
2. Ensure STS3215 motors are properly configured
3. Verify power supply and motor IDs (1-6)

## 🎯 Quick Start

### Basic Cartesian Movement
```python
from drivers.SO100_Robot import SO100Robot

# Initialize robot
robot = SO100Robot(port="COM4", config_dir="config")

# Enable motors
robot.torque_enable(True)

# Move to cartesian position (X, Y, Z in meters)
robot.move_to_cartesian(x=0.25, y=0.05, z=0.15, time_ms=400)

# Gripper control
robot.gripper_open()
robot.gripper_close()

# Cleanup
robot.torque_enable(False)
robot.close()
```

### Advanced Joint Control
```python
import math

# Direct joint angle control (radians)
joint_angles = [0.0, math.pi/4, -math.pi/3, 0.0, math.pi/6, 0.0]
robot.move_to_joints(joint_angles, time_ms=500)

# Read current positions
current_angles = robot.get_joint_angles()
print(f"Current joint angles: {current_angles}")

# Individual motor control
robot.set_target_angle(motor_index=0, angle_radians=math.pi/4, move_time_ms=300)
```

## 📋 Configuration

### Required Files (in `config/` directory)

#### `follower_calibration.csv`
Motor calibration parameters:
```csv
ZERO_OFFSETS,2134,2032,1057,2020,897,1840
DIRECTIONS,-1,1,-1,-1,1,1
CALIBRATION_POSE_ADJUSTMENTS,0.0,0.1,-0.95,0.0,0.0,0.0
```

#### `gripper_values.csv`
Gripper position settings:
```csv
GRIPPER_OPEN,1878
GRIPPER_CLOSE,1798
```

#### `so100.urdf`
Robot kinematic model for inverse kinematics calculations.

### Configuration Loading
```python
# Auto-load from config directory
robot = SO100Robot(port="COM4")  # Uses default config/

# Custom config directory
robot = SO100Robot(port="COM4", config_dir="my_config/")

# Check loaded calibration
calib = robot.getCalibrationvalues()
print(f"Offsets: {calib['offsets']}")
print(f"Directions: {calib['directions']}")
```

## 🎮 API Reference

### Core Methods

#### Robot Initialization
```python
SO100Robot(port, config_dir=None)
```
- `port`: Serial port (e.g., "COM4", "/dev/ttyUSB0")
- `config_dir`: Path to configuration files (optional)

#### Cartesian Movement
```python
move_to_cartesian(x, y, z, time_ms=400)
```
- `x, y, z`: Position in meters (workspace coordinates)
- `time_ms`: Movement duration in milliseconds

**Coordinate System:**
```
X: Forward/Backward (+ = forward)
Y: Left/Right (+ = right)  
Z: Up/Down (+ = up)
Origin: Robot base center
```

#### Joint Control
```python
move_to_joints(joint_angles, time_ms=400)
set_target_angle(motor_index, angle_radians, move_time_ms=400)
get_joint_angles()  # Returns current angles
```

#### Gripper Control
```python
gripper_open()   # Open gripper
gripper_close()  # Close gripper
```

#### Motor Management
```python
torque_enable(enable=True)   # Enable/disable all motors
close()                      # Close serial connection
```

## 📊 Performance Optimization

### Speed Settings
```python
# Ultra-fast movements (experienced users)
robot.move_to_cartesian(0.2, 0.1, 0.15, time_ms=300)

# Balanced speed (recommended)
robot.move_to_cartesian(0.2, 0.1, 0.15, time_ms=400)

# Safe/precise movements
robot.move_to_cartesian(0.2, 0.1, 0.15, time_ms=800)
```

### Protocol Details
- **Motor packets**: 7-byte direct STS3215 protocol
- **Gripper packets**: 8-byte specialized protocol  
- **Communication**: 1,000,000 baud rate
- **Motor delays**: 0.05s between motors for synchronization

## 🔧 Examples

### Pick and Place Operation
```python
from drivers.SO100_Robot import SO100Robot
import time

robot = SO100Robot(port="COM4")
robot.torque_enable(True)

# Define positions
pick_pos = [0.25, 0.05, 0.03]
place_pos = [0.25, -0.05, 0.03]
safe_height = 0.15

# Pick sequence
robot.gripper_open()
robot.move_to_cartesian(pick_pos[0], pick_pos[1], safe_height, time_ms=400)
robot.move_to_cartesian(pick_pos[0], pick_pos[1], pick_pos[2], time_ms=400)
robot.gripper_close()
robot.move_to_cartesian(pick_pos[0], pick_pos[1], safe_height, time_ms=400)

# Place sequence  
robot.move_to_cartesian(place_pos[0], place_pos[1], place_pos[2], time_ms=400)
robot.gripper_open()
robot.move_to_cartesian(place_pos[0], place_pos[1], safe_height, time_ms=400)

robot.torque_enable(False)
robot.close()
```

### Trajectory Following
```python
import numpy as np

# Generate circular trajectory
t = np.linspace(0, 2*np.pi, 20)
radius = 0.05
center = [0.25, 0.0, 0.10]

for angle in t:
    x = center[0] + radius * np.cos(angle)
    y = center[1] + radius * np.sin(angle) 
    z = center[2]
    
    robot.move_to_cartesian(x, y, z, time_ms=200)
    time.sleep(0.1)  # Brief pause between points
```

## 🛠 Troubleshooting

### Connection Issues
```python
# Test serial connection
try:
    robot = SO100Robot(port="COM4")
    print("Connection successful!")
except Exception as e:
    print(f"Connection failed: {e}")
```

**Common Connection Problems:**
- **PermissionError (Device not functioning)**: Usually caused by Bluetooth interference
  - **Solution**: Disable Bluetooth on your laptop/PC before connecting robot
  - **Reason**: Bluetooth virtual COM ports can conflict with real serial ports
- **Port busy**: Another application is using the serial port
  - **Solution**: Close all robot programs and VS Code terminals
- **Wrong port**: Selected port doesn't match robot
  - **Solution**: Run `lerobot-find-port` to detect correct port

### Calibration Problems
```python
# Verify calibration loading
calib = robot.getCalibrationvalues()
if all(offset == 2048 for offset in calib['offsets'].values()):
    print("WARNING: Using default calibration - check config files!")
```

### Movement Issues
- **Slow movement**: Check speed=0 for maximum speed
- **Wrong directions**: Verify DIRECTIONS in calibration
- **Position errors**: Check ZERO_OFFSETS values
- **Gripper not working**: Ensure 8-byte protocol format

## 📁 Project Structure

```
drivers/
├── README.md                    # This file
├── SO100_Robot/
│   ├── __init__.py             # Package exports
│   ├── so100_core/             # Core implementation
│   │   ├── robot.py            # Main SO100Robot class
│   │   ├── kinematics.py       # IK calculations
│   │   └── sts3215.py          # Motor driver (legacy)
│   ├── config/                 # Configuration files
│   │   ├── follower_calibration.csv
│   │   ├── gripper_values.csv
│   │   └── so100.urdf
│   └── scripts/                # Demo scripts
│       ├── pick_and_place_fast.py
│       └── README.md
└── utils/                      # Utility functions
    └── input_utils.py          # Configuration helpers
```

## 🔬 Technical Details

### Serial Protocol Specification

#### Motor Control Packet (7 bytes)
```
[0x2A, pos_low, pos_high, time_low, time_high, speed_low, speed_high]

pos: 0-4095 (12-bit position)
time: 0-65535 milliseconds  
speed: 0 = maximum speed
```

#### Gripper Control Packet (8 bytes)
```
[0x2A, 0x00, pos_low, pos_high, 0x00, 0x00, 0x00, 0x00]

pos: Raw gripper position value
Fixed format with padding zeros
```

### Coordinate Transformations
1. **Cartesian → Joint angles**: IKPy inverse kinematics
2. **Joint angles → Motor raw**: Calibrated conversion with offsets/directions
3. **Motor raw → Serial packets**: STS3215 protocol formatting

## 🚨 Safety Guidelines

⚠️ **Important Safety Notes**

- Always test movements at slow speeds initially
- Keep workspace clear during operation
- Have emergency stop capability ready
- Verify calibration before production use
- Monitor for overheating during extended operation

## 📞 Support

For issues and questions:
- Check `scripts/README.md` for demo examples
- Verify configuration files are properly formatted
- Ensure hardware connections are secure
- Test with provided example scripts first

---

**Optimized for maximum performance and reliability**  
**Direct STS3215 serial communication**  
**Created by Sandor Burian with Copilot assistance**