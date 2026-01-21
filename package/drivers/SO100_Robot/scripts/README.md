# SO100 Robot Scripts

This directory contains demonstration scripts and examples for controlling the SO100 robotic arm.

## Scripts Overview

### `pick_and_place_fast.py`
**Fast pick and place demonstration** - Optimized version with direct serial communication for maximum performance.

**Features:**
- ⚡ **High-speed operation** - 400ms movements (84% faster than original)
- 🎯 **Direct motor control** - Bypasses wrapper layers for optimal performance
- 🤖 **Proper gripper handling** - Uses original 8-byte protocol
- 📐 **Cartesian coordinates** - Simple XYZ positioning with inverse kinematics
- 🔧 **Auto-calibration** - Loads motor offsets, directions, and adjustments

### `calibrate_leader_candle.py`
**Leader robot candle position calibration** - Essential first step for teleoperation setups.

**Purpose:**
- 🕯️ **Candle calibration** - Sets leader robot to perfect vertical "candle" position
- 📊 **Zero point definition** - Defines mathematical [0,0,0,0,0] motor positions  
- 🎯 **Leader preparation** - Prepares leader robot for teleoperation mode
- 💾 **Auto-save** - Saves calibration to `SO100leader_to_cartesian_calibration.csv`

**⚠️ IMPORTANT for Teleoperation:**
This script **must be run first** before using any leader-follower teleoperation setup. It defines the reference position for the leader robot.

**Usage:**
```bash
python calibrate_leader_candle.py
```

**Steps:**
1. Connect leader robot to detected port
2. Robot motors will be released (torque disabled)
3. Manually position robot in perfect "candle" pose:
   - All joints straight and vertical
   - Gripper pointing upward
   - This becomes the mathematical zero point
4. Press ENTER to record calibration values
5. Values automatically saved for teleoperation use

**Troubleshooting:**
- **Connection fails**: Disable Bluetooth on laptop to avoid port conflicts
- **Wrong values (2048)**: Robot is in simulation mode, check connection

## Requirements

- **Hardware:** SO100 Robotic Arm with STS3215 motors
- **Connection:** Serial port (default: COM4)
- **Python:** 3.7+
- **Dependencies:** `serial`, `ikpy`, `numpy`, `csv`

## Configuration

The robot requires calibration files in the `../config/` directory:

- `follower_calibration.csv` - Motor calibration (offsets, directions, adjustments)
- `gripper_values.csv` - Gripper open/close positions
- `so100.urdf` - Robot kinematic model

## Usage

### Quick Start
```bash
# Run pick and place demo
python pick_and_place_fast.py

# Follow prompts for:
# - Serial port (default: COM4)
# - Calibration file (default: follower_calibration.csv) 
# - URDF path (default: so100.urdf)
```

### Interactive Controls
1. **Enter** - Use defaults for quick start
2. **x** - Exit demo at prompt
3. **x** - Release motors and exit

### Example Movement Sequence
```
1. Open gripper
2. Move to pick position (safe height)
3. Descend to object
4. Close gripper
5. Lift object
6. Move to place position
7. Open gripper
8. Retreat to safe height
```

## Configuration Details

### Motor Calibration Format
```csv
ZERO_OFFSETS,2134,2032,1057,2020,897,1840
DIRECTIONS,-1,1,-1,-1,1,1
CALIBRATION_POSE_ADJUSTMENTS,0.0,0.1,-0.95,0.0,0.0,0.0
```

### Gripper Values Format
```csv
GRIPPER_OPEN,1878
GRIPPER_CLOSE,1798
```

## Performance Settings

### Timing Parameters
- **Movement time:** 400ms (fast), 500ms (balanced)
- **Motor delays:** 0.05s between motors (original reliable timing)
- **Gripper wait:** 1.0s for proper operation

### Coordinate System
```
X: Forward/Backward (+ = forward)
Y: Left/Right (+ = right)
Z: Up/Down (+ = up)

Example positions:
PICK_POS = [0.25, 0.05, 0.03]   # Forward, right, low
PLACE_POS = [0.25, -0.05, 0.03] # Forward, left, low
SAFE_Z = 0.15                   # Safe height
```

## Technical Details

### Serial Protocol
- **Baud rate:** 1,000,000
- **Protocol:** STS3215 servo communication
- **Motor control:** Direct 7-byte position packets
- **Gripper control:** Specialized 8-byte protocol

### Motor Commands (7 bytes)
```
[0x2A, pos_low, pos_high, time_low, time_high, speed_low, speed_high]
```

### Gripper Commands (8 bytes)
```
[0x2A, 0x00, pos_low, pos_high, 0x00, 0x00, 0x00, 0x00]
```

## Troubleshooting

### Common Issues

**Robot moves slowly:**
- Check serial connection
- Verify speed=0 for maximum speed
- Ensure direct protocol (no wrapper)

**Gripper doesn't move:**
- Verify 8-byte protocol format
- Check motor ID 6
- Confirm gripper calibration values

**Connection failed:**
- Check COM port in Device Manager
- Verify cable connections
- Try different baud rates

**Import errors:**
- Install missing packages: `pip install pyserial ikpy numpy`
- Verify package structure
- Check Python path

### Debug Information
The script provides detailed debug output:
```
[DEBUG] Config directory: path
[DEBUG] Calibration values: offsets, directions, adjustments
[DEBUG] Gripper packet: [42, 86, 7, 244, 1, 0, 0, 0]
```

## Safety Notes

⚠️ **Always test in safe environment**
- Keep clear workspace around robot
- Have emergency stop ready
- Test movements at slow speeds first
- Verify calibration before operation

## Development

### Adding New Scripts
1. Import from `drivers.SO100_Robot import SO100Robot`
2. Use configuration utilities: `from utils.input_utils import get_robot_configuration`
3. Follow existing patterns for serial communication
4. Test thoroughly with proper calibration

### Performance Optimization
- Use direct serial protocol (avoid wrappers)
- Minimize delays between commands
- Use appropriate movement times
- Consider motor synchronization needs

---

**Created by:** Sandor Burian with Copilot assistance  
**Optimized for:** Maximum performance and reliability  
**Protocol:** Direct STS3215 serial communication