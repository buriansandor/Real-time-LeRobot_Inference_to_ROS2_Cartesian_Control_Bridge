# ROS Bridge Package

This package provides ROS/ROS2 integration for the SO100 robotic arm system.

## 📁 Contents

### `src/so100_teleoperation/`
Complete ROS2 teleoperation system for SO100 leader-follower control.

**Key Features:**
- Real-time position feedback and error detection
- Emergency stop and safety monitoring  
- Modular node architecture
- Performance statistics and logging
- Future MoveIt! integration ready

**Components:**
- `so100_leader_node.py` - Leader robot pose publishing
- `so100_follower_node.py` - Follower robot control with feedback
- `so100_teleop_coordinator.py` - Safety monitoring and coordination
- Launch files and configuration

## 🚀 Quick Start

### ROS2 Teleoperation (Recommended)

```bash
cd package/ros_bridge/src/so100_teleoperation

# Build ROS2 package
colcon build --packages-select so100_teleoperation

# Launch complete teleoperation system
ros2 launch so100_teleoperation so100_teleoperation.launch.py \
    leader_port:=COM5 follower_port:=COM4
```

**Benefits over direct serial control:**
- ✅ Automatic position drift detection and correction
- ✅ Real-time error monitoring and emergency stops
- ✅ Complete system visibility with ROS2 tools
- ✅ Modular architecture for easy extension
- ✅ Professional robotics-grade reliability

## 📋 Requirements

- ROS2 Humble (Windows/Linux)
- SO100 robotic arms with calibrated systems
- Serial port access

## 🔗 Related

- Main teleoperation scripts: `../scripts/`
- SO100 drivers: `../drivers/SO100_Robot/`
- Configuration: `../drivers/SO100_Robot/config/`

For detailed setup and usage instructions, see:
**[src/so100_teleoperation/README.md](src/so100_teleoperation/README.md)**