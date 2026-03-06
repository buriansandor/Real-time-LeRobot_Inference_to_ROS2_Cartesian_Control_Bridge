# AI Context for Development

> ⚠️ **Primary Documentation**: See [llms.txt](llms.txt) for comprehensive system architecture, rules, and guidelines.

## Quick Reference

This project is a **real-time teleoperation bridge** between SO100 leader and AR4 follower robots using ZMQ and Cartesian space control.

### Core Principles (Summary)
- Leader node broadcasts **raw joint angles only** (no kinematics)
- All FK/IK happens on **receiver side**
- Communication in **Cartesian space** (X, Y, Z)
- `RobustPolicy` class maintains LeRobot compatibility
- Non-blocking ZMQ with `CONFLATE` flag for real-time performance

### Key Files
- `llms.txt` - **Read this first!** Complete system rules and architecture
- `package/scripts/zmq_leader_node.py` - Leader publisher
- `package/scripts/zmq_follower_node.py` - Follower subscriber with IK
- `package/drivers/SO100_Robot/so100_core/kinematics.py` - FK/IK implementation

### Additional Project Notes

(Add project-specific context, current tasks, or temporary notes here that don't belong in the main llms.txt documentation)
