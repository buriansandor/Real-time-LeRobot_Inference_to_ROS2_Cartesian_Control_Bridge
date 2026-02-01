# Real-time LeRobot → ROS2 Cartesian Control — Packaged Module

This folder contains the packaged subset of the repository intended for distribution
and reuse. It bundles the runtime modules and example scripts used to bridge
real-time LeRobot inference and ROS2 Cartesian control.

## Summary

- **Purpose:** Provide reusable Python modules for drivers, kinematics, LeRobot
	interface, ROS2 bridging and visualization taken from the full repository.
- **Intended use:** Install as a package (editable or normal) and run the
	example scripts from `package/scripts` or import the modules in your own
	project.

## Quick install (from repository root)

1. Create and activate a virtual environment:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
```

2. Install runtime requirements and the package in editable mode:

```powershell
pip install -r requirements.txt
pip install -e .
```

Notes:
- If you only need the packaged modules, installing from the repository root
	will pick up the package metadata defined by the project `setup.py`.

## Quickstart

- Important scripts are in `package/scripts/`
### Important nodes

- zmq_leader  
    Starts a ZeroMQ publisher that advertises Cartesian setpoints / control commands for followers. Use to simulate or drive multiple followers from one source.  

- zmq_follower  
    Subscribes to a zmq_leader, converts incoming setpoints to the local robot interface and publishes to the ROS2 control topics. Use on robot-side machines.  

- zmq_to_http  
    Bridges ZeroMQ messages to an HTTP endpoint (REST/WebSocket) for web UIs or remote dashboards. Configure the ZMQ source and HTTP server port.  

- zmq_hybrid  
    Hybrid node that can act as both leader and follower and optionally expose an HTTP bridge. Useful for gateway or proxy deployments.  

See the scripts in package/scripts/ for full CLI options and runtime examples.

## Package structure

- `drivers/` — robot drivers and low-level interfaces
- `kinematics/` — kinematics helpers and bridges
- `lerobot_interface/` — LeRobot specific interface code
- `ros_bridge/` — ROS/ROS2 bridging utilities
- `scripts/` — example and utility scripts (quick_test, calibration helpers)
- `utils/` — small helper utilities
- `visualisation/` — visualization tools and demos

For full context and architecture, see the main project README at
[README.md](../README.md).

## Development

- Follow the root `SETUP_GUIDE.md` for development environment and hardware
	setup instructions.
- Contribute changes via pull requests against the main repository.

## License

This repository is covered by the project's top-level `LICENSE` file.
See [LICENSE](../LICENSE) for details.
