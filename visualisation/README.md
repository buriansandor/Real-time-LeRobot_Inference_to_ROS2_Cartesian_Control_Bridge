# Visualisation

## About

> This folder contains visualisation tools and resources for the HFSO100 to AnninAR4 connector project. It provides utilities to display, analyze, and interpret data transformations and connections between the HFSO100 and AnninAR4 systems.

## How to Use
1. **Prerequisites**: Ensure all dependencies are installed as specified in the main project README.
2. **Running Visualisations**: Execute the visualisation scripts in the `venv` from this directory or reference them from other parts of the project.
4. **Configuration**: Check individual script files for configuration options and parameters that can be adjusted to customize the visualisation output.

For detailed usage instructions, refer to the specific visualisation scripts in this folder.

---------------

## Using [so100_viz.py](so100_viz.py)

### Overview
`so100_viz.py` is a Python script for visualizing HFSO100 data structures and transformations.

### Usage
```bash
python so100_viz.py [options]
```

### Basic Examples
```bash
# Run in interactive configuration mode
python so100_viz.py

# Simulation mode with custom baud rate and port
python so100_viz.py --simulation --port COM6 --baud-rate 115200

# Same in shorter format
python so100_viz.py -s -p COM6 -b 115200

# Custom URDF file and motor ID
python so100_viz.py --urdf ../my_robot.urdf --motor-ids 1 2 3 4

# Just simulation
python so100_viz.py --simulation

# get help
python so100_viz.py --help
```
