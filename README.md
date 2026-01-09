# Real-time LeRobot Inference to ROS2 Cartesian Control Bridge
<!--
Created by Sandor Burian with the help of GitHub Copilot (Claude Sonnet 4)
-->

> In this project, we create a connector between Hugging Face's LeRobot  and ROS2. As an example, we use two main open-source robotic arms to demonstrate how it works: the HF SO100 robotic arm and the Annin AR4 robotic arm.
>
> The goals:
> - make it able to use the leader arm of HFSO100 as a real-time leader of AR4
> - make the HFSO100 trained models usable on AR4 directly
>
> To do this, we will use relative workspace mapping, cause the AR4 is much bigger than SO100.

## Installation

This project includes a cross-platform Python setup script that works on Windows, macOS, and Linux.

### Prerequisites

- Python 3.7 or higher
- Git

### Quick Setup

1. **Clone the repository:**
    ```bash
    git clone https://github.com/buriansandor/Real_time_LeRobot_Inference_to_ROS2_Cartesian_Control_Bridge.git
    cd Real-time-LeRobot-Inference-to-ROS2-Cartesian-Control-Bridge
    ```

2. **Run the setup script:**
   ```bash
   python setup.py
   ```

The setup script will automatically:
- Create a virtual environment
- Upgrade pip to the latest version
- Install required dependencies (numpy, ikpy, and their dependencies)
- Generate a `requirements.txt` file

### Manual Setup (Alternative)

If you prefer to set up the environment manually:

1. **Create virtual environment:**
   ```bash
   python -m venv venv
   ```

2. **Activate virtual environment:**
   - **Windows (PowerShell):** `.\venv\Scripts\Activate.ps1`
   - **Windows (CMD):** `venv\Scripts\activate.bat`
   - **macOS/Linux:** `source venv/bin/activate`

3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

### Dependencies

The project uses the following main dependencies:
- **numpy**: Numerical computing library
- **ikpy**: Inverse kinematics library for robotic arm calculations
- **scipy**: Scientific computing (ikpy dependency)
- **sympy**: Symbolic mathematics (ikpy dependency)
- **mpmath**: Multiprecision arithmetic (sympy dependency)

### Development

After setup, activate the virtual environment and start developing:

```bash
# Activate environment (platform-specific)
# Windows: .\venv\Scripts\Activate.ps1
# macOS/Linux: source venv/bin/activate

# Install additional packages if needed
pip install <package-name>

# Update requirements.txt after installing new packages
pip freeze > requirements.txt
```
