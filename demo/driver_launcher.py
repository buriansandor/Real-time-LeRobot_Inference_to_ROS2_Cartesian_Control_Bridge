# test_cartesian.py
import time
from so100.smart_so100 import SmartSO100

# Init
robot = SmartSO100(port="COM5", urdf_path="so100.urdf")

# 1. Go to Home position (e.g., up high)
robot.move_to_xyz(0.2, 0.0, 0.3, duration=2.0)
time.sleep(2.5)

# 2. Move forward
robot.move_to_xyz(0.3, 0.0, 0.1, duration=1.5)