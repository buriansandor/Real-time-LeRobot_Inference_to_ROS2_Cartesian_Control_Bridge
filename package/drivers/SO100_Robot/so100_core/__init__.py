# Import the simplified, working driver
from .so100_driver import SO100Robot

# Keep the old import for compatibility
from .robot import SO100Robot as SO100Robot_Old