#!/usr/bin/env python3
"""
Universal Robot Teleoperation Launcher

Launch any robot as leader with any robot as follower.
Standard ROS2 compatible communication between different robot types.

Usage Examples:
  # SO100 leader with SO100 follower (classic)
  python launch_teleop.py --leader_robot SO100 --leader_port COM3 --follower_robot SO100 --follower_port COM4
  
  # SO100 leader with UR5 follower 
  python launch_teleop.py --leader_robot SO100 --leader_port COM3 --follower_robot UR5 --follower_port /dev/ttyUSB0
  
  # UR10 leader with SO100 follower
  python launch_teleop.py --leader_robot UR10 --leader_port 192.168.1.100 --follower_robot SO100 --follower_port COM4

Created by Sandor Burian
"""

import sys
import os
import subprocess
import time
import signal
from pathlib import Path
import argparse

# Add project paths
script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir))

from robot_interface import get_supported_robots

class UniversalTeleopLauncher:
    """Launch and manage multi-robot teleoperation systems"""
    
    def __init__(self):
        self.processes = []
        self.running = True
        self.log_files = []
        
        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\nReceived signal {signum}. Shutting down...")
        self.shutdown()
        sys.exit(0)
    
    def launch_teleop(self, leader_robot: str, leader_port: str, 
                     follower_robot: str, follower_port: str):
        """
        Launch teleoperation with specified robot types
        
        Args:
            leader_robot: Type of leader robot (SO100, UR5, UR10, etc.)
            leader_port: Connection port for leader robot
            follower_robot: Type of follower robot  
            follower_port: Connection port for follower robot
        """
        
        print(f"Starting Universal Robot Teleoperation")
        print(f"Leader: {leader_robot} on {leader_port}")
        print(f"Follower: {follower_robot} on {follower_port}")
        print(f"Communication: Standard ROS2 topics via ZeroMQ")
        print("-" * 60)
        
        # Validate robot types
        supported = get_supported_robots()
        if leader_robot not in supported:
            print(f"Error: Leader robot '{leader_robot}' not supported. Available: {supported}")
            return False
        if follower_robot not in supported:
            print(f"Error: Follower robot '{follower_robot}' not supported. Available: {supported}")
            return False
        
        try:
            # Start follower node FIRST
            print(f"Starting {follower_robot} follower node...")
            follower_cmd = [
                sys.executable,
                str(script_dir / "so100_follower_minimal.py"),
                "--robot_type", follower_robot,
                "--robot_port", follower_port
            ]
            # Ensure logs directory exists and open log files
            logs_dir = script_dir / "logs"
            logs_dir.mkdir(parents=True, exist_ok=True)
            from datetime import datetime
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            follower_log_path = logs_dir / f"follower_{ts}.log"
            follower_fh = open(follower_log_path, "a", encoding="utf-8")
            self.log_files.append(follower_fh)

            follower_process = subprocess.Popen(
                follower_cmd,
                stdout=follower_fh,
                stderr=subprocess.STDOUT,
                universal_newlines=True
            )
            self.processes.append(("follower", follower_process))

            # Wait for follower to connect before starting leader (ZeroMQ slow joiner fix)
            time.sleep(1)

            # Start leader node SECOND
            print(f"Starting {leader_robot} leader node...")
            leader_cmd = [
                sys.executable, 
                str(script_dir / "so100_leader_minimal.py"),
                "--robot_type", leader_robot,
                "--robot_port", leader_port
            ]
            leader_log_path = logs_dir / f"leader_{ts}.log"
            leader_fh = open(leader_log_path, "a", encoding="utf-8")
            self.log_files.append(leader_fh)

            leader_process = subprocess.Popen(
                leader_cmd,
                stdout=leader_fh,
                stderr=subprocess.STDOUT,
                universal_newlines=True
            )
            self.processes.append(("leader", leader_process))
            
            print("\nTeleoperation System Started!")
            print("Topics published/subscribed:")
            print("  - robot_target_pose (PoseStamped)")
            print("  - robot_target_joints (JointState)")
            print("  - teleop_command (TeleopCommand)")
            print("  - leader_status, robot_status (RobotStatus)")
            print("\nPress Ctrl+C to stop...")
            
            # Monitor processes
            self.monitor_processes()
            
            return True
            
        except Exception as e:
            print(f"Launch error: {e}")
            self.shutdown()
            return False
    
    def monitor_processes(self):
        """Monitor running processes and handle output"""
        try:
            while self.running and self.processes:
                # Check if any process has terminated
                for name, process in self.processes[:]:
                    if process.poll() is not None:
                        print(f"\n{name} process terminated with code {process.returncode}")
                        self.processes.remove((name, process))
                
                # If any process died, shutdown all
                if len(self.processes) < 2 and self.running:
                    print("Process died, shutting down...")
                    self.shutdown()
                    break
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received...")
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown of all processes"""
        self.running = False
        print("\nShutting down teleoperation system...")
        
        for name, process in self.processes:
            try:
                print(f"Terminating {name} process...")
                process.terminate()
                
                # Wait a bit for graceful shutdown
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    print(f"Force killing {name} process...")
                    process.kill()
                    process.wait()
                    
            except Exception as e:
                print(f"Error shutting down {name}: {e}")
        
        self.processes.clear()
        # Close any opened log file handles
        for fh in getattr(self, 'log_files', []):
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass
        print("Shutdown complete.")

def main():
    parser = argparse.ArgumentParser(
        description='Universal Robot Teleoperation Launcher',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # SO100 to SO100 teleoperation (classic)
  python launch_teleop.py --leader_robot SO100 --leader_port COM3 --follower_robot SO100 --follower_port COM4
  
  # LeRobot SO100 leader with SO100 follower
  python launch_teleop.py --leader_robot LeRobot_SO100 --follower_robot SO100 --follower_port COM4
  
  # SO100 leader with LeRobot SO100 follower  
  python launch_teleop.py --leader_robot SO100 --leader_port COM3 --follower_robot LeRobot_SO100
  
  # LeRobot Aloha leader with SO100 follower
  python launch_teleop.py --leader_robot LeRobot_Aloha --follower_robot SO100 --follower_port COM4
        """
    )
    
    # Robot configuration
    supported_robots = get_supported_robots()
    
    parser.add_argument('--leader_robot', default='SO100', choices=supported_robots,
                       help='Type of leader robot')
    parser.add_argument('--leader_port', default='COM3',
                       help='Leader robot connection port')
    parser.add_argument('--follower_robot', default='SO100', choices=supported_robots,
                       help='Type of follower robot')
    parser.add_argument('--follower_port', default='COM4',
                       help='Follower robot connection port')
    
    # Advanced options
    parser.add_argument('--list_robots', action='store_true',
                       help='List all supported robot types')
    
    args = parser.parse_args()
    
    if args.list_robots:
        print("Supported Robot Types:")
        for robot in supported_robots:
            print(f"  - {robot}")
        print(f"\nTotal: {len(supported_robots)} robot types supported")
        return
    
    # Launch teleoperation
    launcher = UniversalTeleopLauncher()
    launcher.launch_teleop(
        args.leader_robot, args.leader_port,
        args.follower_robot, args.follower_port
    )

if __name__ == '__main__':
    main()