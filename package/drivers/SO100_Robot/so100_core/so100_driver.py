#!/usr/bin/env python3
"""
SO100 Robot Control Driver (Simplified)
Based on the original working driver by Sandor Burian
Adapted for the new package structure while keeping the working logic
"""

import os
import serial
import time
import struct
import math
import csv
import ikpy.chain

class SO100Robot:
    def __init__(self, port=None, config_dir=None, calibration_file="follower_calibration.csv", simulation=False):
        """
        Initialize SO100 Robot with simplified configuration
        """
        self.simulation = simulation
        self.port = port
        
        # Find config directory
        if config_dir is None:
            # Default to SO100_Robot/config
            script_dir = os.path.dirname(__file__)  # so100_core directory
            so100_dir = os.path.dirname(script_dir)  # SO100_Robot directory
            config_dir = os.path.join(so100_dir, "config")
        elif not os.path.isabs(config_dir):
            # Relative path - from SO100_Robot directory
            script_dir = os.path.dirname(__file__)
            so100_dir = os.path.dirname(script_dir)
            config_dir = os.path.join(so100_dir, config_dir)
            
        print(f"[SO100] Config directory: {config_dir}")
        
        # Load URDF
        urdf_path = os.path.join(config_dir, "so100.urdf")
        if os.path.exists(urdf_path):
            self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["base"])
            print(f"[SO100] URDF loaded: {urdf_path}")
        else:
            raise FileNotFoundError(f"URDF not found: {urdf_path}")
        
        # Load calibration
        calib_path = os.path.join(config_dir, calibration_file)
        self._load_calibration(calib_path)
        
        # Motor communication
        self.ser = None
        self.port = port  # Store port for recovery
        self.baud = None
        if not simulation and port:
            # Probe for correct baudrate: try high-speed first (1_000_000), then fallback to 115200
            candidate_bauds = [1000000, 115200]
            found = False
            for b in candidate_bauds:
                try:
                    ser_try = serial.Serial(port, b, timeout=0.2)
                    # Send a simple ping (instr=0x01) to motor id=1 and look for a response
                    try:
                        pkt = bytearray([0xFF, 0xFF, 0x01, 0x02, 0x01])
                        # append checksum for pkt[2:]
                        chk = (~sum(pkt[2:])) & 0xFF
                        pkt.append(chk)
                        ser_try.reset_input_buffer()
                        ser_try.write(pkt)
                        try:
                            ser_try.flush()
                        except Exception:
                            pass
                        # read short response
                        resp = ser_try.read(8)
                        if resp and len(resp) >= 4:
                            # got response -> accept this baud
                            self.ser = ser_try
                            self.baud = b
                            print(f"[SO100] Connected to: {port} at {b} baud (detected)")
                            found = True
                            break
                    except Exception:
                        try:
                            ser_try.close()
                        except Exception:
                            pass
                        continue
                except Exception:
                    continue

            if not found:
                print(f"[SO100] Connection failed on {port} for candidate bauds {candidate_bauds}")
                self.simulation = True
            else:
                # After connecting, attempt to enable torque on all motors and verify responses
                try:
                    self.enable_all_motors()
                except Exception as e:
                    print(f"[SO100] Warning: enable_all_motors failed during init: {e}")
        
        # State tracking
        self.current_joint_state = [0.0] * 6
        self.serial_error_count = 0  # Track consecutive errors
        self.max_serial_errors = 3   # Reset connection after this many errors (reduced from 5)
        # Track whether torque is enabled per motor to avoid re-sending enable repeatedly
        self.torque_enabled = [False] * 6
        
        # Gripper limits  
        self.gripper_limits = {'open': 2000, 'close': 1500}
        self._load_gripper_config(os.path.join(config_dir, "gripper_values.csv"))

    def _load_calibration(self, calib_path):
        """Load calibration from CSV file (original working logic)"""
        try:
            with open(calib_path, 'r') as file:
                reader = csv.reader(file)
                print(f"[SO100] Loading calibration: {calib_path}")
                for row in reader:
                    if len(row) > 1:
                        param_name = row[0]
                        values = [float(x) if '.' in x else int(x) for x in row[1:]]
                        if param_name == 'ZERO_OFFSETS': 
                            self.ZERO_OFFSETS = values
                        elif param_name == 'DIRECTIONS': 
                            self.DIRECTIONS = values
                        elif param_name == 'CALIBRATION_POSE_ADJUSTMENTS': 
                            self.CALIBRATION_POSE_ADJUSTMENTS = values
                            
                print(f"[SO100] Calibration loaded successfully")
                print(f"  Offsets: {self.ZERO_OFFSETS}")
                print(f"  Directions: {self.DIRECTIONS}")
                print(f"  Adjustments: {self.CALIBRATION_POSE_ADJUSTMENTS}")
                            
        except FileNotFoundError:
            print(f"[SO100] ERROR: Calibration file not found: {calib_path}")
            print("[SO100] Using default calibration values (may cause incorrect movement)")
            # Safe defaults
            self.ZERO_OFFSETS = [2048] * 6
            self.DIRECTIONS = [1] * 6
            self.CALIBRATION_POSE_ADJUSTMENTS = [0.0] * 6

    def _load_gripper_config(self, gripper_path):
        """Load gripper configuration"""
        try:
            with open(gripper_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if row[0] == 'GRIPPER_OPEN': self.gripper_limits['open'] = int(row[1])
                    elif row[0] == 'GRIPPER_CLOSE': self.gripper_limits['close'] = int(row[1])
                print(f"[SO100] Gripper config: {self.gripper_limits}")
        except FileNotFoundError:
            print("[SO100] Gripper config not found, using defaults")

    def enable_all_motors(self, verify=True):
        """Attempt to enable torque on all motors and optionally verify by reading position."""
        if self.simulation or not self.ser:
            return False

        results = {}
        for motor_id in range(1, 7):
            try:
                # Send torque enable (address 0x28, value 0x01) - existing protocol
                ok = self._write_packet(motor_id, 0x03, [0x28, 0x01])
                results[motor_id] = {'write_ok': ok, 'resp': None}
                if ok:
                    # Mark torque enabled for this motor (we just wrote enable)
                    try:
                        self.torque_enabled[motor_id - 1] = True
                    except Exception:
                        pass
                if verify and ok:
                    # Try to read a position to ensure motor responds
                    pos = self.read_raw_position(motor_id)
                    results[motor_id]['resp'] = pos
                    print(f"[SO100] enable_all_motors: motor {motor_id} pos read -> {pos}")
                time.sleep(0.05)
            except Exception as e:
                results[motor_id] = {'write_ok': False, 'error': str(e)}
        return results

    def _checksum(self, data):
        """STS3215 Checksum calculation"""
        return (~sum(data)) & 0xFF

    def _write_packet(self, motor_id, instruction, parameters, max_retries=4):
        """Send packet to motor with retry mechanism"""
        if self.simulation: 
            return True
        
        # Protocol: [FF, FF, ID, LEN, INSTR, P1, P2..., CHK]
        length = len(parameters) + 2
        packet = [0xFF, 0xFF, motor_id, length, instruction] + parameters
        checksum = self._checksum(packet[2:])
        packet.append(checksum)
        # Debug: print outgoing packet
        try:
            port_name = self.ser.port if self.ser else 'N/A'
        except Exception:
            port_name = 'N/A'
        print(f"[SO100][DEBUG] _write_packet to port {port_name}: {packet} (hex: {' '.join([hex(b) for b in packet])})")
        
        # Try sending the packet with retries
        for attempt in range(max_retries + 1):
            try:
                # Check if serial connection is available
                if not self.ser or not self.ser.is_open:
                    self.serial_error_count += 1
                    return False
                    
                # Flush buffers before writing for cleaner communication
                try:
                    if hasattr(self.ser, 'reset_input_buffer'):
                        self.ser.reset_input_buffer()
                    if hasattr(self.ser, 'reset_output_buffer'):
                        self.ser.reset_output_buffer()
                except:
                    pass  # Ignore flush errors
                
                # Write with timeout protection
                bytes_written = self.ser.write(bytearray(packet))
                print(f"[SO100][DEBUG] Wrote {bytes_written}/{len(packet)} bytes to {port_name}")
                if bytes_written != len(packet):
                    if attempt < max_retries:
                        time.sleep(0.01 * (attempt + 1))  # Progressive delay
                        continue
                    print(f"[SO100] Write incomplete: {bytes_written}/{len(packet)} bytes on port {port_name}")
                    self.serial_error_count += 1
                    return False
                
                # Flush output buffer to ensure packet is sent
                try:
                    self.ser.flush()
                except:
                    pass  # Ignore flush errors
                
                # Small pause to allow device to process packet before reading
                time.sleep(0.02)
                # Try to read any immediate response (non-blocking due to timeout)
                try:
                    if hasattr(self.ser, 'in_waiting') and self.ser.in_waiting > 0:
                        resp = self.ser.read(self.ser.in_waiting)
                    else:
                        # Try reading up to 8 bytes which is typical response length
                        resp = self.ser.read(8)
                    if resp:
                        try:
                            hex_resp = ' '.join([hex(b) for b in resp])
                        except Exception:
                            hex_resp = str(resp)
                        print(f"[SO100][DEBUG] Read response from {port_name}: {hex_resp}")
                    else:
                        # No immediate response; allow small recovery before next attempt
                        pass
                except Exception as e:
                    print(f"[SO100][DEBUG] Read after write failed: {e}")
                self.serial_error_count = 0  # Reset error count on success
                return True
                
            except Exception as e:
                if attempt < max_retries:
                    time.sleep(0.02 * (attempt + 1))  # Progressive delay
                    continue
                    
                print(f"[SO100] Write error: {e}")
                self.serial_error_count += 1
                
                # Try to recover serial connection if too many errors
                if self.serial_error_count >= self.max_serial_errors:
                    print(f"[SO100] Maximum serial errors ({self.max_serial_errors}) reached, attempting recovery...")
                    self._recover_serial_connection()
                return False
                
        return False

    def set_target_angle(self, motor_index, angle_radians, move_time_ms=1000):
        print(f"[DEBUG] SO100Driver: set_target_angle called with motor_index={motor_index}, angle={angle_radians}, move_time_ms={move_time_ms}")
        """
        Move single motor to specific angle (original working method)
        motor_index: 0-5 (0 = motor 1)
        angle_radians: target angle in radians
        move_time_ms: movement time in milliseconds
        Returns: True if successful, False if failed
        """
        if self.simulation: 
            return True

        motor_id = motor_index + 1
        
        # 1. Enable torque first only if not already enabled (reduces serial traffic)
        try:
            if not self.torque_enabled[motor_index]:
                if not self._write_packet(motor_id, 0x03, [0x28, 0x01]):
                    return False
                self.torque_enabled[motor_index] = True
        except Exception:
            # Fallback: attempt to send enable and continue
            if not self._write_packet(motor_id, 0x03, [0x28, 0x01]):
                return False

        # 2. Angle conversion (original working formula)
        offset = self.ZERO_OFFSETS[motor_index]
        direction = self.DIRECTIONS[motor_index]
        calib = self.CALIBRATION_POSE_ADJUSTMENTS[motor_index]
        
        ratio = (2 * math.pi / 4096)
        raw_val = ((angle_radians - calib) / (ratio * direction)) + offset
        raw_int = int(raw_val)
        raw_int = max(0, min(4095, raw_int))  # Safety clamp
        
        # 3. Position command (original working protocol)
        p_low = raw_int & 0xFF
        p_high = (raw_int >> 8) & 0xFF
        t_low = move_time_ms & 0xFF
        t_high = (move_time_ms >> 8) & 0xFF
        
        params = [0x2A, p_low, p_high, t_low, t_high, 0, 0]
        result = self._write_packet(motor_id, 0x03, params)
        print(f"[DEBUG] SO100Driver: set_target_angle returning {result}")
        return result

    def move_to_joints(self, joint_angles, time_ms=1000):
        """
        Move to joint angles with motor-by-motor control (original working logic)
        joint_angles: List of 6 angles in radians
        time_ms: Movement time in milliseconds
        """
        if len(joint_angles) < 6:
            print(f"[SO100] ERROR: Need 6 joint angles, got {len(joint_angles)}")
            return
            
        # ORIGINAL WORKING METHOD: Motor-by-motor with delays
        for i, target_rad in enumerate(joint_angles):
            if i >= 6: break
            
            self.set_target_angle(i, target_rad, time_ms)
            
            # CRITICAL: 0.1s delay between motors for better reliability
            time.sleep(0.1)
        
        # Wait for movement completion
        time.sleep(time_ms / 1000.0)
        
        # Update internal state
        self.current_joint_state = list(joint_angles)

    def read_raw_position(self, motor_id):
        """
        Read raw position from motor (original working method)
        """
        if self.simulation: 
            return 2048
        
        # Check if serial connection is available
        if not self.ser or not self.ser.is_open:
            return None
            
        try:
            msg = [0xFF, 0xFF, motor_id, 0x04, 0x02, 0x38, 0x02]
            msg.append(self._checksum(msg[2:]))
            self.ser.reset_input_buffer()
            self.ser.write(bytearray(msg))
            response = self.ser.read(8)
            if len(response) == 8:
                return struct.unpack('<H', response[5:7])[0]
        except Exception as e:
            # Don't print error for every read failure when in recovery
            return None
        return None

    def get_joint_angles(self):
        """
        Get current joint angles in radians (original working method)
        """
        if self.simulation:
            return [0.0] * 6

        angles = []
        for i in range(6):  # Motors 1-6
            raw = self.read_raw_position(i + 1)
            if raw is None: 
                raw = self.ZERO_OFFSETS[i]

            # Convert raw to radians (original working formula)
            offset = self.ZERO_OFFSETS[i]
            direction = self.DIRECTIONS[i]
            ratio = (2 * math.pi / 4096)
            rel_rad = (raw - offset) * ratio * direction
            final_rad = rel_rad + self.CALIBRATION_POSE_ADJUSTMENTS[i]
            angles.append(final_rad)
            
        return angles

    def move_to_cartesian(self, x, y, z, time_ms=1500):
        """
        Move to cartesian position using motor-by-motor control (original working method)
        """
        print(f"[DEBUG] SO100Driver: move_to_cartesian called with x={x}, y={y}, z={z}, time_ms={time_ms}")
        if self.simulation:
            return True
            
        # Check workspace bounds first
        if not self._is_position_reachable(x, y, z):
            print(f"[SO100] Position ({x:.3f}, {y:.3f}, {z:.3f}) outside workspace")
            return False
            
        # IK calculation with better seed strategy
        current_angles = self.get_joint_angles()
        
        # Try multiple seed strategies for robust IK solving
        seeds = [
            [0] + current_angles[:5] + [0],  # Current position seed
            [0, 0, 0, 0, 0, 0, 0],           # Zero position seed 
            [0, 0, -1.5, -2.0, -1.0, 0, 0], # Common working position
            [0, 0, -0.5, -1.0, -2.0, 0, 0], # Alternative position
        ]
        
        for i, seed in enumerate(seeds):
            try:
                target_joints = self.chain.inverse_kinematics(
                    target_position=[x, y, z],
                    initial_position=seed,
                    max_iter=1000,  # More iterations
                    orientation_mode=None  # Only position, ignore orientation
                )
                
                # Extract motor angles (skip base and end effector)
                motor_commands = target_joints[1:6]
                
                # Validate joint limits before attempting move
                if self._validate_joint_limits(motor_commands):
                    print(f"[SO100] Moving to cartesian ({x:.2f}, {y:.2f}, {z:.2f}) with seed {i}")
                    success = self._execute_joint_movement(motor_commands, time_ms)
                    if success:
                        self.current_joint_state = motor_commands
                        print(f"[DEBUG] SO100Driver: move_to_cartesian returning True")
                        return True
                    else:
                        print(f"[SO100] Movement execution failed with seed {i}")
                else:
                    print(f"[SO100] Joint limits violated with seed {i}")
                    
            except Exception as e:
                print(f"[SO100] IK failed with seed {i}: {e}")
                continue
        
        # If all seeds failed, try position correction
        print(f"[SO100] All IK attempts failed, position may be unreachable")
        return False
    
    def _validate_joint_limits(self, joint_angles):
        """Validate if joint angles are within safe limits"""
        # SO100 joint limits (radians) - more permissive values based on actual hardware
        limits = [
            (-3.14, 3.14),   # Joint 1: ±180° (full rotation)
            (-3.14, 3.14),   # Joint 2: ±180° (increased from ±143°)
            (-3.14, 1.57),   # Joint 3: -180° to +90° (increased range)
            (-3.14, 3.14),   # Joint 4: ±180° (increased from ±143°)
            (-3.14, 3.14),   # Joint 5: ±180° (increased from ±115°)
        ]
        
        for i, angle in enumerate(joint_angles[:5]):
            if angle < limits[i][0] or angle > limits[i][1]:
                print(f"[SO100] Joint {i+1} angle {angle:.2f} outside limits {limits[i]}")
                return False
        return True
    
    def _execute_joint_movement(self, joint_angles, time_ms):
        """Execute joint movement with improved error handling"""
        if self.simulation:
            return True
            
        try:
            for i, angle in enumerate(joint_angles):
                success = self.set_target_angle(i, angle, time_ms)
                if not success:
                    print(f"[SO100] Failed to move motor {i+1}")
                    return False
                # Longer delay between motors for reliability
                time.sleep(0.1)
            
            # Wait for movement completion
            time.sleep(time_ms / 1000.0 + 0.3)
            return True
            
        except Exception as e:
            print(f"[SO100] Joint movement execution error: {e}")
            return False
            
    def _is_position_reachable(self, x, y, z):
        """Check if position is within robot workspace"""
        # SO100 workspace limits (approximate)
        reach = math.sqrt(x*x + y*y + z*z)
        
        # More relaxed workspace limits to match leader robot capabilities
        if reach > 0.45:  # Max reach ~45cm (was 35cm)
            return False
        if z < -0.15:  # Below base level (was -0.1)
            return False
        if z > 0.50:   # Above maximum height (was 0.4)
            return False
            
        return True

    def gripper_open(self):
        """Open gripper (original working method)"""
        val = self.gripper_limits['open']
        if not self.simulation:
            # Enable torque first
            self._write_packet(6, 0x03, [0x28, 0x01])
            # Position command
            p_low = val & 0xFF
            p_high = (val >> 8) & 0xFF
            params = [0x2A, p_low, p_high, 200, 0, 0, 0]  # 200ms movement
            self._write_packet(6, 0x03, params)
        time.sleep(0.3)

    def gripper_close(self):
        """Close gripper (original working method)"""
        val = self.gripper_limits['close']
        if not self.simulation:
            # Enable torque first
            self._write_packet(6, 0x03, [0x28, 0x01])
            # Position command
            p_low = val & 0xFF
            p_high = (val >> 8) & 0xFF
            params = [0x2A, p_low, p_high, 200, 0, 0, 0]  # 200ms movement
            self._write_packet(6, 0x03, params)
        time.sleep(0.3)

    def torque_enable(self, enable=True):
        """Enable/disable motor torque (original working method)"""
        for i in range(1, 7):
            val = 1 if enable else 0
            self._write_packet(i, 0x03, [0x28, val])
            time.sleep(0.01)  # Small delay between motors

    def torque_disable(self):
        """Disable all motor torque (from original working code)"""
        for i in range(1, 7):
            self._write_packet(i, 0x03, [0x28, 0x00])
            time.sleep(0.01)

    def getCalibrationvalues(self):
        """Get current calibration values"""
        return {
            "offsets": {i: self.ZERO_OFFSETS[i] for i in range(6)},
            "directions": {i: self.DIRECTIONS[i] for i in range(6)}, 
            "adjustments": {i: self.CALIBRATION_POSE_ADJUSTMENTS[i] for i in range(6)},
            "gripper_limits": self.gripper_limits
        }

    def get_cartesian_position(self):
        """
        Get current cartesian position using forward kinematics
        Returns [x, y, z] coordinates in meters
        """
        current_joints = self.get_joint_angles()
        if current_joints:
            # Use first 5 joints for position (exclude gripper)
            arm_joints = [0] + current_joints[:5] + [0]  # Pad for IKpy
            frame = self.chain.forward_kinematics(arm_joints)
            return frame[:3, 3]  # Extract XYZ coordinates
        return [0.0, 0.0, 0.0]

    def has_gripper(self):
        """Check if robot has a gripper"""
        return True  # SO100 has a gripper

    def get_joint_states(self):
        """
        Get current joint states for ROS2 feedback
        Returns: (joint_names, positions, velocities, efforts)
        """
        try:
            positions = self.get_joint_angles()
            if positions:
                joint_names = [f"joint_{i+1}" for i in range(6)]
                velocities = [0.0] * 6  # Not measured
                efforts = [0.0] * 6     # Not measured
                return joint_names, positions, velocities, efforts
            else:
                return None, None, None, None
        except Exception:
            return None, None, None, None

    def get_cartesian_pose(self):
        """
        Get current cartesian pose for ROS2 feedback
        Returns: (position, orientation)
        """
        try:
            position = self.get_cartesian_position()
            orientation = [0.0, 0.0, 0.0, 1.0]  # Default quaternion
            return position, orientation
        except Exception:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]

    def _recover_serial_connection(self):
        """Try to recover serial connection after multiple failures - with hard reset"""
        if self.simulation:
            return
            
        print(f"[SO100] Attempting to recover serial connection after {self.serial_error_count} errors...")
        
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                # Close existing connection safely
                if self.ser:
                    try:
                        if self.ser.is_open:
                            self.ser.close()
                    except:
                        pass  # Ignore errors during close
                    self.ser = None
                
                # Wait longer for Windows to release the port
                wait_time = 3.0 + (attempt * 2.0)  # Longer wait: 3.0s, 5.0s, 7.0s
                print(f"[SO100] Waiting {wait_time}s for port to become available...")
                time.sleep(wait_time)
                
                # Try to reopen connection with hardware reset
                # Prefer previously detected baud if available, then try 1_000_000 and 115_200
                candidate_bauds = []
                if getattr(self, 'baud', None):
                    candidate_bauds.append(self.baud)
                for cb in (1000000, 115200):
                    if cb not in candidate_bauds:
                        candidate_bauds.append(cb)

                for baud_try in candidate_bauds:
                    try:
                        print(f"[SO100] Trying to open {self.port} at {baud_try} baud...")
                        self.ser = serial.Serial(self.port, baud_try, timeout=1.0)
                        # Quick verification ping to ensure device responds at this baud
                        try:
                            pkt = bytearray([0xFF, 0xFF, 0x01, 0x02, 0x01])
                            pkt.append(self._checksum(pkt[2:]))
                            # clear input and send
                            try:
                                if hasattr(self.ser, 'reset_input_buffer'):
                                    self.ser.reset_input_buffer()
                            except Exception:
                                pass
                            self.ser.write(pkt)
                            try:
                                self.ser.flush()
                            except Exception:
                                pass
                            resp = self.ser.read(8)
                            if not resp:
                                # No response at this baud, try next
                                try:
                                    if self.ser and self.ser.is_open:
                                        self.ser.close()
                                except Exception:
                                    pass
                                self.ser = None
                                print(f"[SO100] No response at {baud_try} baud, trying next")
                                continue
                        except Exception:
                            try:
                                if self.ser and self.ser.is_open:
                                    self.ser.close()
                            except Exception:
                                pass
                            self.ser = None
                            continue

                        # If we reached here, connection at baud_try seems to work
                        # Perform hardware-level reset after reconnection
                        print(f"[SO100] Performing hard reset on motors...")
                        for motor_id in range(1, 7):
                            try:
                                # Send torque disable/enable cycle
                                self._write_packet(motor_id, 0x03, [0x28, 0x00])  # Torque disable
                                time.sleep(0.1)
                                self._write_packet(motor_id, 0x03, [0x28, 0x01])  # Torque enable
                                time.sleep(0.1)
                            except Exception:
                                pass  # Continue with other motors

                        # Remember the working baud
                        self.baud = baud_try
                        self.serial_error_count = 0
                        print(f"[SO100] Serial connection recovered successfully on attempt {attempt + 1} at {baud_try} baud")
                        return
                    except Exception as e:
                        print(f"[SO100] Recovery attempt open at {baud_try} failed: {e}")
                        try:
                            if self.ser:
                                self.ser.close()
                        except Exception:
                            pass
                        self.ser = None
                        continue
                    
            except Exception as e:
                print(f"[SO100] Recovery attempt {attempt + 1} failed: {e}")
                self.ser = None  # Ensure it's None on failure
                
        # If all attempts failed, switch to simulation mode
        print(f"[SO100] Failed to recover serial connection after {max_attempts} attempts")
        print(f"[SO100] Switching to simulation mode due to unrecoverable serial error")
        self.simulation = True
        self.ser = None

    def close(self):
        """Close serial connection"""
        if self.ser:
            self.ser.close()
    
    def __del__(self):
        self.close()
        print("SO100Robot driver closed.")
