#!/usr/bin/env python3
"""
Motor position callibration for SO100 robot arm  
Created by Sandor Burian with the help of Google Gemini Pro.
"""

import serial
import time
import struct
import argparse

from visualisation.so100_viz import BAUD_RATE

def parse_arguments():
    parser = argparse.ArgumentParser(description="SO100 Motor Position Callibration")
    parser.add_argument('--port', type=str, required=True, help='Serial port (e.g., COM5 or /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=1000000, help='Baud rate (default: 1000000)')
    parser.add_argument('--motors', type=int, nargs='+', default=[1,2,3,4,5,6], help='List of motor IDs to read (default: 1 2 3 4 5 6)')
    return parser.parse_args()

# --- Settings ---
def checksum(data):
    return (~sum(data)) & 0xFF

def disable_torque(ser, motor_id):
    msg = [0xFF, 0xFF, motor_id, 0x04, 0x03, 0x28, 0x00]
    msg.append(checksum(msg[2:]))
    ser.write(bytearray(msg))
    time.sleep(0.005)

def read_position(ser, motor_id):
    msg = [0xFF, 0xFF, motor_id, 0x04, 0x02, 0x38, 0x02]
    msg.append(checksum(msg[2:]))
    ser.reset_input_buffer()
    ser.write(bytearray(msg))
    response = ser.read(8)
    if len(response) == 8:
        return struct.unpack('<H', response[5:7])[0]
    return 0

def main():
    args = parse_arguments()
    COM_PORT = args.port
    BAUD_RATE = args.baudrate
    MOTOR_IDS = args.motors
    
    print("\nCallibrating the motors")
    print("Connecting...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)

    print("Note: You will see values, when you done with callibration in candle position, then please do a keyboard interrupt!")
    print("Loosening... Set the robot to VERTICAL (Candle position)!")
    for mid in MOTOR_IDS:
        disable_torque(ser, mid)

    positions = []
    try:
        while True:
            positions = []
            for mid in MOTOR_IDS:
                pos = read_position(ser, mid)
                positions.append(pos)
            
            print(f"Current positions: {positions}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        ser.close()
    return positions

if __name__ == "__main__":
    main()