#!/usr/bin/env python3
"""Simple serial test for SO100 motors.
Sends torque-enable and read-position packets to motors 1..6 and prints responses.
Run with: python tools\serial_test_so100.py --port COM4
"""
import serial
import time
import argparse


def checksum(data):
    return (~sum(data)) & 0xFF


def write_packet(ser, motor_id, instruction, params):
    packet = [0xFF, 0xFF, motor_id, len(params) + 2, instruction] + params
    packet.append(checksum(packet[2:]))
    ba = bytearray(packet)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    written = ser.write(ba)
    try:
        ser.flush()
    except Exception:
        pass
    print(f"WROTE -> motor={motor_id} instr=0x{instruction:02x} len={len(ba)} bytes: {' '.join(hex(b) for b in ba)}")
    return written


def read_response(ser, expect=8, timeout=1.0):
    # read up to expect bytes (blocking up to timeout)
    t0 = time.time()
    buf = b''
    while time.time() - t0 < timeout and len(buf) < expect:
        remaining = expect - len(buf)
        try:
            r = ser.read(remaining)
        except Exception:
            r = b''
        if r:
            buf += r
        else:
            time.sleep(0.01)
    if not buf:
        return None
    try:
        return ' '.join(hex(b) for b in buf)
    except Exception:
        return str(buf)


def torque_enable_all(ser):
    results = {}
    for mid in range(1, 7):
        # write torque enable: instruction 0x03, params [0x28, 0x01]
        write_packet(ser, mid, 0x03, [0x28, 0x01])
        resp = read_response(ser, expect=8, timeout=0.5)
        print(f"RESP enable motor {mid}: {resp}")
        # try reading position as verification: instruction 0x02 params [0x38, 0x02]
        write_packet(ser, mid, 0x02, [0x38, 0x02])
        resp2 = read_response(ser, expect=8, timeout=0.5)
        print(f"RESP readpos motor {mid}: {resp2}")
        results[mid] = (resp, resp2)
        time.sleep(0.05)
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--baud', type=int, default=None,
                        help='If omitted, auto-probe 1000000 then 115200')
    parser.add_argument('--rts', action='store_true', help='Toggle RTS line after opening')
    args = parser.parse_args()

    candidates = [1000000, 115200] if args.baud is None else [args.baud]

    chosen = None
    final_results = None
    for b in candidates:
        print(f"Trying {args.port} @ {b} (rts_toggle={args.rts})")
        try:
            ser = serial.Serial(args.port, b, timeout=0.2)
        except Exception as e:
            print(f"Failed to open port at {b}: {e}")
            continue

        if args.rts:
            try:
                ser.rts = False
                time.sleep(0.02)
                ser.rts = True
                time.sleep(0.02)
            except Exception:
                pass

        print("Port opened, running torque-enable + read tests. Keep robot safe (motors may not lock).")
        try:
            results = torque_enable_all(ser)
            # consider success if any motor returned a non-None response
            success = any(rsp is not None or rsp2 is not None for (_, (rsp, rsp2)) in results.items())
            final_results = results
            chosen = b
        finally:
            try:
                ser.close()
            except Exception:
                pass

        if success:
            break

    if chosen is None:
        print("Unable to open port on any candidate baudrates.")
        return

    print("Test complete. Selected baud:", chosen)
    print("Results:\n", final_results)

if __name__ == '__main__':
    main()
