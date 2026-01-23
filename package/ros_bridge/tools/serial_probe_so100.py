#!/usr/bin/env python3
"""Probe SO100 serial interface with multiple tests.
Tries pings, broadcast, multiple baudrates, and RTS toggle for RS485 direction.
Usage: python tools\serial_probe_so100.py --port COM4
"""
import serial
import time
import argparse


def checksum(data):
    return (~sum(data)) & 0xFF


def build_packet(motor_id, instr, params):
    packet = [0xFF, 0xFF, motor_id, len(params) + 2, instr] + params
    packet.append(checksum(packet[2:]))
    return bytearray(packet)


def read_resp(ser, expect=8, timeout=0.3):
    t0 = time.time()
    buf = b''
    while time.time() - t0 < timeout:
        try:
            r = ser.read(ser.in_waiting or 1)
        except Exception:
            r = b''
        if r:
            buf += r
        else:
            time.sleep(0.01)
        if len(buf) >= expect:
            break
    if not buf:
        return None
    return ' '.join(hex(b) for b in buf)


def try_ping(ser, motor_id, use_rts=False):
    pkt = build_packet(motor_id, 0x01, [])
    try:
        if use_rts:
            try:
                ser.setRTS(True)
            except Exception:
                pass
        ser.reset_input_buffer()
        ser.write(pkt)
        try:
            ser.flush()
        except Exception:
            pass
        if use_rts:
            # small turnaround
            time.sleep(0.002)
            try:
                ser.setRTS(False)
            except Exception:
                pass
        resp = read_resp(ser, expect=8, timeout=0.3)
        return resp
    except Exception as e:
        return f"ERROR: {e}"


def probe_port(port, baud, ids, try_rts=False):
    results = {}
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(f"Failed to open {port}@{baud}: {e}")
        return None
    print(f"Opened {port}@{baud} (rts_toggle={try_rts})")
    # Try ping each id
    for mid in ids:
        resp = try_ping(ser, mid, use_rts=try_rts)
        print(f"Ping id={mid} -> {resp}")
        results[(baud, mid, try_rts)] = resp
        time.sleep(0.02)
    # Try broadcast ping (id 0xFE)
    resp = try_ping(ser, 0xFE, use_rts=try_rts)
    print(f"Broadcast ping -> {resp}")
    results[(baud, 0xFE, try_rts)] = resp
    # Try read position instruction for id 1
    pkt = build_packet(1, 0x02, [0x38, 0x02])
    try:
        if try_rts:
            try:
                ser.setRTS(True)
            except Exception:
                pass
        ser.reset_input_buffer()
        ser.write(pkt)
        try:
            ser.flush()
        except Exception:
            pass
        if try_rts:
            time.sleep(0.002)
            try:
                ser.setRTS(False)
            except Exception:
                pass
        resp2 = read_resp(ser, expect=8, timeout=0.3)
        print(f"Readpos id=1 -> {resp2}")
        results[(baud, 'readpos', try_rts)] = resp2
    except Exception as e:
        results[(baud, 'readpos', try_rts)] = f"ERROR: {e}"

    try:
        ser.close()
    except Exception:
        pass
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--bauds', default='115200,1000000', help='Comma separated bauds to try')
    parser.add_argument('--ids', default='1,2,3,4,5,6')
    args = parser.parse_args()

    bauds = [int(x) for x in args.bauds.split(',') if x]
    ids = [int(x) for x in args.ids.split(',') if x]

    all_results = {}
    for baud in bauds:
        # Try without RTS toggle
        r = probe_port(args.port, baud, ids, try_rts=False)
        all_results.update(r or {})
        time.sleep(0.1)
        # Try with RTS toggling (useful for RS485 direction control)
        r2 = probe_port(args.port, baud, ids, try_rts=True)
        all_results.update(r2 or {})
        time.sleep(0.1)

    print('\n=== SUMMARY ===')
    for k, v in all_results.items():
        print(k, '->', v)

if __name__ == '__main__':
    main()
