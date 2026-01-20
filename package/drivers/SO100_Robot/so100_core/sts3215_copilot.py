import time
import serial
import struct

class STS3215Driver:
    def __init__(self, port, baudrate=1000000, timeout=0.05):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.lock = False # Egyszerű thread-safety kezdemény

    def close(self):
        if self.ser.is_open:
            self.ser.close()

    def _read_packet(self, motor_id, length):
        # A te eredeti logikád alapján
        msg = [0xFF, 0xFF, motor_id, length + 2, 0x02] # 0x02 = Read
        chk = (~sum(msg[2:]) & 0xFF)
        msg.append(chk)
        
        self.ser.reset_input_buffer()
        self.ser.write(bytearray(msg))
        time.sleep(0.002) # Kicsi várakozás a busznak
        
        response = self.ser.read(length + 6) # Header + data + checksum
        if len(response) < length + 6:
            return None
        return response

    def _write_packet(self, motor_id, instruction, params):
        # A te eredeti write logikád
        length = len(params) + 2
        msg = [0xFF, 0xFF, motor_id, length, instruction] + params
        chk = (~sum(msg[2:]) & 0xFF)
        msg.append(chk)
        self.ser.write(bytearray(msg))

    def read_position(self, motor_id):
        # Nyers 0-4096 érték olvasása
        # Cím: 0x38 (Present Position), Hossz: 2
        resp = self._read_packet(motor_id, 2) # Read 2 bytes
        if resp:
            # A válasz felépítése: FF FF ID Len Err P1 P2 Chk
            # Az adat az 5. és 6. byte (index 5, 6)
            try:
                low = resp[5]
                high = resp[6]
                val = (high << 8) + low
                return val
            except IndexError:
                return None
        return None

    def write_position(self, motor_id, position, speed=1000, acc=50):
        # Nyers pozíció írása (Gen Write: 0x03)
        # Regiszter 0x2A (Target Position)
        pos = int(position)
        msp = int(speed)
        acc = int(acc)
        
        params = [
            0x2A, # Address
            0x00,
            pos & 0xFF, (pos >> 8) & 0xFF,
            0x00, 0x00, # Time (nem használjuk most)
            msp & 0xFF, (msp >> 8) & 0xFF, # Speed
        ]
        self._write_packet(motor_id, 0x03, params)

    def torque_enable(self, motor_id, enable=True):
        val = 1 if enable else 0
        self._write_packet(motor_id, 0x03, [0x28, 0x00, val])