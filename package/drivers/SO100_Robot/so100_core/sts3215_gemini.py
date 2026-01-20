import time
import serial
import struct

class STS3215Driver:
    def __init__(self, port, baudrate=1000000, timeout=0.05):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def close(self):
        if self.ser.is_open:
            self.ser.close()

    def _write_packet(self, motor_id, instruction, params):
        length = len(params) + 2
        msg = [0xFF, 0xFF, motor_id, length, instruction] + params
        chk = (~sum(msg[2:]) & 0xFF)
        msg.append(chk)
        self.ser.write(bytearray(msg))

    def _read_packet(self, motor_id, length):
        msg = [0xFF, 0xFF, motor_id, length + 2, 0x02] 
        chk = (~sum(msg[2:]) & 0xFF)
        msg.append(chk)
        self.ser.reset_input_buffer()
        self.ser.write(bytearray(msg))
        # time.sleep(0.002) # Kicsit gyorsíthatunk rajta, ha kivesszük
        
        response = self.ser.read(length + 6)
        if len(response) < length + 6:
            return None
        return response

    def read_position(self, motor_id):
        resp = self._read_packet(motor_id, 2)
        if resp:
            try:
                low = resp[5]
                high = resp[6]
                return (high << 8) + low
            except IndexError:
                return None
        return None

    def write_position(self, motor_id, position, time_ms=0, speed=0):
        """
        Mozgatás adott pozícióra.
        HA time_ms > 0: A szervó pontosan ennyi idő alatt ér oda (Sima mozgás!)
        HA time_ms = 0: A szervó a megadott speed sebességgel megy.
        """
        pos = int(position)
        t = int(time_ms)
        sp = int(speed)
        
        # Regiszter kiosztás (STS3215):
        # 0x2A: Pozíció (2 byte)
        # 0x2C: Time (2 byte) <-- EZT KELL HASZNÁLNUNK!
        # 0x2E: Speed (2 byte)
        
        params = [
            0x2A, # Cím
            0x00,
            pos & 0xFF, (pos >> 8) & 0xFF,      # Target Position
            t & 0xFF, (t >> 8) & 0xFF,          # Time (ms)
            sp & 0xFF, (sp >> 8) & 0xFF,        # Speed
        ]
        self._write_packet(motor_id, 0x03, params)

    def torque_enable(self, motor_id, enable=True):
        val = 1 if enable else 0
        self._write_packet(motor_id, 0x03, [0x28, 0x00, val])