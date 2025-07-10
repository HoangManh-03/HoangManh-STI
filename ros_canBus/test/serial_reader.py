import sys
import time
import serial

class SerialPort:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.is_connect = False
        self.data_receive = None
        self.buffer = [0] * 14
        self.index = 0
        self.st = time.time()

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.5)  # Blocking read
            print(f"[Serial] Connected to {self.port} baudrate {self.baudrate}")
            self.is_connect = True
        except serial.SerialException as e:
            print(f"[Serial] Failed to open port {self.port}: {e}")
            self.is_connect = False

    def start(self):
        self.connect()

    def stop(self):
        if self.is_connect:
            try:
                self.serial.close()
            except Exception:
                pass

    def write(self, data: bytes):
        if self.is_connect and self.serial.is_open:
            try:
                self.serial.write(data)
            except Exception as e:
                print("Error when writing to serial: ", e)
                self.is_connect = False

    def read_loop(self):
        try:
            if self.is_connect:
                byte = self.serial.read(1)  # Blocking read

                print("hello")
                if self.index == 0:
                    if byte[0] == 0x2A:
                        self.buffer[self.index] = byte[0]
                        self.index += 1
                    else:
                        self.index = 0
                else:
                    self.buffer[self.index] = byte[0]
                    self.index += 1

                    if self.index == 14:
                        if self.buffer[13] == 0x23:
                            self.data_receive = self.buffer[1:13]

                            if self.data_receive[0] == 3:
                                t = time.time()
                                print((t - self.st) * 1000)
                                self.st = t
                        self.index = 0

        except Exception as e:
            print("Error when reading from serial: ", e)
            self.is_connect = False

def main():
    serial_conn = SerialPort('/dev/stibase_rtc', 115200)
    serial_conn.start()

    while True:
        try:
            serial_conn.read_loop()
        except KeyboardInterrupt:
            serial_conn.stop()
            break

if __name__ == '__main__':
    main()
