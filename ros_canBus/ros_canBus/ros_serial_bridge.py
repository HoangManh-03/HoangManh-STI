import sys
import time
import serial
import threading

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

from message_pkg.msg import *

class SerialPort:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.is_connect = False
        self.data_receive = None
        self.buffer = [0] * 14
        self.index = 0

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.5)
            print(f"[Serial] Connected to {self.port} baudrate {self.baudrate}")
            self.is_connect = True
        except serial.SerialException as e:
            print(f"[Serial] Failed to open port {self.port}: {e}")
            self.is_connect = False

    def start(self):
        self.connect()

    def stop(self):
        if self.is_connect and self.serial:
            try:
                self.serial.close()
            except Exception:
                pass

    def write(self, data: bytes):
        if self.is_connect and self.serial.is_open:
            try:
                self.serial.write(data)
            except Exception as e:
                print("Error when writing to Serial:", e)
                self.is_connect = False

    def read_loop(self):
        try:
            if self.is_connect and self.serial.in_waiting:
                byte = self.serial.read(1)
                try:
                    if self.index == 0:
                        if byte[0] == 0x2a:
                            self.buffer[self.index] = byte[0]
                            self.index += 1
                    else:
                        self.buffer[self.index] = byte[0]
                        self.index += 1

                        if self.index == 14:
                            if self.buffer[13] == 0x23:
                                self.data_receive = self.buffer[1:13]
                            self.index = 0

                except Exception as e:
                    print("Error while processing packet, error: ", e)

        except Exception as e:
            print("Error when reading Serial:", e)
            self.is_connect = False

    def run_forever(self):
        while True:
            self.read_loop()
            time.sleep(0.001)

class SerialROSBridge(LifecycleNode):
    def __init__(self):
        super().__init__('serial_ros_bridge_node')
        self.get_logger().warn("ROS 2 Node serial_ros_bridge_node Initialized!")
        self.killnode = 0

        self.declare_parameters('', [
            ('serial_port', "stibase_rtc"),
            ('baudrate', 115200)
        ])

        self.PORT = "/dev/" + self.get_parameter("serial_port").value
        self.BAUDRATE = self.get_parameter("baudrate").value

        # -- Publisher 
        self.pub_can_receive = self.create_publisher(Canreceived, '/can_received', 10)

        # -- Subcriber
        self.sub_canSend = self.create_subscription(
            Cansend,
            'can_send',
            self.callback_dataCanSend,
            10
        )
        self.sdata_CANReceived = Canreceived()
        self.serial = SerialPort(self.PORT, self.BAUDRATE)
        self.serial.start()

        # -- Thread đọc serial
        self.serial_thread = threading.Thread(target=self.serial.run_forever, daemon=True)
        self.serial_thread.start()

        self.rate = 50
        self.timer_period = 1/self.rate
        self.timer = self.create_timer(self.timer_period, self.run)

        # -- Reconnect timeout
        self.timeReconnectSerial = time.time()

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    def callback_dataCanSend(self, data):
        frame = [0x2a]

        frame.append((data.id >> 0) & 0xFF)
        frame.append((data.id >> 8) & 0xFF)
        frame.append((data.id >> 16) & 0xFF)
        frame.append((data.id >> 24) & 0xFF)

        frame.extend([
            data.byte0, data.byte1, data.byte2, data.byte3,
            data.byte4, data.byte5, data.byte6, data.byte7
        ])
        frame.append(0x23)

        self.serial.write(bytes(frame))

    def run(self):
        if self.serial.is_connect:
            if self.serial.data_receive is not None:
                CAN_dataReceived = Canreceived()
                CAN_dataReceived.idsend = (
                    self.serial.data_receive[0] |
                    (self.serial.data_receive[1] << 8) |
                    (self.serial.data_receive[2] << 16) |
                    (self.serial.data_receive[3] << 24)
                )
                CAN_dataReceived.byte0 = self.serial.data_receive[4]
                CAN_dataReceived.byte1 = self.serial.data_receive[5]
                CAN_dataReceived.byte2 = self.serial.data_receive[6]
                CAN_dataReceived.byte3 = self.serial.data_receive[7]
                CAN_dataReceived.byte4 = self.serial.data_receive[8]
                CAN_dataReceived.byte5 = self.serial.data_receive[9]
                CAN_dataReceived.byte6 = self.serial.data_receive[10]
                CAN_dataReceived.byte7 = self.serial.data_receive[11]

                self.pub_can_receive.publish(CAN_dataReceived)
                self.serial.data_receive = None

        else:
            if time.time() - self.timeReconnectSerial > 1.5:
                self.serial.connect()
                self.timeReconnectSerial = time.time()

        if self.killnode:
            sys.exit(0)

    def destroy_node(self):
        self.serial.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = SerialROSBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')

if __name__ == '__main__':
    main()
