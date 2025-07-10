import serial
import sys
import math
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

import os
import time
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

from message_pkg.msg import *
from std_msgs.msg import Int16, Int8

class read_rfid(LifecycleNode):
    def __init__(self):
        super().__init__('sti_rfid')
        self.get_logger().warn("ROS 2 Node sti_rfid Initialized!")
        self.killnode = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('port_rfid',  'stibase_rfid'),
                ('baudrate',  19200),
                ('id_rfid_front',  1),
                ('id_rfid_behind',  2)
            ]
        )

        # -- Get param
        self.port_rfid = '/dev/' + self.get_parameter('port_rfid').value
        self.BAUDRATE = self.get_parameter('baudrate').value
        self.ID_RFID_FRONT = self.get_parameter('id_rfid_front').value
        self.ID_RFID_BEHIND = self.get_parameter('id_rfid_behind').value

        # -- Publisher 
        # -
        self.public_rfid_front = self.create_publisher(RFID, '/rfid_front_respond', 10)
        self.dataRFID_front = RFID()
        # -
        self.public_rfid_behind = self.create_publisher(RFID, '/rfid_behind_respond', 10)
        self.dataRFID_behind = RFID()

        self.addRegisters_read = 0x0E
        self.numberByte_read =  4 # 7 words 14 Bytes

        self.timewait = 0.001
        self.toggle = False

        try:
            self.MODBUS = modbus_rtu.RtuMaster(
                serial.Serial(port= self.port_rfid, baudrate= self.BAUDRATE, bytesize=8, parity='E', stopbits=1, xonxoff=0)
            )

            self.MODBUS.open 
            self.MODBUS.set_timeout(1.0)
            self.MODBUS.set_verbose(True)
            print("Modbus connected ! rfid")

        except modbus_tk.modbus.ModbusError as exc:
            print("Modbus false!") 
            sys.exit()

        # -- Loop
        self.rate = 30
        self.timer_period = 1/self.rate
        self.timer = self.create_timer(self.timer_period, self.run)

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    def run(self):
        self.toggle = not self.toggle
        # read data rfid front
        if self.toggle:
            try:
                rawData = self.MODBUS.execute(self.ID_RFID_FRONT, cst.READ_HOLDING_REGISTERS, self.addRegisters_read, self.numberByte_read)
                cardNumber = rawData[1] << 16 | rawData[2] 
                timeRead = round((rawData[3] & 0x00FF) * 0.02, 2)

                if timeRead > 0.05:
                    self.dataRFID_front.status = 0
                else:
                    self.dataRFID_front.status = 1

                self.dataRFID_front.id = cardNumber
                self.dataRFID_front.time_read = timeRead

            except Exception as e:
                print("Read data front rfid - Error: ", e) 
                self.dataRFID_front.status = -1

            self.public_rfid_front.publish(self.dataRFID_front)
        
        # # time.sleep(self.timewait)
        else:
            # read data rfid behind
            try:
                rawData = self.MODBUS.execute(self.ID_RFID_BEHIND, cst.READ_HOLDING_REGISTERS, self.addRegisters_read, self.numberByte_read)
                cardNumber = rawData[1] << 16 | rawData[2] 
                timeRead = round((rawData[3] & 0x00FF) * 0.02, 2)

                if timeRead > 0.05:
                    self.dataRFID_behind.status = 0
                else:
                    self.dataRFID_behind.status = 1

                self.dataRFID_behind.id = cardNumber
                self.dataRFID_behind.time_read = timeRead

            except Exception as e:
                print("Read data behind rfid - Error: ", e) 
                self.dataRFID_behind.status = -1

            self.public_rfid_behind.publish(self.dataRFID_behind)

        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)


def main():
    # -- Khoi tao ROS
    rclpy.init()
    node = read_rfid()
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