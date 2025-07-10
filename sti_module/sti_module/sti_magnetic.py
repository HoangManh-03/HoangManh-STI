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

class read_magneticLine(LifecycleNode):
    def __init__(self):
        super().__init__('sti_magneticLine')
        self.get_logger().info("ROS 2 Node sti_magneticLine Initialized!")
        self.killnode = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('port_magLine',  'stibase_magline'),
                ('baudrate',  19200),
                ('id_magline_front',  6),
                ('id_magline_behind',  4)
            ]
        )

        # -- Get param
        self.port_magLine = '/dev/' + self.get_parameter('port_magLine').value
        self.BAUDRATE = self.get_parameter('baudrate').value
        self.ID_MAGLINE_FRONT = self.get_parameter('id_magline_front').value
        self.ID_MAGLINE_BEHIND = self.get_parameter('id_magline_behind').value

        # -- Publisher 
        # -
        self.public_value_front = self.create_publisher(MagneticLine, '/magneticLine_front', 10)
        self.magneticLine_front = MagneticLine()
        # -
        self.public_value_behind = self.create_publisher(MagneticLine, '/magneticLine_behind', 10)
        self.magneticLine_behind = MagneticLine()

        self.numberRegisters_read = 40
        self.numberByte_read = 1
        self.id_modbus_recived = False
        self.maxValue = 100
        self.maxOriginValue = math.pow(2, 16)
        self.timewait = 0.001

        self.toggle = False

        try:
            self.MODBUS = modbus_rtu.RtuMaster(
                serial.Serial(port= self.port_magLine, baudrate= self.BAUDRATE, bytesize=8, parity='E', stopbits=1, xonxoff=0)
            )

            self.MODBUS.open()
            self.MODBUS.set_timeout(0.1)
            self.MODBUS.set_verbose(True)
            print("Modbus connected !")

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

    def tranformValue(self, raw_value):
        value = (raw_value/self.maxOriginValue)*100
        return int(value)

    def pareDataRead(self, data):
        a = bin(data[0])
        arr = []
        for j in range(16):
            arr.append(0)
        # print "len", len(arr)
        dis = len(a) - 2
        for k in range(dis):
            arr[15 - k] = a[len(a) - 1 - k]

        # print ("arr", arr)
        t = 0
        n = 0
        for i in range(16):
            if (arr[i] == '1'):
                # print(i)
                n += 1.
                t += i

        if n == 0:
            data_ = -1
        elif n > 14:
            data_ = 20
        else:
            data_ = float(t/n)

        # print(data_)
        return data_

    def run(self):
        self.toggle = not self.toggle
        
        if self.toggle:
            # read data front magline
            try:
                data = self.MODBUS.execute(self.ID_MAGLINE_FRONT, cst.READ_HOLDING_REGISTERS, self.numberRegisters_read, self.numberByte_read)
                data_pare = self.pareDataRead(data)

                self.magneticLine_front.status = 1
                self.magneticLine_front.value = float(data_pare)

            except Exception as e:
                self.magneticLine_front.status = 0
                self.magneticLine_front.value = -1.
                print("Read data front magline - Error: ", e) 
            
            # print("phia truoc: ", self.magneticLine_front)
            self.public_value_front.publish(self.magneticLine_front)

        # doi mot thoi gian
        # time.sleep(self.timewait)
        else:
            # read data behind magline
            try:
                data = self.MODBUS.execute(self.ID_MAGLINE_BEHIND, cst.READ_HOLDING_REGISTERS, self.numberRegisters_read, self.numberByte_read)
                data_pare = self.pareDataRead(data)

                self.magneticLine_behind.status = 1
                self.magneticLine_behind.value = float(data_pare)

            except Exception as e:
                self.magneticLine_behind.status = 0
                self.magneticLine_behind.value = -1.
                print("Read data behind magline - Error: ", e) 

            # print("phia sau: ", self.magneticLine_behind)
            self.public_value_behind.publish(self.magneticLine_behind)

        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)

def main():
    # -- Khoi tao ROS
    rclpy.init()
    node = read_magneticLine()
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