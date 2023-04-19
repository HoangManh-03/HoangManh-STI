#!/usr/bin/env python
# author : Quang - BEE

import serial
import sys
import math
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from sti_msgs.msg import Magnetic_line

import rospy
import time

class read_magneticLine():
	def __init__(self):
		rospy.init_node('sti_magneticLine', anonymous=False)
		self.rate = rospy.Rate(40) # max = 79 Hz

		self.public_value = rospy.Publisher("/magneticLine1", Magnetic_line, queue_size=10)
		self.magneticLine = Magnetic_line()
		self.magneticLine.status = 1
		# status = 1 (all right) | != 1 error.
		port = rospy.get_param("port_magLine", 'stibase_magLine')
		self.port_magLine = '/dev/' + port
		self.BAUDRATE = 115200
		self.ID_SLAVE = 1
		self.numberRegisters_read = 40
		self.numberByte_read = 1
		self.id_modbus_recived = False
		self.maxValue = 100
		self.maxOriginValue = math.pow(2, 16)
		try:
			self.MODBUS = modbus_rtu.RtuMaster(
				serial.Serial(port= self.port_magLine, baudrate= self.BAUDRATE, bytesize=8, parity='E', stopbits=1, xonxoff=0)
			)

			self.MODBUS.open 
			self.MODBUS.set_timeout(1.0)
			self.MODBUS.set_verbose(True)
			print("Modbus connected !")

		except modbus_tk.modbus.ModbusError as exc:
			print("Modbus false!") 
			sys.exit()

	def tranformValue(self, raw_value):
		value = (raw_value/self.maxOriginValue)*100
		return int(value)

	def run(self):
		data = 0
		data = self.MODBUS.execute(self.ID_SLAVE, cst.READ_HOLDING_REGISTERS, self.numberRegisters_read, self.numberByte_read)
		a = bin(data[0])
		arr = []
		for j in range(16):
			arr.append(0)
		# print "len", len(arr)
		dis = len(a) - 2
		for k in range(dis):
			arr[15 - k] = a[len(a) - 1 - k]

		# print "arr", arr
		t = 0
		n = 0
		for i in range(16):
			if (arr[i] == '1'):
				# print i
				n += 1.
				t += i

		if n == 0:
			data_ = -1
		elif n > 14:
			data_ = 20
		else:
			data_ = (t/n)

		# print "av:", data_

		self.magneticLine.value = data_
		self.public_value.publish(self.magneticLine)
		self.rate.sleep()

def main():
    # Start the job threads
	class_1 = read_magneticLine()
	while not rospy.is_shutdown():		
	# while (1):
		class_1.run()

if __name__ == '__main__':
	main() 	
