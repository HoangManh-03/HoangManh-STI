#!/usr/bin/env python3

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 17/11/2020

"""

import rospy
import sys
import time
import threading
import signal
import struct

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from message_pkg.msg import Loadcell_respond, Loadcell_query

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from decimal import *
import math

class ReadLoadCell():
	def __init__(self):
		# status = 1 (all right) | != 1 error.
		self.PORT =  rospy.get_param("port", "/dev/stibase_loadcell")
		self.BAUDRATE = rospy.get_param("baudrate", 57600) # 57600 115200

		# ------------------- Load cell
		self.ID_loadcell = rospy.get_param("id_loadcell", 3)
		self.topicSubLoadcell = rospy.get_param("topicSub_loadcell", "/loadcell_query")
		self.topicPubLoadcell = rospy.get_param("topicPub_loadcell", "/loadcell_respond")
		self.origin_weight = rospy.get_param("origin_weight", 100)

		# -- Registers
		self.reg_lc1 = 0
		self.reg_lc2 = 1
		self.reg_lc3 = 2

		# ---------------------------------------- Modbus
		print("Modbus run!")
		for i in range(3):
			try:
				self.MODBUS = modbus_rtu.RtuMaster(
					serial.Serial(port= self.PORT, baudrate= self.BAUDRATE, bytesize=8, parity='N', stopbits=1, xonxoff=0)
				)

				self.MODBUS.open 
				self.MODBUS.set_timeout(1.0)
				self.MODBUS.set_verbose(True)
				print("Modbus connected !")
				break

			except modbus_tk.modbus.ModbusError as exc:
				print("Modbus false!") 
				sys.exit()
					
		# ---------------------------------------- ROS
		print("ROS Initial!")
		rospy.init_node('read_LoadCell', anonymous=False)
		self.rate = rospy.Rate(5)
		# -- Load cell
		self.pub_dataLoadCell = rospy.Publisher(self.topicPubLoadcell, Loadcell_respond, queue_size= 50)
		self.loadcellRespond = Loadcell_respond()

		# rospy.Subscriber(self.topicSubLoadcell, Loadcell_query, self.Loadcell_infoCallback)
		# self.loadcellQuery = Loadcell_query()
		self.is_loadcellQuery = 0
		self.lastTime_checkloadcellQuery = time.time()


	def Loadcell_infoCallback(self, data):
		self.loadcellQuery = data
		self.is_loadcellQuery = 1
		self.lastTime_checkloadcellQuery = time.time()

	def int_to_byte(self, val): # int to a bytes
		if val > 255:
			rospy.logerr("int_to_byte: Val error: %s", val)
			val = 255
		elif val < 0:
			rospy.logerr("int_to_byte: Val error: %s", val)
			val = 0
		return struct.pack("B", int(val)) # bytes(chr(int(val)), 'ascii')

	def int_to_bytes(self, val, n):
		ss = b''
		x = 0
		t = 0
		for i in range(0, n):
			t += pow(256, n - i)*x
			x = int((val - t)/pow(256, n - i - 1) )
			ss += self.int_to_byte(x)
		return ss

	def convert_(self):
		byte_ = b'\xE7\x00\x05\x01' # 93001D01
		# bytes_as_bits = ''.join(format(byte, '08b')[::-1] for byte in byte_) # dao bit 0 <-> 1
		bytes_as_bits = ''.join(format(byte, '08b') for byte in byte_) # 
		print ("bit: ", bytes_as_bits)

	def pubRespondLoadcell(self, status, weight, arrBit):
		loadcellRespond = Loadcell_respond()
		loadcellRespond.header.stamp = rospy.Time.now()
		loadcellRespond.status = status
		if status == 1:
			mess = '|'
			loadcellRespond.weight = weight
			if arrBit[0] == '1':
				mess = mess + ' LOST LOADCELL 4 |'
			if arrBit[1] == '1':
				mess = mess + ' LOST LOADCELL 3 |'
			if arrBit[2] == '1':
				mess = mess + ' LOST LOADCELL 2 |'
			if arrBit[3] == '1':
				mess = mess + ' LOST LOADCELL 1 |'
			
			if mess == '|':
				loadcellRespond.message_info = 'ALL LOADCELL STILL ALIVE'
			else:
				loadcellRespond.message_info = mess

		elif status == -1:
			loadcellRespond.weight = 0.0
			loadcellRespond.message_info = 'ERROR'

		else:
			loadcellRespond.weight = 0.0
			loadcellRespond.message_info = 'WAIT DATA REQUEST'

		self.pub_dataLoadCell.publish(loadcellRespond)
		
	def read_weight(self):
		try:
			time.sleep(0.4)
			rawData1 = self.MODBUS.execute(self.ID_loadcell, cst.READ_HOLDING_REGISTERS, self.reg_lc1, 1)
			time.sleep(0.4)
			rawData2 = self.MODBUS.execute(self.ID_loadcell, cst.READ_HOLDING_REGISTERS, self.reg_lc2, 1)
			time.sleep(0.4)
			rawData3 = self.MODBUS.execute(self.ID_loadcell, cst.READ_HOLDING_REGISTERS, self.reg_lc3, 1)
			print ("rawData3: ", rawData3)
			# print("data1: " + str(rawData1[0]) + " | data2: " + str(rawData2[0]) )
			# print("data1: " + str(self.int_to_bytes(rawData1[0], 2) ) + " | data2: " + str(self.int_to_bytes(rawData2[0], 2) ) + " | data3: " + str(rawData3[0]))
			st = b''
			a1 = b''
			a2 = b''
			a1 = self.int_to_bytes(rawData1[0], 2)
			# print(a1)

			a10 = self.int_to_byte(a1[0])
			a11 = self.int_to_byte(a1[1])

			a2 = self.int_to_bytes(rawData2[0], 2)
			a20 = self.int_to_byte(a2[0])
			a21 = self.int_to_byte(a2[1])

			# --
			st += a11
			st += a10
			st += a21
			st += a20

			value = struct.unpack('f', st)[0]
			temp = '000' + format(rawData3[0], "b")
			temp = temp[len(temp)-4:]
			# print(temp)

			print ("weight: ", value)
			sts = 0
			if (rawData3[0] != 0):
				sts = -1
			else:
				sts = 1
			self.pubRespondLoadcell(sts, value, temp)

		except:
			print ("Read LOADCELL reg_feedback error")
			self.pubRespondLoadcell(-1, 0.0, '')

	def run(self):
		while not rospy.is_shutdown():
			self.read_weight()

			self.rate.sleep()

def main():
	print('Starting main program')
	program = ReadLoadCell()
	program.run()

if __name__ == '__main__':
	main()
