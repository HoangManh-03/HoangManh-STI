#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 28/06/2023
"""

import socketio
import json 

import socket
import roslib

# get ip
import os                                                                                                                                                           
import re 

import sys
import struct
import time
from decimal import *
import math
import rospy
from datetime import datetime

from sti_msgs.msg import NN_cmdRequest   
from sti_msgs.msg import NN_infoRequest  
from sti_msgs.msg import NN_infoRespond 

from sti_msgs.msg import FL_infoRespond
from sti_msgs.msg import FL_cmdRespond
from sti_msgs.msg import FL_infoRequest
from sti_msgs.msg import FL_cmdRequest

# import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point


class Communicate_socketIO():
	def __init__(self):
		rospy.init_node('Communicate_socketIO', anonymous=False)
		self.rate = rospy.Rate(10)
		# -- 
		self.name_card = rospy.get_param("name_card", "wlp0s20f3")
		self.name_card = "wlo2" # "wlp0s20f3"
		self.server_IP = '192.168.1.99'
		self.server_port = 3000

		self.AGV_IP = '192.168.1.100'
		self.AGV_port = 6000

		# -
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.AGVInfor_callback)	
		self.AGV_information = NN_infoRespond()
		self.is_AGVInfor = 0
		# -
		self.NN_cmdPub = rospy.Publisher("/NN_cmdRequest", NN_cmdRequest, queue_size=50)
		self.server_sendCommand = NN_cmdRequest()
		# -
		self.NN_infoRequestPub = rospy.Publisher("/NN_infoRequest", NN_infoRequest, queue_size=50)
		self.server_requestInfor = NN_infoRequest()
		# ---------
		
	def AGVInfor_callback(self, data):
		self.AGV_information = data
		self.is_AGVInfor = 1

	def run(self):

		self.rate.sleep()

def main():
	print('Starting main program')
    # - Start the job threads
	myObject = Communicate_socketIO()		
	my_socketIO = socketio.Client()

	# my_socketIO.connect('http://' + myObject.server_IP +':' + str(myObject.server_port))
	# -
	is_connected = 0
	time_save = time.time()

	@my_socketIO.on('Server-request-AGV')
	def on_message(data):
		print('I received a message!')
		print(data)

	@my_socketIO.event
	def connect():
		print("I'm connected!")
		is_connected = 1

	@my_socketIO.event
	def connect_error(data):
		print("The connection failed!")

	@my_socketIO.event 
	def disconnect():
		print("I'm disconnected!")

	# - Keep the main thread running, otherwise signals are ignored.
	while not rospy.is_shutdown():
		myObject.run()
		# -
		if is_connected == 0:
			delta_t = (time.time() - time_save)%60
			if delta_t > 0.6:
				time_save = time.time()
				try:
					my_socketIO.connect('http://' + myObject.server_IP +':' + str(myObject.server_port))
				except Exception:
					pass

if __name__ == '__main__':
    main()