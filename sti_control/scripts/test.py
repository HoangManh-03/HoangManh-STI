#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

def int_to_bytes( val, n): # int to n bytes
	ss = b''
	x = 0
	t = 0
	for i in range(0, n):
		t += pow(256, n - i)*x
		print ("-----------i: ", i)
		print ("tt: ", t)
		x = int((val - t)/pow(256, n - i - 1) )
		print ("xx: ", x)
		# ss += self.int_to_byte(x)
	return ss

def int_to_byte(val): # int to a bytes
	if val > 255:
		# rospy.logerr("int_to_byte: Val error: %s", val)
		val = 255
	elif val < 0:
		# rospy.logerr("int_to_byte: Val error: %s", val)
		val = 0
	return struct.pack("B", int(val)) # bytes(chr(int(val)), 'ascii')

def intA_to_bytes(val, n): # int to n bytes
	ss = b''
	x = 0
	t = 0

	if (val >= 0):
		val_1 = val
	else:
		val_1 = pow(256, 4) + val

	for i in range(0, n):
		t += pow(256, n - i)*x
		x = int((val_1 - t)/pow(256, n - i - 1) )
		ss += int_to_byte(x)

	print(ss)
	return ss

# int_to_bytes(15000, 4)

a = intA_to_bytes(500, 2)

for i in a:
	print(i)

# t = 0
# x = (150000 - t)/pow(256, 4 - 1 - 1) 

# print ("x: ", x)
