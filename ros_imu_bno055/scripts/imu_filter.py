#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Authors : BEE
# kinematic.
# DATE: 21/06/2021
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal
import tf
import PyKDL
from sensor_msgs.msg import Imu
from math import pi

class filter_imu():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('filter_imu', anonymous=False) # False

		# -- parameter
		self.rate = rospy.Rate(1)
		self.error_threshold = rospy.get_param('error_threshold', 0.0015)
		self.error_coefficient = rospy.get_param('error_coefficient', 0.03392)
		# -- SUB raw_imu_bno055
		rospy.Subscriber('/imu/data', Imu, self.imu_callback)
		self.imu_data = Imu()

		# self.orientation_now = PyKDL.Rotation.Quaternion(0, 0, 0, 1)
		# self.orientation_pre = PyKDL.Rotation.Quaternion(0, 0, 0, 1)
		# self.timeRead_pre = time.time()
 		# self.delta_time = 0.0
		# self.max_delta = 0.0

		self.pub_imuFilter = rospy.Publisher("imu_filter", Imu, queue_size= 50)
		self.imuFilter = Imu()

		self.countTime = 0

	def imu_callback(self, data):
		self.imuFilter = data
		val_z = data.angular_velocity.z
		if (abs(val_z) < self.error_threshold):   #0.0015
			val_z = 0.0
		else:
			val_z = val_z - val_z*self.error_coefficient  #0.03392
		self.imuFilter.angular_velocity.z = val_z
		
		self.pub_imuFilter.publish(self.imuFilter)

	def filter_old(self, data):
		self.delta_time = time.time() - self.timeRead_pre
		self.timeRead_pre = time.time()
        # -- 
		self.orientation_now = PyKDL.Rotation.Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

		quat = self.orientation_now * self.orientation_pre.Inverse()
		self.orientation_pre = self.orientation_now
        # -- 
		roll, pitch, yaw1 = quat.GetRPY()  # get roll pitch yaw result
        # -- 
		vel1 = yaw1*(1/self.delta_time)
		vel2 = data.angular_velocity.z
		delta =  vel1 - vel2		

		self.imuFilter = data
		if (abs(vel1) < 0.002):
			vel1 = 0.0
		self.imuFilter.angular_velocity.z = vel1
		
		if (self.countTime > 2):
			self.pub_imuFilter.publish(self.imuFilter)
		else:
			self.countTime += 1

	def run(self):
		print ("Launch ALL!")
		while not rospy.is_shutdown(): # not self.shutdown_flag.is_set() or 

			self.rate.sleep()

		print('program stopped')

def main():
	print('Starting main program')

	program = filter_imu()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()

