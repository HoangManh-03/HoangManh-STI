#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# DATE: 22/10/2021
# AUTHOR: HOANG VAN QUANG - BEE

import rospy
import sys
import time
import math

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist

class filterPose_nav():
	def __init__(self):
		print("ROS Initial: filterPose_nav!")
		rospy.init_node('filterPose_nav', anonymous = False) # False

		self.rate = rospy.Rate(1)
		
		rospy.Subscriber("/odometry", Odometry, self.odometry_callback)
		self.odometry = Odometry()
		
		self.pub_setSpeed = rospy.Publisher("/odometry/filtered", Odometry, queue_size = 10) # /odometry/setSpeed
		self.setSpeed = Odometry()

	def odometry_callback(self, data):
		self.odometry = data
		
		self.setSpeed.header.stamp = rospy.Time.now()
		# self.setSpeed.header.frame_id = ""
		self.setSpeed.twist = self.odometry.twist
		
		self.pub_setSpeed.publish(self.setSpeed)
		
	def run(self):
		while not rospy.is_shutdown():
			self.rate.sleep()

def main():
	print('Starting main program')
	program = convert_setSpeed()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()