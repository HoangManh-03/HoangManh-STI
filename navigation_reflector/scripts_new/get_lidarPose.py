#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    - Author: Archie
    - Date: 2025-04-08
    - Description: Get pos of lidar after each transform

"""

import rospy
import sys
import time
import math
import tf
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist

class get_robotPose():
	def __init__(self):
		print("ROS Initial: get_info_lidar!")
		rospy.init_node('get_info_lidar', anonymous = False) # False

		self.rate = rospy.Rate(10)

		self.map_frame = "/frame_map_nav350"
		self.base_frame = "/world"
		
		self.listener = tf.TransformListener()
		# --
		# rospy.Subscriber("/nav350laser/odom", Odometry, self.odometryNAV_callback)
		# -
		self.pub_info_lidar = rospy.Publisher("/lidar_pose", PoseStamped, queue_size = 10)
		self.lidar_info = PoseStamped()

	# def odometryNAV_callback(self, data):
	# 	try:
	# 		now = rospy.Time.now()
	# 		self.listener.waitForTransform(self.map_frame, self.base_frame, now, rospy.Duration(1.0))
	# 		(trans, rot) = self.listener.lookupTransform(self.map_frame, self.base_frame, now)

	# 		# --
	# 		self.robotPose_nav.header.frame_id = self.base_frame
	# 		self.robotPose_nav.header.stamp = now
	# 		# -
	# 		self.robotPose_nav.pose.orientation.x = rot[0]
	# 		self.robotPose_nav.pose.orientation.y = rot[1]
	# 		self.robotPose_nav.pose.orientation.z = rot[2]
	# 		self.robotPose_nav.pose.orientation.w = rot[3]
	# 		# -
	# 		self.robotPose_nav.pose.position.x = trans[0]
	# 		self.robotPose_nav.pose.position.y = trans[1]
	# 		self.robotPose_nav.pose.position.z = trans[2]

	# 		# --
	# 		self.pub_robotPose.publish(self.robotPose_nav)

	# 	except (tf.Exception, tf.LookupException, tf.ConnectivityException):
	# 		print ("Error")
		
	def run(self):
		# now = rospy.Time.now()
		# self.listener.waitForTransform(self.map_frame, self.base_frame, now, rospy.Duration(1.0))

		while not rospy.is_shutdown():
			try:
				now = rospy.Time.now()
				self.listener.waitForTransform(self.map_frame, self.base_frame, now, rospy.Duration(4.0))
				(trans, rot) = self.listener.lookupTransform(self.map_frame, self.base_frame, now)

				# --
				self.lidar_info.header.frame_id = self.base_frame
				self.lidar_info.header.stamp = now
				# -
				self.lidar_info.pose.orientation.x = rot[0]
				self.lidar_info.pose.orientation.y = rot[1]
				self.lidar_info.pose.orientation.z = rot[2]
				self.lidar_info.pose.orientation.w = rot[3]
				# -
				self.lidar_info.pose.position.x = trans[0]
				self.lidar_info.pose.position.y = trans[1]
				self.lidar_info.pose.position.z = trans[2]

				# --
				self.pub_info_lidar.publish(self.lidar_info)

			except (tf.Exception, tf.LookupException, tf.ConnectivityException):
				print ("Error")
				
			self.rate.sleep()

def main():
	print('Starting main program')
	program = get_robotPose()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()