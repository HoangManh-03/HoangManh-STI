#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Authors : BEE
# kinematic.
# DATE: 02/03/2022
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal
import tf
import PyKDL
from math import pi
from geometry_msgs.msg import PoseStamped, Quaternion

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class calibPose():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('calibPose', anonymous=False) # False

		# -- parameter
		self.rate = rospy.Rate(1)
		self.coefficient_x = rospy.get_param('coefficient_x', 0.0)
		self.coefficient_y = rospy.get_param('coefficient_y', 0.024)
		self.coefficient_z = rospy.get_param('coefficient_z', 0.0)
		self.coefficient_angle = rospy.get_param('coefficient_angle', 0.018) # -- rad

		print ("coe_x: ", self.coefficient_x)
		print ("coe_y: ", self.coefficient_y)
		print ("coe_z: ", self.coefficient_z)
		print ("coe_angle: ", self.coefficient_angle)

		# -- SUB raw_imu_bno055
		# rospy.Subscriber('/robotPose_origin', PoseStamped, self.pose_callback)
		rospy.Subscriber('/robotPose_nav', PoseStamped, self.pose_callback)
		self.pose_origin = PoseStamped()

		# self.pub_poseCalib = rospy.Publisher("/robotPose_nav", PoseStamped, queue_size= 10)
		self.pub_poseCalib = rospy.Publisher("/poseCheck", PoseStamped, queue_size= 10)
		self.poseCalib = PoseStamped()

	def pose_callback(self, data):
		self.pose_origin = data
		
		self.poseCalib.header = self.pose_origin.header
		self.poseCalib.pose.position.x = self.pose_origin.pose.position.x + self.coefficient_x
		self.poseCalib.pose.position.y = self.pose_origin.pose.position.y + self.coefficient_y
		self.poseCalib.pose.position.z = self.pose_origin.pose.position.z + self.coefficient_z

		angle_origin = self.quaternion_to_euler(self.pose_origin.pose.orientation)
		angle_calib = angle_origin + self.coefficient_angle

		limit_angle = self.limit_angle(angle_calib)
		self.poseCalib.pose.orientation = self.euler_to_quaternion(limit_angle)
		print ("DO: ", degrees(angle_calib) )
		self.pub_poseCalib.publish(self.poseCalib)

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def limit_angle(self, euler_in):
		euler_out = 0.0
		if (abs(euler_in) <= pi):
			euler_out = euler_in
		else:
			if (abs(euler_in) > 2*pi):
				euler_out = 0
			else:
				if (euler_in >= 0):
					euler_out = -pi + (euler_in%pi)
				else:
					euler_out = pi - abs(euler_in)%pi
		return euler_out

	def run(self):
		print ("Launch ALL!")
		while not rospy.is_shutdown(): # not self.shutdown_flag.is_set() or 
			# print ("RAD: ", self.limit_angle(radians(184)) )
			# print ("pi: ", radians(-184))
			# print ("4 DO: ", radians(176))
			self.rate.sleep()

		print('program stopped')

def main():
	print('Starting main program')

	program = calibPose()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()

