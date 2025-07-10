#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dev: Archie Phung
Date Modify: 07/03/2024

For purpose:
	1. Find position of robot with a translation and rotation from Lidar position which use equation
	(Fake the lookupTransform function)
	   >> NOT OK, 

"""
import rospy
import math 

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_pkg.msg import *
from sti_msgs.msg import *

import numpy as np
from scipy.spatial.transform import Rotation

# -0.404 -0.042 -1 0.036 0 0 frame_nav350 frame_robot
class PoseRobot():
	def __init__(self):
		print("ROS Initial: Find_PoseRobot!")
		rospy.init_node('robotPose_equation', anonymous = False, disable_signals=True) # False

		self.rate = rospy.Rate(10)

		rospy.Subscriber("/nav350laser/odom", Odometry, self.callback_laserPos)
		self.laser_pose = Odometry()

		# self.pub_robotPose = rospy.Publisher("/robotPose_equation", PoseStamped, queue_size=50)
		# self.robotPose = PoseStamped()

		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.flag_run = 0
		self.is_exit = 0

	def callback_laserPos(self, data):
		self.laser_pose = data
		self.x = self.laser_pose.pose.pose.position.x
		self.y = self.laser_pose.pose.pose.position.y
		self.z = self.laser_pose.pose.pose.position.z
		self.flag_run = 1

	def callback_tfstatic(self, data):
		self.tfstatic = data

	def shutdown(self):
		self.is_exit = 1

	def get_posRb(self):
		try:
			if self.is_exit == 0:
				while not rospy.is_shutdown():
					P1 = np.array([self.x, self.y, self.z])
					# Translation vector
					T = np.array([-0.404,-0.042,-1])
					# Rotation matrix
					r = Rotation.from_euler('xyz', [0, 0, 2.062648062], degrees=True)
					R = r.as_matrix()
					# Translate
					P_translated = P1 + T

					# Rotate
					P2 = np.dot(R, P_translated)

					print("The position of the point after translation and rotation is ", P2)

					self.rate.sleep()
		except KeyboardInterrupt:
			rospy.on_shutdown(self.shutdown)
			self.is_exit = 1
			print('!!FINISH!!')

def main():
	print('Starting main program')
	program = PoseRobot()
	program.get_posRb()
	print('Exiting main program')	

if __name__ == '__main__':
    main()