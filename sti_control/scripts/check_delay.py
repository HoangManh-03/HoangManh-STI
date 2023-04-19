#!/usr/bin/env python3
# Author : Hoang Van Quang - BEE
# Date: 20-10-2020

import os
import rospy
import time
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class check_delay:
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('check_delay', anonymous= True)
		self.rate = rospy.Rate(10)

		rospy.Subscriber('/odometry', Odometry, self.odometry_callback)
		self.odometry = Odometry()

		rospy.Subscriber('/nav350laser/odom', Odometry, self.nav350laser_callback)
		self.nav350_odom = Odometry()

		rospy.Subscriber('/robotPose_nav', PoseStamped, self.robotPose_nav_callback)
		self.robotPose_nav = PoseStamped()

		self.pub_setPose = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size= 10)
		self.set_pose = PoseWithCovarianceStamped()

		self.count = 0

	def odometry_callback(self, data):
		self.odometry = data

	def robotPose_nav_callback(self, data):
		self.robotPose_nav = data

	def nav350laser_callback(self, data):
		self.nav350_odom = data
		self.compare_cooridate()

		if (self.count == 0):
			self.setpose_right()
			self.count = 1

	def setpose_right(self):
		self.set_pose.header.frame_id = 'frame_odom'
		# self.set_pose.header.stamp = rospy.Time.now()
		self.set_pose.pose.pose.position.x = self.robotPose_nav.pose.position.x
		self.set_pose.pose.pose.position.y = self.robotPose_nav.pose.position.y

		self.set_pose.pose.pose.orientation.x = self.robotPose_nav.pose.orientation.x
		self.set_pose.pose.pose.orientation.y = self.robotPose_nav.pose.orientation.y
		self.set_pose.pose.pose.orientation.z = self.robotPose_nav.pose.orientation.z
		self.set_pose.pose.pose.orientation.w = self.robotPose_nav.pose.orientation.w

		self.set_pose.pose.covariance[0] = 0.001
		self.set_pose.pose.covariance[7] = 0.001
		self.set_pose.pose.covariance[35] = 0.001

		self.pub_setPose.publish(self.set_pose)

	def compare_cooridate(self): 
		print ("--------------------------")
		print ("    |   odometry   |   robotPose_nav")
		print (" T  | " + str(self.odometry.header.stamp) + " | " +  str(self.robotPose_nav.header.stamp))
		print (" X  | " + str(round(self.odometry.pose.pose.position.x, 3) ) + " | " + str(round(self.robotPose_nav.pose.position.x, 3) ) )
		print (" Y  | " + str(round(self.odometry.pose.pose.position.y, 3) ) + " | " + str(round(self.robotPose_nav.pose.position.y, 3) ) )
		print (" V  | " + str(round(self.odometry.twist.twist.linear.x, 3) ) + " | " + str(round(self.odometry.twist.twist.angular.z, 3) ) )

	def doIt(self):
		
		while not rospy.is_shutdown():

			self.rate.sleep()

def main():
	print('Program starting')

	program = check_delay()
	program.doIt()

	print('Programer stopped')

if __name__ == '__main__':
    main()