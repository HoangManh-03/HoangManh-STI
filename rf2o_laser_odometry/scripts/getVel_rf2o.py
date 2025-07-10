#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sti_msgs.msg import *
import math
import time
import threading
import signal
import os

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class getVel_pose():
	def __init__(self):
		print("ROS Initial: getVel_pose!")
		rospy.init_node('getVel_pose', anonymous = False) # False
		self.rate = rospy.Rate(1)
		# -- 
		# ------------------------ nav350 ------------------------------------ #
		rospy.Subscriber('/odom_nav350', Odometry, self.callback_odomFf2o_nav350)
		self.odom_ = Odometry()
		self.is_readOdom_ = 0
		self.timeStampe_odomFf2o = rospy.Time.now()
        # -
		self.posenav350_now = Pose()
		self.saveTime_nav350_now = time.time()
        # -
		self.posenav350_bef = Pose()
		self.saveTime_nav350_bef = time.time()

		# -- 
		self.pub_odomnav350 = rospy.Publisher("/nav350_odomRf2o", Odometry, queue_size = 60)
		self.nav350_odomRf2o = Odometry()

	def callback_odomFf2o_nav350(self, data):
		self.odom_nav350 = data
		self.is_readOdom_nav350 = 1
		self.timeStampe_odomFf2o = rospy.Time.now()
		# -
		self.nav350_odomRf2o = self.odom_nav350
		self.nav350_odomRf2o.header.frame_id = "odom_nav350"
		# self.nav350_odomRf2o.header.stamp = rospy.get_rostime()
		# --
		self.posenav350_now = self.odom_nav350.pose.pose
		self.saveTime_nav350_now = time.time()
		# --
		new_robot, new_goal = self.fakePose_robotFollowGoal(self.posenav350_bef, self.posenav350_now)
		# --
		delta_x = 0.0
		delta_y = 0.0
		vel_y = 0.0
		vel_x = 0.0
		# -
		delta_x = new_robot.position.x
		delta_y = new_robot.position.y
		# --
		delta_time = 0.0
		delta_time = (self.saveTime_nav350_now - self.saveTime_nav350_bef)%60 
		# --
		if delta_time < 0.5:
			vel_y = delta_y/delta_time
			vel_x = delta_x/delta_time
		# --
		self.saveTime_nav350_bef = self.saveTime_nav350_now

		# print ("------------")
		# print ("delta_y: ", delta_y)
		# print ("delta_time: ", delta_time)

		self.posenav350_bef.position = self.posenav350_now.position
		self.posenav350_bef.orientation = self.posenav350_now.orientation
		# --
		self.nav350_odomRf2o.twist.twist.linear.x = vel_x*-1
		self.nav350_odomRf2o.twist.twist.linear.y = vel_y*-1
		self.pub_odomnav350.publish(self.nav350_odomRf2o)
		# print ("-- Y: " + str(delta_y) + " | T: " + str(delta_time))

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def limitAngle(self, angle_in): # - rad
		qua_in = self.euler_to_quaternion(angle_in)
		angle_out = self.quaternion_to_euler(qua_in)
		return angle_out

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def angleLine_AB(self, pointA, pointB): # -- Angle Line Point A to Point B. | Point()
		d_x = pointB.x - pointA.x
		d_y = pointB.y - pointA.y
		ang = 0
		if d_x == 0:
			if d_y >= 0:
				ang = pi/2.
			else:
				ang = -pi/2.
		else:
			if d_y == 0:
				if d_x > 0:
					ang = 0
				else:
					ang = pi
			else:
				ang = atan2(d_y, d_x)
				if ang > pi:
					ang = ang - 2*pi
				if ang < -pi:
					ang = 2*pi + ang
		return ang

	def fakePose_robotFollowGoal(self, robotPose, goalPose): # -- goal -> main.
		distancePoint = self.calculate_distance(robotPose.position, goalPose.position)
		# --
		angle_goalToRobot = self.angleLine_AB(goalPose.position, robotPose.position)
		# angle_robotToGoal = self.angleLine_AB(robotPose.position, goalPose.position)
		# --
		angleGoal = self.quaternion_to_euler(goalPose.orientation)
		angleRobot = self.quaternion_to_euler(robotPose.orientation)
		# -- 
		# print ("angleRobot: ", degrees(angleRobot))
		# print ("angleGoal: ", degrees(angleGoal))
		# print ("angle_goalToRobot: ", degrees(angle_goalToRobot))
		# print ("angle_robotToGoal: ", degrees(angle_robotToGoal))
		# -- 
		angleNew_goalToRobot = angle_goalToRobot - angleGoal
		angleNew_goalToRobot = self.limitAngle(angleNew_goalToRobot)
		# -- 
		# print ("angleNew_goalToRobot: ", degrees(angleNew_goalToRobot) )
		# -- 
		angleRobot_new = angleRobot - angleGoal
		angleRobot_new = self.limitAngle(angleRobot_new)
		# -- 
		poseRobot_new = Pose()
		poseRobot_new.position.x = cos(angleNew_goalToRobot)*distancePoint
		poseRobot_new.position.y = sin(angleNew_goalToRobot)*distancePoint
		poseRobot_new.orientation = self.euler_to_quaternion(angleRobot_new)

		# print ("poseRobot_new X: ", poseRobot_new.position.x)
		# print ("poseRobot_new Y: ", poseRobot_new.position.y)
		# print ("poseRobot_new R: ", degrees(angleRobot_new) )
		# print (str(round(poseGoal_new.position.x, 3)) + " | " + str(round(poseGoal_new.position.y, 3)))

		poseGoal_new = Pose()
		poseGoal_new.orientation = self.euler_to_quaternion(0)

		return poseRobot_new, poseGoal_new


	def run(self):
		while not rospy.is_shutdown():

			self.rate.sleep()
			
		# self.pub_cmdVel.publish(Twist())

def main():
	print('Starting main program')
	program = getVel_pose()
	program.run()
	print('Exiting main program')
	program.stop_run()

if __name__ == '__main__':
    main()