#!/usr/bin/env python
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

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from sti_msgs.msg import Velocities
from sensor_msgs.msg import Imu

from math import sin , cos , pi , atan2

class calib_imu():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('calib_imu', anonymous=False) # False

		# -- parameter
		self.rate = rospy.Rate(100)
		# -- SUB raw_imu_bno055
		rospy.Subscriber('/imu/data', Imu, self.imu_callback)
		self.imu_data = Imu()
		self.is_newData = 0
		self.timeRead = rospy.Time.now()
		self.delta_time = 0.0
		# -- PUB
		self.pub_odometry = rospy.Publisher("odom_imu", Odometry, queue_size= 50)
		self.odometry = Odometry()

		self.frame_id = "odom"
		self.child_frame_id = "imu_odom"
		# --
		self.preTime_pub = time.time()
		# -- 
		self.preTime_rawVel = rospy.Time.now()
		self.timeout = 0.1
		# -- 
		self.odometry.header.frame_id = "imu_link"
		self.odometry.child_frame_id = "imu_odom"

		self.odometry.pose.covariance[0] = 0.001
		self.odometry.pose.covariance[7] = 0.001
		self.odometry.pose.covariance[35] = 0.001

		self.odometry.twist.covariance[0] = 0.0001
		self.odometry.twist.covariance[7] = 0.0001
		self.odometry.twist.covariance[35] = 0.0001
		# -- 
		self.odom_broadcaster = tf.TransformBroadcaster()
		self.pos_x = 0.0
		self.pos_y = 0.0
		self.agl_z = 0.0
		# -- 
		self.max_velRoll = 0.0
		self.odom_quat = []

		self.pre_yaw = 0.0
		self.detal_yaw = 0.0
		self.max_freq = 0.0
		self.min_freq = 100.0
		self.pre_time = time.time()
		self.tb = 0.0
		self.vel = 0.0
		self.count = 1

	def imu_callback(self, data):
		current_time = rospy.Time.now()
		self.imu_data = data
		self.is_newData = 1
		# --
		self.delta_time = (current_time - self.timeRead).to_sec()
		self.timeRead = current_time
		# --
		orientation_list = [self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
		if (abs(self.imu_data.angular_velocity.z) < 0.004):
			self.detal_yaw = 0.0
			self.pre_yaw = yaw
		else:
			self.detal_yaw = self.pre_yaw - yaw
			self.pre_yaw = yaw

		self.agl_z += self.detal_yaw

	def run(self):
		print "Launch ALL!"
		while not rospy.is_shutdown(): # not self.shutdown_flag.is_set() or 
			# if (self.count > 3):
				# self.is_newData = 0
				# --
				# if (delta_time <= self.timeout):

			self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.agl_z)
			# --
			self.odom_broadcaster.sendTransform(
			    (0.0, 0.0, 0.0),
			    self.odom_quat,
			    self.timeRead,
			    self.child_frame_id,
			    self.frame_id
			)
			# -- 
			self.odometry.header.stamp = self.timeRead
			self.odometry.pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(self.odom_quat[0], self.odom_quat[1], self.odom_quat[2], self.odom_quat[3]))
			self.odometry.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, self.imu_data.angular_velocity.z))

			self.pub_odometry.publish(self.odometry)

			self.rate.sleep()
		print('program stopped')

def main():
	print('Starting main program')

	program = calib_imu()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()

