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

# from sti_msgs.msg import Velocities
from sensor_msgs.msg import Imu

from math import sin , cos , pi , atan2

class calib_imu():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('calib_imu', anonymous=False) # False

		# -- parameter
		self.rate = rospy.Rate(1)
		# -- SUB raw_imu_bno055  imu_filter
		rospy.Subscriber('/imu/data', Imu, self.imu_callback)
		self.imu_data = Imu()

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

	def imu_callback(self, data):
		# --
		current_time = rospy.Time.now()
		delta_time = (current_time - self.preTime_rawVel).to_sec()
		self.preTime_rawVel = current_time
		# --
		# if (delta_time <= self.timeout):
		vel_z = data.angular_velocity.z - data.angular_velocity.z*0.03392
		delta_angle = vel_z*delta_time
		# delta_angle = data.angular_velocity.z*delta_time
		# delta_angle = delta_angle - delta_angle*0.03392
		# --
		self.agl_z += delta_angle
		# else:
		# 	print "Timeout!"
		# --
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.agl_z)
		# odom_angle = tf.transformations.euler_from_quaternion(odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3])
		odom_angle = tf.transformations.euler_from_quaternion(odom_quat)
		print ("z: ", round(odom_angle[2], 3))
		# --
		self.odom_broadcaster.sendTransform(
		    (0.0, 0.0, 0.0),
		    odom_quat,
		    current_time,
		    self.child_frame_id,
		    self.frame_id
		)
		# -- 
		self.odometry.header.stamp = current_time
		self.odometry.pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]))
		self.odometry.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, data.angular_velocity.z))

		self.pub_odometry.publish(self.odometry)

	def run(self):
		print "Launch ALL!"
		while not rospy.is_shutdown(): # not self.shutdown_flag.is_set() or 

			self.rate.sleep()
		print('program stopped')

def main():
	print('Starting main program')

	program = calib_imu()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()

