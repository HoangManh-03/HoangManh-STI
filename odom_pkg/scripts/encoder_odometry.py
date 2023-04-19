#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Authors : BEE
# kinematic.
# DATE: 21/06/2021
# EDIT: 16/10/2021
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

from math import sin , cos , pi , atan2

class encoderOdometry():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('encoder_odometry', anonymous=False) # False

		# -- parameter
		self.topic_subVel = rospy.get_param("topic_subVel", "/raw_vel")
		self.topic_pubOdom = rospy.get_param("topic_pubOdom", "/raw_odom")
		self.frame_id = rospy.get_param("frame_id", "/frame_odom")
		self.child_frame_id = rospy.get_param("child_frame_id", "frame_robot1")
		self.enb_pub_tf = rospy.get_param("enb_pub_tf", 0)

		self.rate = rospy.Rate(1)
		# -- SUB
		rospy.Subscriber(self.topic_subVel, Velocities, self.sub_callback)
		self.raw_vel = Velocities()

		# -- PUB
		self.pub_odometry = rospy.Publisher(self.topic_pubOdom, Odometry, queue_size= 50)
		self.odometry = Odometry()

		# --
		self.preTime_pub = time.time()
		# -- 
		self.preTime_rawVel = rospy.Time.now()
		self.timeout = 0.06
		# -- 
		self.odometry.header.frame_id = self.frame_id
		self.odometry.child_frame_id = self.child_frame_id

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

	def sub_callback(self, data):
		# --
		current_time = rospy.Time.now()
		delta_time = (current_time - self.preTime_rawVel).to_sec()
		self.preTime_rawVel = current_time
		# --
		if (delta_time <= self.timeout):
			delta_angle = data.angular_z*delta_time
			delta_x = data.linear_x*cos(self.agl_z)*delta_time
			delta_y = data.linear_x*sin(self.agl_z)*delta_time
		# --
			self.pos_x += delta_x
			self.pos_y += delta_y
			self.agl_z += delta_angle
		else:
			print "Timeout!"

		# --
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.agl_z)

		# -- use to calibration
		if (self.enb_pub_tf):
			self.odom_broadcaster.sendTransform(
			    (self.pos_x, self.pos_y, 0.),
			    odom_quat,
			    current_time,
			    self.child_frame_id,
			    self.frame_id
			)
		# -- 
		self.odometry.header.stamp = current_time
		self.odometry.pose.pose = Pose(Point(self.pos_x, self.pos_y, 0.), Quaternion(odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]))
		self.odometry.twist.twist = Twist(Vector3(data.linear_x, 0, 0), Vector3(0, 0, data.angular_z))

		self.pub_odometry.publish(self.odometry)

	def run(self):
		print "Launch ALL!"
		while not rospy.is_shutdown(): # not self.shutdown_flag.is_set() or 

			self.rate.sleep()
		print('program stopped')

def main():
	print('Starting main program')

	program = encoderOdometry()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()
