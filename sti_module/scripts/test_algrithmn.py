#!/usr/bin/env python3
import rospy
import sys
import time
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# from sti_msgs.msg import Move_query, Move_respond
# from sti_msgs.msg import Tag_node, Zone_safety, Tag_info

from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Twist, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int16

def calculateAngle_twoPoint( point_1, point_2): # point_1 | point_2 | geometry_msgs/Point | out | radian
	delta_x = point_2.x - point_1.x
	delta_y = point_2.y - point_1.y
	if (delta_x == 0):
		if (delta_y >= 0):
			return pi/2.
		else:
			return pi*(-1/2.)
	else:
		return atan2(delta_y, delta_x)

def limmit_corner(angle_in):
	angle_out = 0.0
	angle_out = angle_in
	if (abs(angle_in) >= pi):
		if (angle_in >= 0):
			angle_out = (pi*2 - abs(angle_in))*(-1)
		else:
			angle_out = pi*2 - abs(angle_in)		
	return angle_out

def calculate_distance(p1, p2): # p1, p2 | geometry_msgs/Point
	x = p2.x - p1.x
	y = p2.y - p1.y
	return sqrt(x*x + y*y)
	
def quaternion_to_euler( qua):
	quat = (qua.x, qua.y, qua.z, qua.w )
	a, b, euler = euler_from_quaternion(quat)
	return euler

def euler_to_quaternion( euler):
	quat_out = Quaternion()
	quat = quaternion_from_euler(0, 0, euler)
	quat_out.x = quat[0]
	quat_out.y = quat[1]
	quat_out.z = quat[2]
	quat_out.w = quat[3]
	return quat_out
	
def differrence_from_targetSpecial( pose_first, pose_end):
	pose_out = Pose()
	angle_first = quaternion_to_euler(pose_first.orientation)
	print ("angle_first: ", degrees(angle_first) )
	angle_end = quaternion_to_euler(pose_end.orientation)
	print ("angle_end: ", degrees(angle_end) )
	angle_end_to_first = calculateAngle_twoPoint(pose_end.position, pose_first.position)
	print ("angle_end_to_first: ", degrees(angle_end_to_first) )
	# --
	deltaDis = calculate_distance(pose_first.position, pose_end.position)
	print ("deltaDis: ", deltaDis)
	# -- 
	deltaAngle_end_with_etf = angle_end_to_first - angle_end #- angle_end_to_first
	print ("deltaAngle_end_with_etf1: ", degrees(deltaAngle_end_with_etf) )
	deltaAngle_end_with_etf = limmit_corner(deltaAngle_end_with_etf)
	print ("deltaAngle_end_with_etf: ", degrees(deltaAngle_end_with_etf) )
	
	# --
	deltaAngle_end_with_first = angle_first - angle_end
	deltaAngle_end_with_first = limmit_corner(deltaAngle_end_with_first)
	print ("deltaAngle_end_with_first: ", degrees(deltaAngle_end_with_first) )
	# --
	pose_out.position.x = deltaDis*cos(deltaAngle_end_with_etf)
	pose_out.position.y = deltaDis*sin(deltaAngle_end_with_etf)

	print ("x: ", pose_out.position.x)
	print ("y: ", pose_out.position.y)

	theta_x = pose_out.position.x
	theta_y = pose_out.position.y

	# return theta_x, theta_y, theta_g

pose_first = Pose()
pose_end = Pose()

pose_first.position.x = 0
pose_first.position.y = 0
pose_first.orientation = euler_to_quaternion(radians(0))

pose_end.position.x = 1
pose_end.position.y = 2
pose_end.orientation = euler_to_quaternion(radians(-90))

# pose_first.position.x = 1
# pose_first.position.y = 1
# pose_first.orientation = euler_to_quaternion(radians(0))

# pose_end.position.x = 1.86602
# pose_end.position.y = 1.5
# pose_end.orientation = euler_to_quaternion(radians(150))

differrence_from_targetSpecial(pose_first, pose_end)