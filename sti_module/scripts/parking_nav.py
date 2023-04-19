#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# DATE: 22/10/2021
# AUTHOR: HOANG VAN QUANG - BEE

import rospy
import sys
import time
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# from message_pkg.msg import Parking_request, Parking_respond
from sti_msgs.msg import *
from message_pkg.msg import *

from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int16


class ValueCompleted:
	def __init__(self, dis = 0.0, ang = 0.0):
		self.distance = dis # m
		self.angle = ang    # rad

	# def get_data(self):
	# 	print(f'{self.distance}+{self.angle}j')

class parking():
	def __init__(self):
		print("ROS Initial: navigation!")
		rospy.init_node('parking', anonymous= False) # False
		
		# -- -- -- parameter
		self.control_topic = rospy.get_param("control_topic", '/cmd_vel')
		self.control_frequence = rospy.get_param("control_frequence", 60)
		self.debug = rospy.get_param("debug", 1)
		self.rate = rospy.Rate(self.control_frequence)
		# self.rate = rospy.Rate(100)
		# --
		self.linear_max = rospy.get_param("linear_max", 0.7) # m/s
		self.linear_max = 0.6
		self.linear_min = rospy.get_param("linear_min", 0.01)
		self.linear_min = 0.08
		# --
		self.rotation_max = rospy.get_param("rotation_max", 0.4) # rad/s
		self.rotation_max = 0.3
		# -
		self.rotation_min = rospy.get_param("rotation_min", 0.02) # 0.06
		self.rotation_min = 0.1
		# --
		self.accuracy_position = rospy.get_param("accuracy_position", 0.008) # m
		self.accuracy_angle = rospy.get_param("accuracy_angle", radians(0.5)) # rad
		# --
		self.deceleration_distance = rospy.get_param("deceleration_distance", 0.4) # m
		self.deceleration_distance = 0.4
		self.deceleration_angle = rospy.get_param("deceleration_angle", radians(24)) # rad 30
		self.deceleration_angle = radians(20)
		# --
		self.acceleration_distance = rospy.get_param("acceleration_distance", 0.1) # m
		self.acceleration_angle = rospy.get_param("acceleration_angle", radians(30)) # rad

		# -- -- -- 
		self.pub_cmdVel = rospy.Publisher(self.control_topic, Twist, queue_size= 10)
		self.cmd_vel = Twist()
		# -- show path plan
		self.pub_pathGlobal = rospy.Publisher("/path_global", Path, queue_size= 10)
		self.pub_pathLocal = rospy.Publisher("/path_local", Path, queue_size = 10)
		self.pub_markerArray = rospy.Publisher("/path_marker", MarkerArray, queue_size=10)
		
		# --
		self.pub_moveRespond = rospy.Publisher("/move_respond", Move_respond, queue_size=10)
		self.move_respond = Move_respond()
		# --
		rospy.Subscriber("/parking_request", Parking_request, self.callback_query)
		self.parking_request = Parking_request()
		self.parking_request = Parking_request()
		self.pre_moveQuery = Parking_request()
		self.isRead_query = 0
		# -- 
		# rospy.Subscriber("/zone_safety", Zone_safety, self.callback_zone)
		# self.zone_safety = Zone_safety()
		# self.isRead_zone = 1
		# --
		rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback_robotPose)
		self.robot_pose = PoseStamped()
		self.isRead_robotPose = 0
		# --
		# self.valueCompleted_level0 = ValueCompleted(0., self.accuracy_angle)
		self.valueCompleted_level1 = ValueCompleted(self.accuracy_position, self.accuracy_angle)
		self.valueCompleted_level2 = ValueCompleted(self.accuracy_position, self.accuracy_angle)
		# -- 
		self.angleThreshold_mustRotation = radians(30) # rad
		# -- thông số khi đi thẳng.
		self.angleThreshold_adjustment = radians(0.4) # rad - ngưỡng điều chỉnh khi đi thẳng.
		# -- -- -- 
		self.locateGoal = 0 # vị trí trong danh sách điểm đang chạy.
		self.locateGoal_next = 0

		self.type_run = 0
		# -- 
		self.step_run = 0
		self.notification = ''
		self.status = 0
		self.is_queryChangeTarget = 0
		self.is_completed_listPoint = 0 # su dung khi chay het cac diem co trong danh sach. Nhung chua toi dich
		self.is_completed_all = 0
		# -- 
		self.preTime_show = time.time()
		# --
		self.step_fineTuning = 0
		self.completed_fineTuning_x = 0
		self.completed_fineTuning_y = 0
		self.choose_axis = 0
		self.angle_target = 0.0
		self.angleRobot = 0.0
		# -- 
		self.step_goGo = 0
		self.delta_distance = 0.0
		self.delta_angle = 0.0
		# --
		self.angle_target = 0.0
		self.requir_type = 0
		# -- 
		self.step_parking = -1
		# -- new
		self.listPose_save = []
		self.is_geted_poseFilter = 0
		self.poseFilter = Pose()
		self.preTime_checkGetPose = time.time()
		self.poseBefore = Pose()
		self.poseAfter = Pose()
		self.delta_distane_backware = 0.0

	# def callback_zone(self, data):
	# 	self.zone_safety = data
	# 	self.isRead_zone = 1

	def callback_query(self, data):
		# self.move_query = data
		self.move_query_readed = data
		self.isRead_query = 1
		self.is_completed_listPoint = 0

	def callback_robotPose(self, data):
		self.robot_pose = data
		self.isRead_robotPose = 1
		self.is_geted_pose, self.poseFilter = self.filter_PoseRobot(5, data)

	def constrain(self, input_val, min_val, max_val):
		if (input_val > max_val):
			return max_val
		if (input_val < min_val):
			return min_val
		return input_val
	
	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def show_localPlan(self, type_run):
		time_now = rospy.Time.now()
		frame_id = "odom"
		path_now = Path()
		path_now.header.stamp = time_now
		path_now.header.frame_id = frame_id	
		# -- add positon robot
		poseStamped_now = PoseStamped()
		poseStamped_now.header.stamp = time_now
		poseStamped_now.header.frame_id = frame_id
		poseStamped_now.pose.position.x = self.robot_pose.pose.position.x
		poseStamped_now.pose.position.y = self.robot_pose.pose.position.y
		poseStamped_now.pose.orientation = self.robot_pose.pose.orientation
		path_now.poses.append(poseStamped_now)
		if (type_run):
			# -- add positon goal
			poseStamped_now = PoseStamped()
			poseStamped_now.header.stamp = time_now
			poseStamped_now.header.frame_id = frame_id
			poseStamped_now.pose.position.x = self.tag_goal.pose.position.x
			poseStamped_now.pose.position.y = self.tag_goal.pose.position.y
			poseStamped_now.pose.orientation = self.tag_goal.pose.orientation
			path_now.poses.append(poseStamped_now)
		# -- publish
		self.pub_pathLocal.publish(path_now)

	def show_globalPlan(self, move_query):
		# -- show plan
		maker_array = MarkerArray()
		scale = Vector3()
		scale.x = 0
		scale.y = 0
		scale.z = 0.1
		color1 = ColorRGBA()
		color1.r = 0.0
		color1.g = 1.0
		color1.b = 0.0
		color1.a = 1.0
		color2 = ColorRGBA()
		color2.r = 1.0
		color2.g = 0.0
		color2.b = 0.0
		color2.a = 1.0
		frame_id = "odom"
		name_space = "tag"
		type_maker = Marker.TEXT_VIEW_FACING
		# --
		time_now = rospy.Time.now()
		path_now = Path()
		path_now.header.stamp = time_now
		path_now.header.frame_id = frame_id

		for lc in range(5):
			if (move_query.tags[lc].id != 0):
				# -- show plan
				poseStamped_now = PoseStamped()
				poseStamped_now.header.stamp = time_now
				poseStamped_now.header.frame_id = frame_id
				poseStamped_now.pose.position.x = move_query.tags[lc].pose.position.x
				poseStamped_now.pose.position.y = move_query.tags[lc].pose.position.y
				poseStamped_now.pose.orientation = move_query.tags[lc].pose.orientation
				path_now.poses.append(poseStamped_now)
				# -- show maker
				
				maker_now = Marker()
				maker_now.header.stamp = time_now
				if (move_query.tags[lc].id == move_query.tag_taget):
					maker_now.color = color1
				else:
					maker_now.color = color2

				maker_now.scale = scale					
				maker_now.header.frame_id = frame_id
				maker_now.ns = name_space
				maker_now.id = move_query.tags[lc].id
				maker_now.type = type_maker
				maker_now.pose.position.x = move_query.tags[lc].pose.position.x
				maker_now.pose.position.y = move_query.tags[lc].pose.position.y
				maker_now.pose.position.z = 0.1
				maker_now.pose.orientation = move_query.tags[lc].pose.orientation
				maker_now.text = str(move_query.tags[lc].id)
				maker_array.markers.append(maker_now)

		self.pub_pathGlobal.publish(path_now)
		self.pub_markerArray.publish(maker_array)

	def show_plan(self):
		t = time.time() - self.preTime_show
		if (t > 0.4):
			self.preTime_show = time.time()
			self.show_localPlan(1)
			self.show_globalPlan(self.move_query_readed)

	def calculateAngle_twoPoint(self, point_1, point_2): # point_1 | point_2 | geometry_msgs/Point | out | radian
		delta_x = point_2.x - point_1.x
		delta_y = point_2.y - point_1.y
		if (delta_x == 0):
			if (delta_y >= 0):
				return pi/2.
			else:
				return pi*(-1/2.)
		else:
			return atan2(delta_y, delta_x)

	def compare_points(p1, p2, val): # Point()
		del_x = abs(p2.x - p1.x)
		del_y = abs(p2.y - p1.y)
		if (del_x <= val and del_y <= val):
			return 1
		else:
			return 0
		
	def conpareCoordinate_Poses(self, pose1, pose2):
		delta_x = abs(pose1.position.x - pose2.position.x)
		delta_y = abs(pose1.position.y - pose2.position.y)
		if (delta_x > 0.01 or delta_y > 0.01):
			return 0
		return 1

	""" 
	pose_now, pose_target | geometry_msgs/Pose
	type_run:
		0 | simple run.
		1 | accuracy run. 
	"""
	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = quat[0]
		quat.y = quat[1]
		quat.z = quat[2]
		quat.w = quat[3]
		return quat

	def limmit_corner(self, angle_in):
		angle_out = 0.0
		angle_out = angle_in
		if (abs(angle_in) >= pi):
			if (angle_in >= 0):
				angle_out = (pi*2 - abs(angle_in))*(-1)
			else:
				angle_out = pi*2 - abs(angle_in)		
		return angle_out

	def idenify_caseMove(self, angle_robot, angle_toGoal, angle_toGoal_next, tag_goal, tag_target, isHave_tagNext):
		type_run = 0
		# -- tinh goc lech
		# angle_toGoal_next = self.calculateAngle_twoPoint(self.tag_goal.pose.position, self.tag_goal_next.pose.position)
		# angle_toGoal = self.calculateAngle_twoPoint(self.robot_pose.pose.position, self.tag_goal.pose.position)
		# angle_robot
		corner = angle_toGoal_next - angle_toGoal	
		corner = self.limmit_corner(corner)

		# -- xac dinh truong hop di chuyen.
		if (tag_goal == tag_target): # -- Di chuyển điểm cuối.
			type_run = 1

		else: # -- Di chuyển qua các điểm thường.
			if (isHave_tagNext == -1 or isHave_tagNext == 0): # -- Không tồn tại điểm tiếp theo (3)
				type_run = 3

			else: # -- Tồn tại điểm tiếp theo.
				if (abs(corner) > pi/18. and abs(corner) < pi*0.75): 
					type_run = 4 # -- Điểm tiếp theo tạo thành góc vuông(4)

				elif (abs(corner) >= pi*0.75): 
					type_run = 4 # -- Điểm tiếp theo tạo thành góc 180 độ (4)

				else: # -- Điểm tiếp theo tạo thành góc 0. (5)
					type_run = 5 # -- Robot co phuong trung voi huong hien tai

		return type_run

	def isNear_axis(self, point_robot, point_goal, dis):
		d_x = abs(point_robot.x - point_goal.x)
		d_y = abs(point_robot.y - point_goal.y)
		if (d_x <= dis or d_y <= dis):
			return 1
		else:
			return 0

	def interpolate_poseTagret(self, poseTarget, offset):
		tagret_interpolate = Pose()
		angle = self.quaternion_to_euler(poseTarget.orientation)
		tagret_interpolate.position.x = poseTarget.position.x + cos(angle)*offset
		tagret_interpolate.position.y = poseTarget.position.y + sin(angle)*offset
		tagret_interpolate.orientation = poseTarget.orientation

		return tagret_interpolate

	# def differrence_from_targetSpecial(self, pose_first, pose_end):
	# 	pose_out = Pose()
	# 	angle_first = self.quaternion_to_euler(pose_first.orientation)
	# 	angle_end = self.quaternion_to_euler(pose_end.orientation)
	# 	angle_end_to_first = self.calculateAngle_twoPoint(pose_end, pose_first)
	# 	# --
	# 	deltaDis = self.calculate_distance(pose_first.position, pose_end.position)
	# 	# -- 
	# 	deltaAngle_end_with_etf = angle_end_to_first - angle_end
	# 	deltaAngle_end_with_etf = self.limmit_corner(deltaAngle_end_with_etf)
	# 	# --
	# 	deltaAngle_end_with_first = angle_end - angle_first
	# 	deltaAngle_end_with_first = self.limmit_corner(deltaAngle_end_with_first)
	# 	# --
	# 	pose_out.position.x = deltaDis*cos(deltaAngle_end_with_etf)
	# 	pose_out.position.y = deltaDis*sin(deltaAngle_end_with_etf)

	# 	return theta_x, theta_y

	def fineTuning_position(self, pose_target, valueCompleted): # -- axis_prioritized: 0 (X) - 1 (Y)
		sts = 0
		# -- -- Tinh toan tong the
		if (self.step_fineTuning == 0):
			# -- Tinh goc hien tai robot.
			quat = ( self.robot_pose.pose.orientation.x,\
				self.robot_pose.pose.orientation.y,\
				self.robot_pose.pose.orientation.z,\
				self.robot_pose.pose.orientation.w )
			a, b, self.angleRobot = euler_from_quaternion(quat)

			# -- Tinh goc hien tai muc tieu.
			quat = ( pose_target.orientation.x,\
				pose_target.orientation.y,\
				pose_target.orientation.z,\
				pose_target.orientation.w )
			a, b, self.angleFinal = euler_from_quaternion(quat)

			# -- Do lech Truc X, Truc Y.
			difference_x = pose_target.position.x - self.robot_pose.pose.position.x
			difference_y = pose_target.position.y - self.robot_pose.pose.position.y
			# -- Xac dinh chinh Truc nao truoc (Do lech lon hon -> uu tien).
			
			if (abs(self.angleFinal) <= pi*0.25): # -- goc cuoi la truc X => xoay Y truoc
				self.choose_axis = 1
				if (self.angleRobot >= 0):
					self.angle_target = pi/2.0
				else:
					self.angle_target = -pi/2.0
					# print ("here 1")
				
			elif (abs(self.angleFinal) >= pi*0.75):
				self.choose_axis = 1
				if (self.angleRobot >= 0):
					self.angle_target = pi/2.0
				else:
					self.angle_target = -pi/2.0

			else: # -- goc cuoi la truc Y => xoay X truoc
				self.choose_axis = 0
				self.angle_target = 0.0

			self.step_fineTuning = 1
			# print ("self.choose_axis: ", self.choose_axis)
			# print ("self.angleRobot: ", self.angleRobot)
			# print ("self.angle_target: ", self.angle_target)

		# -- -- Run
		elif (self.step_fineTuning == 1):
			# print ("choose_axis: ", self.choose_axis)
			if (self.completed_fineTuning_x == 0 and self.completed_fineTuning_y == 0):
				if (self.choose_axis == 0):
					self.step_fineTuning = 2
				else:
					self.step_fineTuning = 5

			elif (self.completed_fineTuning_x == 1 and self.completed_fineTuning_y == 0):
				self.step_fineTuning = 5
				self.angle_target = self.angleFinal
				self.choose_axis = 0

			elif (self.completed_fineTuning_x == 0 and self.completed_fineTuning_y == 1):
				self.step_fineTuning = 2
				self.angle_target = self.angleFinal

			elif (self.completed_fineTuning_x == 1 and self.completed_fineTuning_y == 1):
				self.step_fineTuning = 8 # 8
				self.preTime_setPose = time.time()  

		elif (self.step_fineTuning == 2): # -- X
			difference_x = pose_target.position.x - self.robot_pose.pose.position.x
			# print ("pose_target_x: ", pose_target.position.x)
			# print ("position_x: ", self.robot_pose.pose.position.x)

			if (abs(difference_x) <= valueCompleted.distance):
				self.completed_fineTuning_x = 1
				self.step_fineTuning = 1
			else:
				self.step_fineTuning = 3

		elif (self.step_fineTuning == 3): # -- X
			# -- Tinh goc hien tai.
			quat = ( self.robot_pose.pose.orientation.x,\
				self.robot_pose.pose.orientation.y,\
				self.robot_pose.pose.orientation.z,\
				self.robot_pose.pose.orientation.w )
			a, b, self.angleRobot = euler_from_quaternion(quat)

			self.delta_angle = self.angle_target - self.angleRobot
			self.delta_angle = self.limmit_corner(self.delta_angle)

			# -- Quay
			self.cmd_vel.linear.x = 0
			if (abs(self.delta_angle) <= valueCompleted.angle):
				self.cmd_vel.angular.z = 0
				self.step_fineTuning = 4
			else:
				coefficient_angle = abs(self.delta_angle)/self.deceleration_angle
				angulerVel_l1 = coefficient_angle*self.rotation_max
				angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
				
				if (self.delta_angle >= 0):
					self.cmd_vel.angular.z = angulerVel_l1
				else:
					self.cmd_vel.angular.z = -angulerVel_l1

			# print ("------rotation X---------")
			# print ("angle_target: ", self.angle_target)
			# print ("delta_angle: ", self.delta_angle)
			# print ("angular_z: ", self.cmd_vel.angular.z)

		elif (self.step_fineTuning == 4): # -- X
			self.cmd_vel.angular.z = 0.0

			point_1 = Point(self.robot_pose.pose.position.x, 0, 0)
			point_2 = Point(pose_target.position.x, 0, 0)
			angle_betweent = self.calculateAngle_twoPoint(point_1, point_2)

			difference_x = pose_target.position.x - self.robot_pose.pose.position.x
			# print ("pose_target_x: ", pose_target.position.x)
			# print ("position_x: ", self.robot_pose.pose.position.x)

			if (abs(difference_x) <= valueCompleted.distance):
				self.completed_fineTuning_x = 1
				self.step_fineTuning = 1
				self.angle_target = 0.0
				self.cmd_vel.angular.z = 0.0
			else:
				dif_angle = self.angle_target - angle_betweent

				if (abs(dif_angle) < pi/4.):
					self.cmd_vel.linear.x = 0.012
				else:
					self.cmd_vel.linear.x = -0.012

			# print ("------move Xs---------")
			# print ("angle_target: ", self.angle_target)
			# print ("difference_x: ", difference_x)
			# print ("angle_betweent: ", angle_betweent)
			# print ("linear_x: ", self.cmd_vel.linear.x)

			# self.cmd_vel.linear.x = 0			

		elif (self.step_fineTuning == 5): # -- Y
			difference_y = pose_target.position.y - self.robot_pose.pose.position.y
			if (abs(difference_y) <= valueCompleted.distance):
				self.completed_fineTuning_y = 1
				self.step_fineTuning = 1
			else:
				self.step_fineTuning = 6

		elif (self.step_fineTuning == 6): # -- Y
			# -- Tinh goc hien tai.
			quat = ( self.robot_pose.pose.orientation.x,\
				self.robot_pose.pose.orientation.y,\
				self.robot_pose.pose.orientation.z,\
				self.robot_pose.pose.orientation.w )
			a, b, self.angleRobot = euler_from_quaternion(quat)

			self.delta_angle = self.angle_target - self.angleRobot
			self.delta_angle = self.limmit_corner(self.delta_angle)

			# print ("------Y---------")
			# print ("angle_target: ", self.angle_target)
			# print ("angleRobot: ", self.angleRobot)
			# print ("delta_angle: ", self.delta_angle)

			# -- Quay
			self.cmd_vel.linear.x = 0
			if (abs(self.delta_angle) <= valueCompleted.angle):
				self.cmd_vel.angular.z = 0
				self.step_fineTuning = 7
			else:
				coefficient_angle = abs(self.delta_angle)/self.deceleration_angle
				angulerVel_l1 = coefficient_angle*self.rotation_max
				angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
				
				if (self.delta_angle >= 0):
					self.cmd_vel.angular.z = angulerVel_l1
				else:
					self.cmd_vel.angular.z = -angulerVel_l1

		elif (self.step_fineTuning == 7): 
			self.cmd_vel.angular.z = 0.0
			angle_betweent = self.calculateAngle_twoPoint(self.robot_pose.pose.position, pose_target.position)

			difference_y = pose_target.position.y - self.robot_pose.pose.position.y
			if (abs(difference_y) <= valueCompleted.distance):
				self.completed_fineTuning_y = 1
				self.step_fineTuning = 1
				self.angle_target = 0.0
				self.cmd_vel.linear.x = 0.0
				self.cmd_vel.angular.z = 0.0
			else:
				if (self.angle_target < 0):
					if (difference_y < 0):
						self.cmd_vel.linear.x = 0.012
					else:
						self.cmd_vel.linear.x = -0.012
				else:
					if (difference_y < 0):
						self.cmd_vel.linear.x = -0.012
					else:
						self.cmd_vel.linear.x = 0.012

			# print ("------move Y---------")
			# # print ("angle_target: ", self.angle_target)
			# print ("difference_y: ", difference_y)
			# print ("angle_betweent: ", angle_betweent)
			# print ("linear_x: ", self.cmd_vel.linear.x)
			# print ("---------------")

		elif (self.step_fineTuning == 8):
			# -- setpose
			self.cmd_vel = Twist()
			self.setpose_requir.data = 1
			# print ("seting poseTag!")
			delta_time = (time.time() - self.preTime_setPose)%60
			if (delta_time >= 0.8): # or thieu topic respond
				self.setpose_requir.data = 0
				self.step_fineTuning = 9

		elif (self.step_fineTuning == 9):
			# -- Tinh goc hien tai.
			quat = ( self.robot_pose.pose.orientation.x,\
				self.robot_pose.pose.orientation.y,\
				self.robot_pose.pose.orientation.z,\
				self.robot_pose.pose.orientation.w )
			a, b, self.angleRobot = euler_from_quaternion(quat)

			# -- Tinh goc hien tai muc tieu.
			quat = ( pose_target.orientation.x,\
				pose_target.orientation.y,\
				pose_target.orientation.z,\
				pose_target.orientation.w )
			a, b, self.angleFinal = euler_from_quaternion(quat)

			self.delta_angle = self.angleFinal - self.angleRobot
			self.delta_angle = self.limmit_corner(self.delta_angle)

			# -- Quay
			self.cmd_vel.linear.x = 0
			if (abs(self.delta_angle) <= valueCompleted.angle):
				self.cmd_vel.angular.z = 0
				self.step_fineTuning = 10
				self.preTime_setPose = time.time()
				# print ("completed time 1!")
			else:
				coefficient_angle = abs(self.delta_angle)/self.deceleration_angle
				angulerVel_l1 = coefficient_angle*self.rotation_max
				angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
				
				if (self.delta_angle >= 0):
					self.cmd_vel.angular.z = angulerVel_l1 # angulerVel_l1
				else:
					self.cmd_vel.angular.z = -angulerVel_l1 # -angulerVel_l1

			# print ("-- fine-tuning time 1 --")
			# print ("angleFinal: ", self.angleFinal)
			# print ("angleRobot: ", self.angleRobot)
			# print ("delta_angle: ", self.delta_angle)
			# print ("angular.z: ", self.cmd_vel.angular.z)

		elif (self.step_fineTuning == 10):
			# -- setpose
			self.cmd_vel = Twist()
			self.setpose_requir.data = 1
			# print ("seting poseTag!")
			delta_time = (time.time() - self.preTime_setPose)%60
			if (delta_time >= 0.8): # or thieu topic respond
				self.setpose_requir.data = 0
				self.step_fineTuning = 11

		elif (self.step_fineTuning == 11):
			# -- Tinh goc hien tai.
			quat = ( self.robot_pose.pose.orientation.x,\
				self.robot_pose.pose.orientation.y,\
				self.robot_pose.pose.orientation.z,\
				self.robot_pose.pose.orientation.w )
			a, b, self.angleRobot = euler_from_quaternion(quat)

			# -- Tinh goc hien tai muc tieu.
			quat = ( pose_target.orientation.x,\
				pose_target.orientation.y,\
				pose_target.orientation.z,\
				pose_target.orientation.w )
			a, b, self.angleFinal = euler_from_quaternion(quat)

			self.delta_angle = self.angleFinal - self.angleRobot
			self.delta_angle = self.limmit_corner(self.delta_angle)

			# print ("------Y---------")
			# print ("angle_target: ", self.angle_target)
			# print ("angleRobot: ", self.angleRobot)
			# print ("delta_angle: ", self.delta_angle)

			# -- Quay
			self.cmd_vel.linear.x = 0
			if (abs(self.delta_angle) <= valueCompleted.angle):
				self.cmd_vel.angular.z = 0
				self.step_fineTuning = 12
				self.preTime_setPose = time.time()
				# print ("completed time 2!")
			else:
				coefficient_angle = abs(self.delta_angle)/self.deceleration_angle
				angulerVel_l1 = coefficient_angle*self.rotation_max
				angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
				
				if (self.delta_angle >= 0):
					self.cmd_vel.angular.z = angulerVel_l1 # angulerVel_l1
				else:
					self.cmd_vel.angular.z = -angulerVel_l1 # -angulerVel_l1

			# print ("-- fine-tuning time 2 --")
			# print ("angleFinal: ", self.angleFinal)
			# print ("angleRobot: ", self.angleRobot)
			# print ("delta_angle: ", self.delta_angle)
			# print ("angular.z: ", self.cmd_vel.angular.z)

		elif (self.step_fineTuning == 12):
			# -- setpose
			self.cmd_vel = Twist()
			self.setpose_requir.data = 1
			# print ("seting poseTag!")
			delta_time = (time.time() - self.preTime_setPose)%60
			if (delta_time >= 0.8): # or thieu topic respond
				self.setpose_requir.data = 0
				self.step_fineTuning = 13

		elif (self.step_fineTuning == 13):
			# -- Tinh goc hien tai.
			quat = ( self.robot_pose.pose.orientation.x,\
				self.robot_pose.pose.orientation.y,\
				self.robot_pose.pose.orientation.z,\
				self.robot_pose.pose.orientation.w )
			a, b, self.angleRobot = euler_from_quaternion(quat)

			# -- Tinh goc hien tai muc tieu.
			quat = ( pose_target.orientation.x,\
				pose_target.orientation.y,\
				pose_target.orientation.z,\
				pose_target.orientation.w )
			a, b, self.angleFinal = euler_from_quaternion(quat)

			self.delta_angle = self.angleFinal - self.angleRobot
			self.delta_angle = self.limmit_corner(self.delta_angle)

			# print ("------Y---------")
			# print ("angle_target: ", self.angle_target)
			# print ("angleRobot: ", self.angleRobot)
			# print ("delta_angle: ", self.delta_angle)

			# -- Quay
			self.cmd_vel.linear.x = 0
			if (abs(self.delta_angle) <= valueCompleted.angle):
				self.cmd_vel.angular.z = 0
				self.step_fineTuning = 14
				self.preTime_setPose = time.time()
				# print ("completed time 3!")
			else:
				coefficient_angle = abs(self.delta_angle)/self.deceleration_angle
				angulerVel_l1 = coefficient_angle*self.rotation_max
				angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
				
				if (self.delta_angle >= 0):
					self.cmd_vel.angular.z = angulerVel_l1 # angulerVel_l1
				else:
					self.cmd_vel.angular.z = -angulerVel_l1 # -angulerVel_l1

			# print ("-- fine-tuning time 2 --")
			# print ("angleFinal: ", self.angleFinal)
			# print ("angleRobot: ", self.angleRobot)
			# print ("delta_angle: ", self.delta_angle)
			# print ("angular.z: ", self.cmd_vel.angular.z)

		elif (self.step_fineTuning == 14):
			# -- setpose
			self.cmd_vel = Twist()
			self.setpose_requir.data = 1
			# print ("seting poseTag!")
			delta_time = (time.time() - self.preTime_setPose)%60
			if (delta_time >= 0.2): # or thieu topic respond
				self.setpose_requir.data = 0
				self.step_fineTuning = 15

		elif (self.step_fineTuning == 15):
			self.cmd_vel.linear.x = 0.0
			self.cmd_vel.angular.z = 0.0
			self.completed_fineTuning_x = 0
			self.completed_fineTuning_y = 0
			sts = 1

		# print ("angle_target: ", self.angle_target)
		# print ("step_fineTuning: ", self.step_fineTuning)
		# self.pub_cmdVel.publish(self.cmd_vel)
		return sts

	def filter_PoseRobot(self, times, poseRobot_in):
		poseRobot_out = Pose()

		if (len(self.listPose_save) >= 5):
			total_x = 0.0
			total_y = 0.0
			total_g = 0.0
			angle = 0.0

			for no in range(5):
				total_x += self.listPose_save[no].position.x
				total_y += self.listPose_save[no].position.y
				total_g += self.quaternion_to_euler(self.listPose_save[no].orientation)

				poseRobot_out.position.x = total_x/5.
				poseRobot_out.position.y = total_y/5.
				poseRobot_out.orientation = self.euler_to_quaternion(total_g/5.)
			return 1, poseRobot_out
		else:
			return 0, Pose()

	def parking(self, target_pose, offset): # target_pose
		linearVel_l1 = 0.0
		linearVel_l2 = 0.0
		angulerVel_l1 = 0.0
		angulerVel_l2 = 0.0

		if (self.step_parking == -1): # -- reset all
			self.preTime_checkGetPose = time.time()
			self.cmd_vel = Twist()
			self.listPose_save = []
			self.is_geted_poseFilter = 0

			self.step_parking = 0
			
		if (self.step_parking == 0): # -- getPose robot
			self.cmd_vel = Twist()
			delta_time = (time.time() - self.preTime_checkGetPose)%60

			# if (self.is_geted_poseFilter == 1):
			self.step_parking = 1

			if (delta_time >= 4): # -- ERROR get pose 
				self.step_parking = 11

		elif (self.step_parking == 1):	# -- kiem tra do chinh xac cua toa do robot asu khi loc
			# -- dang lam ...
			self.step_parking = 2

		elif (self.step_parking == 2): # -- kiem tra khoang cach
			delta_dis = self.calculate_distance(self.robot_pose.pose.position, target_pose.position)
			if (abs(delta_dis) < self.valueCompleted_level1.distance):
				self.step_parking = 6
			else:
				self.step_parking = 3	

		elif (self.step_parking == 3): # -- Quay
			# -- 
			angle_robot = self.quaternion_to_euler(self.robot_pose.pose.orientation)
			angle_to_goal = self.calculateAngle_twoPoint(self.robot_pose.pose.position, target_pose.position)
			print ("----------------------------")
			print ("angle_robot: ", degrees(angle_robot))
			print ("angle_to_goal: ", degrees(angle_to_goal))
			# -- 
			delta_angle = angle_to_goal - angle_robot
			delta_angle = self.limmit_corner(delta_angle)
			print ("delta_angle: ", degrees(delta_angle))
			# -- xac dinh do lon 
			coefficient_angle = abs(delta_angle)/self.deceleration_angle
			if (coefficient_angle > 1):
				coefficient_angle = 1.0
			
			angulerVel_l1 = self.rotation_max*coefficient_angle
			angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
			# -- xac dinh chieu
			if (delta_angle >= 0):
				angulerVel_l2 = angulerVel_l1
			else:
				angulerVel_l2 = angulerVel_l1*(-1)

			if (abs(delta_angle) < radians(0.3)):	
				angulerVel_l2 = 0.0
				self.step_parking = 4

			print ("angulerVel_l2: ", round(angulerVel_l2, 3))

		elif (self.step_parking == 4): # -- Tinh chinh quay
			# -- dang lam ...
			self.step_parking = 5

		elif (self.step_parking == 5): # -- di thang
			# -- -- Linear
			delta_dis = self.calculate_distance(self.robot_pose.pose.position, target_pose.position)
			coefficient_linear = abs(delta_dis)/self.deceleration_distance
			if (coefficient_linear >= 1.0):
				coefficient_linear = 1.0
			linearVel_l1 = coefficient_linear*self.linear_max

			linearVel_l2 = self.constrain(linearVel_l1, self.linear_min, self.linear_max)
			
			if (abs(delta_dis) < self.valueCompleted_level1.distance):
				print ("Ok position")
				self.step_goGo = 6
				linearVel_l2 = 0.0
				
		elif (self.step_parking == 6): # -- quay huong cuoi lan 1
			# -- 
			angle_robot = self.quaternion_to_euler(self.robot_pose.pose.orientation)
			angle_to_goalFinal = self.quaternion_to_euler(target_pose.orientation)
			# -- 
			delta_angle = angle_to_goalFinal - angle_robot
			delta_angle = self.limmit_corner(delta_angle)
			# -- xac dinh do lon 
			coefficient_angle = abs(delta_angle)/self.deceleration_angle
			if (coefficient_angle > 1):
				coefficient_angle = 1.0
			
			angulerVel_l1 = self.rotation_max*coefficient_angle
			angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
			# -- xac dinh chieu
			if (delta_angle >= 0):
				angulerVel_l2 = angulerVel_l1
			else:
				angulerVel_l2 = angulerVel_l1*(-1)

			if (abs(delta_angle) < self.valueCompleted_level1.angle):	
				angulerVel_l2 = 0.0
				self.step_parking = 7

		elif (self.step_parking == 7): # -- quay huong cuoi lan 2
			# -- 
			angle_robot = self.quaternion_to_euler(self.robot_pose.pose.orientation)
			angle_to_goalFinal = self.quaternion_to_euler(self.target_pose.orientation)
			# -- 
			delta_angle = angle_to_goalFinal - angle_robot
			delta_angle = self.limmit_corner(delta_angle)
			# -- xac dinh do lon 
			coefficient_angle = abs(delta_angle)/self.deceleration_angle
			if (coefficient_angle > 1):
				coefficient_angle = 1.0
			
			angulerVel_l1 = self.rotation_max*coefficient_angle
			angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
			# -- xac dinh chieu
			if (delta_angle >= 0):
				angulerVel_l2 = angulerVel_l1
			else:
				angulerVel_l2 = angulerVel_l1*(-1)

			if (abs(delta_angle) < self.valueCompleted_level1.angle):	
				angulerVel_l2 = 0.0
				self.step_parking = 8

		elif (self.step_parking == 8): # -- tim diem cuoi.
			self.poseBefore = self.robot_pose.pose
			self.poseAfter = self.interpolate_poseTagret(target_pose, offset)
			
			self.step_parking = 9

		elif (self.step_parking == 9): # -- lui vao diem kieu 1: dung odom.
			delta_distane_backware = self.calculate_distance(self.poseBefore, self.poseAfter)
			# -- -- Linear
			coefficient_linear = abs(delta_distane_backware)/(0.15)
			if (coefficient_linear >= 1.0):
				coefficient_linear = 1.0

			linearVel_l1 = coefficient_linear*(0.3)
			linearVel_l2 = self.constrain(linearVel_l1, self.linear_min, 0.3)*(-1)
			# --
			if (abs(delta_distane_backware) < self.valueCompleted_level1.distance):
				print ("Back Ok position!")
				self.step_parking = 10
				linearVel_l2 = 0.0

		elif (self.step_parking == 10): # -- stop.
			linearVel_l2 = 0.0
			angulerVel_l2 = 0.0

		self.cmd_vel.linear.x = linearVel_l2
		self.cmd_vel.angular.z = angulerVel_l2

		self.pub_cmdVel.publish(self.cmd_vel)
		print ("self.step_parking: ", self.step_parking)

	def run(self):
		print ("Launch ALL!")

		# self.tester = 1
		self.poseA = Pose()

		self.poseA.position.x = 2.103
		self.poseA.position.y = -2.846
		self.poseA.orientation.z = -0.674
		self.poseA.orientation.w = 0.738

		while not rospy.is_shutdown():
			# self.process()
			if (self.isRead_robotPose == 1):
				self.parking(self.poseA, 1.8)

			self.rate.sleep()
		self.pub_cmdVel.publish(Twist())
		time.sleep(0.01)
		self.pub_cmdVel.publish(Twist())
		time.sleep(0.01)
		self.pub_cmdVel.publish(Twist())
		

def main():
	print('Starting main program')
	program = parking()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()

"""
kiểu di chuyến:
A, Di chuyển điểm cuối. 
	I, Tinh chỉnh tự do. (1)
	II, Tinh chỉnh theo góc vuông. (2)
B, Di chuyển qua các điểm thường.
	I, Không tồn tại điểm tiếp theo (3)
	II, Tồn tại điểm tiếp theo.
		1, Điểm tiếp theo tạo thành góc vuông hoặc 180 độ (4)
		2, Điểm tiếp theo tạo thành góc 0. (5)
----
Trạng thái:
1, Đang thực hiện yêu cầu dừng. (0)
2, Đang thực hiện lệnh khởi tạo lại. (1)
3, Đang thực hiện lệnh di chuyển: 
	I, Di chuyển bình thường. (2)
		a, Quay tại chỗ.
		b, Tiến.
	II, Dừng:
		a, Do Đã chạy hết điểm. (3)
		b, Do vật cản. (4)
		c, Lỗi định dạng lệnh. (5)
		d, Đã hoàn thành. (6)


"""
