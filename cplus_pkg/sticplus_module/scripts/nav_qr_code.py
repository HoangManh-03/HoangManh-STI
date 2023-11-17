#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# DATE: 31/07/2021
# UPDATE: 25/08/2021
# AUTHOR: HOANG VAN QUANG - BEE

import rospy
import sys
import time
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sti_msgs.msg import Move_request, Move_respond
from sti_msgs.msg import Tag_node, Zone_safety, Tag_info

from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int16


"""
Quy trinh set_pose
	1, Den 1 diem dang o giua tag
	2, Dung
	3, Setpose trong vong 1 s.
	4, Chay tiep

"""

class ValueCompleted:
	def __init__(self, dis = 0.0, ang = 0.0):
		self.distance = dis # m
		self.angle = ang    # rad

	# def get_data(self):
	# 	print(f'{self.distance}+{self.angle}j')

class navigation():
	def __init__(self):
		print("ROS Initial: navigation!")
		rospy.init_node('navigation', anonymous= False) # False
		
		# -- -- -- parameter
		self.control_topic = rospy.get_param("control_topic", '/cmd_vel')
		self.control_frequence = rospy.get_param("control_frequence", 60)
		self.debug = rospy.get_param("debug", 1)
		self.rate = rospy.Rate(self.control_frequence)
		# self.rate = rospy.Rate(100)
		# --
		self.linear_max = rospy.get_param("linear_max", 0.7) # m/s
		self.linear_max = 0.24
		self.linear_min = rospy.get_param("linear_min", 0.01)
		self.linear_min = 0.012
		# --
		self.rotation_max = rospy.get_param("rotation_max", 0.5) # rad/s
		self.rotation_max = 1.0
		# -
		self.rotation_min = rospy.get_param("rotation_min", 0.02) # 0.06
		self.rotation_min = 0.02
		# --
		self.accuracy_position = rospy.get_param("accuracy_position", 0.004) # m
		self.accuracy_angle = rospy.get_param("accuracy_angle", radians(0.1)) # rad
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
		rospy.Subscriber("/move_query", Move_request, self.callback_query)
		self.move_query = Move_request()
		self.move_query_readed = Move_request()
		self.pre_moveQuery = Move_request()
		self.isRead_query = 0
		# -- 
		rospy.Subscriber("/zone_safety", Zone_safety, self.callback_zone)
		self.zone_safety = Zone_safety()
		self.isRead_zone = 1
		# --
		rospy.Subscriber("/pose_robot", PoseStamped, self.callback_robotPose)
		self.robot_pose = PoseStamped()
		self.isRead_robotPose = 0
		# --
		self.pub_setPose = rospy.Publisher("/setpose_requir", Int16, queue_size=10)
		self.setpose_requir = Int16()
		self.preTime_setPose = time.time()
		self.requir_setpose = 0
		# --
		rospy.Subscriber('/tag_info', Tag_info, self.callBack_tagInfo)
		self.tagInfo_now = Tag_info()
		# --
		# self.valueCompleted_level0 = ValueCompleted(0., self.accuracy_angle)
		self.valueCompleted_level1 = ValueCompleted(self.deceleration_distance + 0.1, radians(15))
		self.valueCompleted_level2 = ValueCompleted(self.accuracy_position, self.accuracy_angle)
		self.valueCompleted_level3 = ValueCompleted(0.002, radians(0.1))
		self.valueCompleted_level4 = ValueCompleted(0.02, radians(0.1))
		# -- 
		self.angleThreshold_mustRotation = radians(30) # rad
		# -- thông số khi đi thẳng.
		self.angleThreshold_adjustment = radians(0.4) # rad - ngưỡng điều chỉnh khi đi thẳng.
		# -- -- -- 
		self.locateGoal = 0 # vị trí trong danh sách điểm đang chạy.
		self.locateGoal_next = 0
		self.poseGoal = Pose()
		self.poseGoal_next = Pose()

		self.pre_idTarget = 0
		self.idTarget = 0

		self.type_run = 0
		self.is_centerTag = 0
		# -- 
		self.tag_goal = Tag_node()
		self.tag_goal_next = Tag_node()
		self.tag_target = Tag_node()
		# -- 
		self.step_run = 0
		self.notification = ''
		self.status = 0
		self.is_queryChangeTarget = 0
		self.is_centerTag = 0
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
		self.step_backward_to_charger = -1
		self.step_run_2_points = -1
		# -- 
		self.tester = 0
		# -- -- -- 
		self.poseExample = Pose()
		self.poseExample.position.x = 0.0
		self.poseExample.position.y = 0.0
		# -- 
		self.poseExample.orientation.x = 0
		self.poseExample.orientation.y = 0
		#
		# self.poseExample.orientation.z = 0
		# self.poseExample.orientation.w = 1
		#
		# self.poseExample.orientation.z = -0.7071068
		# self.poseExample.orientation.w = 0.7071068
		#
		self.poseExample.orientation.z = 1
		self.poseExample.orientation.w = 0
		#
		# self.poseExample.orientation.z = -0.7071068
		# self.poseExample.orientation.w = 0.7071068
		# -- 
		self.position_start = Point()
		# -- POINT 1
		self.poseA = Pose()
		self.poseB = Pose()

		self.pose1 = Pose()
		self.pose1.position.x = 0.0
		self.pose1.position.y = 0.0
		# -- 
		self.pose1.orientation.x = 0
		self.pose1.orientation.y = 0
		self.pose1.orientation.z = -0.7071068
		self.pose1.orientation.w = 0.7071068
		# -- POINT 2
		self.pose2 = Pose()
		self.pose2.position.x = 0.0
		self.pose2.position.y = -0.698 # -0.8113
		# -- 
		self.pose2.orientation.x = 0
		self.pose2.orientation.y = 0
		self.pose2.orientation.z = -0.7071068
		self.pose2.orientation.w = 0.7071068
		# -- POINT 3
		self.pose3 = Pose()
		self.pose3.position.x = 0.0
		self.pose3.position.y = -0.698 # -0.8113
		# -- 
		self.pose3.orientation.x = 0
		self.pose3.orientation.y = 0
		self.pose3.orientation.z = 0.7071068
		self.pose3.orientation.w = 0.7071068
		# -- POINT 4
		self.pose4 = Pose()
		self.pose4.position.x = 0
		self.pose4.position.y = 0.0
		# -- 
		self.pose4.orientation.x = 0
		self.pose4.orientation.y = 0
		self.pose4.orientation.z = 0.7071068
		self.pose4.orientation.w = 0.7071068

	def callBack_tagInfo(self, data):
		self.tagInfo_now = data

	def callback_zone(self, data):
		self.zone_safety = data
		self.isRead_zone = 1

	def callback_query(self, data):
		# self.move_query = data
		self.move_query_readed = data
		self.isRead_query = 1
		self.is_completed_listPoint = 0

	def callback_robotPose(self, data):
		self.robot_pose = data
		self.isRead_robotPose = 1

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

	def check_listRight(self, move_query): # Kiểm tra yêu cầu của traffic có hợp lệ không.
		lenght = len(move_query.tags)
		# print ("lenght: ", lenght)
		if (lenght == 5 and move_query.tag_taget > 0):
			return 1
		else:
			print ("len lists tag: ", lenght)
			print ("tag_taget: ", move_query.tag_taget)
			return 0

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

	def findAngle_targetSecondary(self, pose_robot, pose_target, angle_toGoal, delta_distance, distance_enb):
		angle_out = 0.0
		point_targetSecondary = Point()

		#  if (delta_distance < 0.3 ): # 
		if (self.isNear_axis(pose_robot.position, pose_target.position) == 0):
			angle_out = angle_toGoal
		else:
			if (abs(angle_toGoal) <= pi*0.25):
				# print ("0000")
				point_targetSecondary.x = self.robot_pose.pose.position.x + distance_enb
				point_targetSecondary.y = pose_target.position.y
			elif (abs(angle_toGoal) >= pi*0.75):
				# print ("180")
				point_targetSecondary.x = self.robot_pose.pose.position.x - distance_enb
				point_targetSecondary.y = pose_target.position.y
			else:
				if (angle_toGoal >= 0):
					# print ("pi/2")
					point_targetSecondary.x = pose_target.position.x
					point_targetSecondary.y = self.robot_pose.pose.position.y + distance_enb
				else:
					# print ("-pi/2")
					point_targetSecondary.x = pose_target.position.x
					point_targetSecondary.y = self.robot_pose.pose.position.y - distance_enb

			angle_out = self.calculateAngle_twoPoint(self.robot_pose.pose.position, point_targetSecondary)
		return angle_out

	def findAngle_targetSecondary_vs1(self, pose_robot, pose_target, angle_toGoal, delta_distance, distance_enb):
		angle_out = 0.0
		point_targetSecondary = Point()

		d_x = abs(pose_robot.position.x - pose_target.position.x)
		d_y = abs(pose_robot.position.y - pose_target.position.y)

		if (d_x >= d_y): # == > lay truc Y lam puong
			if (abs(angle_toGoal) <= pi*0.25):
				# print ("00")
				# d_y = abs(pose_robot.position.y - pose_target.position.y)
				if (d_y > 0.2):
					point_targetSecondary.x = pose_robot.position.x
				else:
					point_targetSecondary.x = pose_robot.position.x + (0.2 - d_y + 0.2)

				point_targetSecondary.y = pose_target.position.y

			elif (abs(angle_toGoal) >= pi*0.75):
				# print ("180")
				d_y = abs(pose_robot.position.y - pose_target.position.y)
				if (d_y > 0.2):
					point_targetSecondary.x = pose_robot.position.x
				else:
					point_targetSecondary.x = pose_robot.position.x - (0.2 - d_y + 0.2)

				point_targetSecondary.y = pose_target.position.y

		else:
			if (angle_toGoal >= 0):
				# print ("pi/2")
				point_targetSecondary.x = pose_target.position.x

				if (d_x > 0.2):
					point_targetSecondary.y = pose_robot.position.y
				else:
					point_targetSecondary.y = pose_robot.position.y + (0.2 - d_x + 0.2)

				# print ("pose_target.position: " + str(pose_target.position.x) + " | " + str(pose_target.position.y))
				# print ("point_targetSecondary: " + str(point_targetSecondary.x) + " | " + str(point_targetSecondary.y))
				
			else:
				# print ("-pi/2")
				point_targetSecondary.x = pose_target.position.x
				if (d_x > 0.2):
					point_targetSecondary.y = pose_robot.position.y
				else:
					point_targetSecondary.y = pose_robot.position.y - (0.2 - d_x + 0.2)

		angle_out = self.calculateAngle_twoPoint(pose_robot.position, point_targetSecondary)
		
		dis_now = self.calculate_distance(pose_robot.position, point_targetSecondary)
		if (dis_now/2. > delta_distance):
			if (self.isNear_axis(pose_robot.position, pose_target.position, 0.04) == 0 ):
				angle_out = angle_toGoal
				print ("here")

		# print ("point_targetSecondary: " + str(point_targetSecondary.x) + " | " + str(point_targetSecondary.y) + " | A: " + str(angle_out))
		return angle_out

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

	def fineTuning_position_vs1(self, pose_target, valueCompleted):
		# -- -- Tinh toan tong the
		if (self.step_fineTuning == 0):
			# -- Tinh goc hien tai.
			quat = ( self.robot_pose.pose.orientation.x,\
				self.robot_pose.pose.orientation.y,\
				self.robot_pose.pose.orientation.z,\
				self.robot_pose.pose.orientation.w )
			a, b, self.angleRobot = euler_from_quaternion(quat)
		
			# -- Do lech Truc X, Truc Y.
			difference_x = pose_target.position.x - self.robot_pose.pose.position.x
			difference_y = pose_target.position.y - self.robot_pose.pose.position.y
			# -- Xac dinh chinh Truc nao truoc (Do lech lon hon -> uu tien).
			
			# -- Tim huong quay gan nhat.
			if (abs(self.angleRobot) <= pi*0.25):
				self.angle_target = 0.0
				self.choose_axis = 0
			elif (abs(self.angleRobot) >= pi*0.75):
				self.choose_axis = 0
				self.angle_target = pi
			else:
				if (self.angleRobot >= 0):
					self.choose_axis = 1
					self.angle_target = pi/2.0
				else:
					self.choose_axis = 1
					self.angle_target = -pi/2.0

			self.step_fineTuning = 1
		# -- -- Run
		elif (self.step_fineTuning == 1):
			# print ("choose_axis: ", self.choose_axis)
			if (self.completed_fineTuning_x == 0 and self.completed_fineTuning_y == 0):
				if (self.choose_axis == 0):
					self.step_fineTuning = 2
				else:
					self.step_fineTuning = 4

			elif (self.completed_fineTuning_x == 1 and self.completed_fineTuning_y == 0):
				self.step_fineTuning = 4
				self.angle_target = pi/2.0

			elif (self.completed_fineTuning_x == 0 and self.completed_fineTuning_y == 1):
				self.step_fineTuning = 2
				self.angle_target = 0

			elif (self.completed_fineTuning_x == 1 and self.completed_fineTuning_y == 1):
				self.step_fineTuning = 6


		elif (self.step_fineTuning == 2): # -- X
			# -- Tinh goc hien tai.
			quat = ( self.robot_pose.pose.orientation.x,\
				self.robot_pose.pose.orientation.y,\
				self.robot_pose.pose.orientation.z,\
				self.robot_pose.pose.orientation.w )
			a, b, self.angleRobot = euler_from_quaternion(quat)
			self.delta_angle = self.angle_target - self.angleRobot

			if (abs(self.delta_angle) >= pi):
				if (self.delta_angle >= 0):
					self.delta_angle = (pi*2 - abs(self.delta_angle))*(-1)
				else:
					self.delta_angle = pi*2 - abs(self.delta_angle)	

			# -- Quay
			self.cmd_vel.linear.x = 0
			if (abs(self.delta_angle) <= valueCompleted.angle + 0.01):
				self.cmd_vel.angular.z = 0
				self.step_fineTuning = 3
			else:
				coefficient_angle = abs(self.delta_angle)/self.deceleration_angle
				angulerVel_l1 = coefficient_angle*0.6 # self.rotation_max
				angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
				
				if (self.delta_angle >= 0):
					self.cmd_vel.angular.z = angulerVel_l1
				else:
					self.cmd_vel.angular.z = -angulerVel_l1

			print ("------rotation X---------")
			print ("angle_target: ", self.angle_target)
			print ("delta_angle: ", self.delta_angle)
			print ("angular_z: ", self.cmd_vel.angular.z)

		elif (self.step_fineTuning == 3):
			self.cmd_vel.angular.z = 0.0

			point_1 = Point(self.robot_pose.pose.position.x, 0, 0)
			point_2 = Point(pose_target.position.x, 0, 0)
			angle_betweent = self.calculateAngle_twoPoint(point_1, point_2)

			difference_x = pose_target.position.x - self.robot_pose.pose.position.x
			print ("pose_target_x: ", pose_target.position.x)
			print ("position_x: ", self.robot_pose.pose.position.x)

			if (abs(difference_x) <= valueCompleted.distance):
				self.completed_fineTuning_x = 1
				self.step_fineTuning = 1
				self.angle_target = 0.0
				self.cmd_vel.angular.z = 0.0
			else:
				dif_angle = self.angle_target - angle_betweent

				if (abs(dif_angle) < pi/4.):
					self.cmd_vel.linear.x = 0.06
				else:
					self.cmd_vel.linear.x = -0.06

			print ("------move X---------")
			print ("angle_target: ", self.angle_target)
			print ("difference_x: ", difference_x)
			print ("angle_betweent: ", angle_betweent)
			print ("linear_x: ", self.cmd_vel.linear.x)

			# self.cmd_vel.linear.x = 0			

		elif (self.step_fineTuning == 4): # -- Y
			# -- Tinh goc hien tai.
			quat = ( self.robot_pose.pose.orientation.x,\
				self.robot_pose.pose.orientation.y,\
				self.robot_pose.pose.orientation.z,\
				self.robot_pose.pose.orientation.w )
			a, b, self.angleRobot = euler_from_quaternion(quat)

			self.delta_angle = self.angle_target - self.angleRobot

			if (abs(self.delta_angle) >= pi):
				if (self.delta_angle >= 0):
					self.delta_angle = (pi*2 - abs(self.delta_angle))*(-1)
				else:
					self.delta_angle = pi*2 - abs(self.delta_angle)

			print ("------Y---------")
			print ("angle_target: ", self.angle_target)
			print ("angleRobot: ", self.angleRobot)
			print ("delta_angle: ", self.delta_angle)

			# -- Quay
			self.cmd_vel.linear.x = 0
			if (abs(self.delta_angle) <= valueCompleted.angle + 0.01):
				self.cmd_vel.angular.z = 0
				self.step_fineTuning = 5
			else:
				coefficient_angle = abs(self.delta_angle)/self.deceleration_angle
				angulerVel_l1 = coefficient_angle*0.6
				angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)
				
				if (self.delta_angle >= 0):
					self.cmd_vel.angular.z = angulerVel_l1
				else:
					self.cmd_vel.angular.z = -angulerVel_l1

		elif (self.step_fineTuning == 5): 
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
						self.cmd_vel.linear.x = 0.06
					else:
						self.cmd_vel.linear.x = -0.06
				else:
					if (difference_y < 0):
						self.cmd_vel.linear.x = -0.06
					else:
						self.cmd_vel.linear.x = 0.06

			print ("------move Y---------")
			print ("angle_target: ", self.angle_target)
			print ("difference_y: ", difference_y)
			print ("angle_betweent: ", angle_betweent)
			print ("linear_x: ", self.cmd_vel.linear.x)

		elif (self.step_fineTuning == 6):
			self.cmd_vel.linear.x = 0.0
			self.cmd_vel.angular.z = 0.0
			print ("completed!")

		print ("step_fineTuning: ", self.step_fineTuning)

		self.pub_cmdVel.publish(self.cmd_vel)
		# -- Quay dung huong.
		# -- Tinh huong di chuyen.
		# -- Tien/Lui chinh vi tri Truc 1.
		# -- Quay dung huong.
		# -- Tinh huong di chuyen.
		# -- Tien/Lui chinh vi tri Truc 2.

	def fineTuning_position_vs2(self, pose_target, valueCompleted): # -- axis_prioritized: 0 (X) - 1 (Y)
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

	def fineTuning_position_vs3(self, pose_target, valueCompleted): # -- axis_prioritized: 0 (X) - 1 (Y)
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
			print ("self.choose_axis: ", self.choose_axis)
			print ("self.angleRobot: ", self.angleRobot)
			print ("self.angle_target: ", self.angle_target)

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
					self.cmd_vel.linear.x = 0.02
				else:
					self.cmd_vel.linear.x = -0.02

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
						self.cmd_vel.linear.x = 0.02
					else:
						self.cmd_vel.linear.x = -0.02
				else:
					if (difference_y < 0):
						self.cmd_vel.linear.x = -0.02
					else:
						self.cmd_vel.linear.x = 0.02

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
			if (delta_time >= 0.4): # or thieu topic respond
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
				print ("completed time 1!")
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
			if (delta_time >= 0.4): # or thieu topic respond
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
				print ("completed time 2!")
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
			if (delta_time >= 0.4): # or thieu topic respond
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
				print ("completed time 3!")
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
			delta_time = (time.time() - self.preTime_setPose)%60
			if (delta_time >= 0.2): # or thieu topic respond
				self.setpose_requir.data = 0
				self.step_fineTuning = 15

		elif (self.step_fineTuning == 15):
			self.cmd_vel.linear.x = 0.0
			self.cmd_vel.angular.z = 0.0
			sts = 1

		# print ("angle_target: ", self.angle_target)
		# print ("step_fineTuning: ", self.step_fineTuning)
		# self.pub_cmdVel.publish(self.cmd_vel)
		return sts
		# -- Quay dung huong.
		# -- Tinh huong di chuyen.
		# -- Tien/Lui chinh vi tri Truc 1.
		# -- Quay dung huong.
		# -- Tinh huong di chuyen.
		# -- Tien/Lui chinh vi tri Truc 2.

	"""
	Khoang cach dat sac xa diem it nhat 35 cm.
	
	kHOANG CACH LUC SAC: Tu AGV to Box sac: 7.2 cm
	NEN DE: 20 cm
	=> Lap dat thuc te: 28.2 cm (KC tu mep sau agv toi box sac)
	"""

	def backward_to_charger(self, dis): # target_pose
		if (self.step_backward_to_charger == -1):
			self.preTime_setPose = time.time()
			self.step_backward_to_charger = 0
			self.setpose_requir.data = 0

		if (self.step_backward_to_charger == 0):
			# -- setpose
			self.cmd_vel = Twist()
			self.setpose_requir.data = 1
			# print ("seting poseTag!")
			delta_time = (time.time() - self.preTime_setPose)%60
			if (delta_time >= 1): # or thieu topic respond
				self.step_backward_to_charger = 1
				self.setpose_requir.data = 0
				
		elif (self.step_backward_to_charger == 1):
			# -- di chuyen chin xac - huong cuoi cung ve sac
			sts = self.fineTuning_position_vs3(self.poseExample, self.valueCompleted_level3)

			if (sts == 1):
				self.position_start = self.robot_pose.pose.position
				self.step_backward_to_charger = 2

		elif (self.step_backward_to_charger == 2):

			self.step_backward_to_charger = 3

		elif (self.step_backward_to_charger == 3):
			# -- lui vao sac
			self.cmd_vel.angular.z = 0.0

			delta_distance = self.calculate_distance(self.position_start, self.robot_pose.pose.position)

			if (delta_distance < dis):
				self.cmd_vel.linear.x = -0.04
			else:
				self.cmd_vel.linear.x = 0.0
				self.step_backward_to_charger = 4
				print ("OK!")

		elif (self.step_backward_to_charger == 4):
			# -- doi
			pass
			
		self.pub_cmdVel.publish(self.cmd_vel)
		self.pub_setPose.publish(self.setpose_requir)

	def move_betweent_points(self, point_1, point_2): # Pose()
		sts_out = 0
		if (self.step_run_2_points == -1):
			print ("Point 2 - Y= ", point_2.position.y)
			self.preTime_setPose = time.time()
			self.step_run_2_points = 0
			self.setpose_requir.data = 0

		if (self.step_run_2_points == 0):
			# -- setpose
			self.cmd_vel = Twist()
			self.setpose_requir.data = 1
			# print ("seting poseTag!")
			delta_time = (time.time() - self.preTime_setPose)%60
			if (delta_time >= 0.4): # or thieu topic respond
				self.step_run_2_points = 1
				self.setpose_requir.data = 0
				
		elif (self.step_run_2_points == 1):
			# -- di chuyen chinh xac - huong cuoi cung ve sac
			sts = self.fineTuning_position_vs2(point_1, self.valueCompleted_level3)

			if (sts == 1):
				self.position_start = self.robot_pose.pose.position
				self.step_run_2_points = 2 # 

		elif (self.step_run_2_points == 2):
			# -- chosse axis
			# print ("point_2.position.y: ", point_2.position.y)
			difference_x = abs(point_2.position.x - self.robot_pose.pose.position.x)
			difference_y = abs(point_2.position.y - self.robot_pose.pose.position.y)
			if (difference_x > 0.06 and difference_y > 0.06):
				self.step_run_2_points = 6 # loi xa
			else:
				# quat = ( self.robot_pose.pose.orientation.x,\
				# 	self.robot_pose.pose.orientation.y,\
				# 	self.robot_pose.pose.orientation.z,\
				# 	self.robot_pose.pose.orientation.w )
				# a, b, self.angleRobot = euler_from_quaternion(quat)
				# print ("angle_robot: ", degrees(self.angleRobot))

				# print ("difference_x: ", difference_x)
				# print ("difference_y: ", difference_y)
				

				if (difference_x < difference_y):
					self.step_run_2_points = 3 # -- chon Y
					# print ("chon Y")
					print ("1 - Tag info: X= " + str(round(self.tagInfo_now.pose.position.x, 3)) + " |Y= " +  str(round(self.tagInfo_now.pose.position.y, 3)) + " |R= " +  str(round(self.tagInfo_now.angle, 2)) )
				else:
					self.step_run_2_points = 4 # -- chon X
					# print ("chon X")

		elif (self.step_run_2_points == 3): # -- chon Y
			# -- 
			
			delta_distance = abs(self.robot_pose.pose.position.y -  point_2.position.y)
			# print ("delta_distance X: ", delta_distance)
			if (delta_distance < 1.0 and delta_distance >= 0.598): # - tang toc
				coefficient_linear = (0.698 - delta_distance)/self.acceleration_distance
				if (coefficient_linear >= 1.0):
					coefficient_linear = 1.0
				linearVel_l1 = coefficient_linear*self.linear_max
				linearVel_l2 = self.constrain(linearVel_l1, self.linear_min, self.linear_max)
				self.cmd_vel.linear.x = linearVel_l2

			else: # - giam toc
				coefficient_linear = delta_distance/self.deceleration_distance
				if (coefficient_linear >= 1.0):
					coefficient_linear = 1.0
				linearVel_l1 = coefficient_linear*self.linear_max

				linearVel_l2 = self.constrain(linearVel_l1, self.linear_min, self.linear_max)
				self.cmd_vel.linear.x = linearVel_l2


			# -- -- Angular
			angle_goGoal = self.calculateAngle_twoPoint(self.robot_pose.pose.position, point_2.position)
			# --
			quat = ( self.robot_pose.pose.orientation.x, self.robot_pose.pose.orientation.y, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w )
			a, b, angle_robot = euler_from_quaternion(quat)
			# --
			angle_goGoal_secondary = self.findAngle_targetSecondary_vs1(self.robot_pose.pose, point_2, angle_goGoal, delta_distance, 0.0)
			# -- 
			deltaAngle_normal_2 = angle_goGoal_secondary - angle_robot
			deltaAngle_normal_2 = self.limmit_corner(deltaAngle_normal_2)

			coefficient_angle = 1.4 # 2

			angulerVel_l1 = abs(deltaAngle_normal_2)*coefficient_angle
			angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)

			if (abs(deltaAngle_normal_2) < radians(0.1)):
				angulerVel_l1 = 0.0
			else:
				if (angulerVel_l1 < self.rotation_min):
					angulerVel_l1 = self.rotation_min

			# -- xac dinh chieu
			if (deltaAngle_normal_2 >= 0):
				angulerVel_l2 = angulerVel_l1
			else:
				angulerVel_l2 = angulerVel_l1*(-1)

			# -- loai bo van de bi lech dau khi den dich
			if (delta_distance < 0.01):
				self.cmd_vel.angular.z = 0
			else:
				self.cmd_vel.angular.z = angulerVel_l2

			if (delta_distance < 0.001):
				self.cmd_vel.linear.x = 0.0
				self.cmd_vel.angular.z = 0.0
				self.step_run_2_points = 5
				# print ("OK!")
				
		elif (self.step_run_2_points == 4): # -- chon X
			# -- 
			self.cmd_vel.angular.z = 0.0
			delta_distance = abs(self.robot_pose.pose.position.x -  point_2.position.x)
			# print ("delta_distance X: ", delta_distance)
			if (delta_distance < 1.0 and delta_distance >= 0.598): # - tang toc
				coefficient_linear = (0.698 - delta_distance)/self.acceleration_distance
				if (coefficient_linear >= 1.0):
					coefficient_linear = 1.0
				linearVel_l1 = coefficient_linear*self.linear_max
			else: # - giam toc
				coefficient_linear = delta_distance/self.deceleration_distance
				if (coefficient_linear >= 1.0):
					coefficient_linear = 1.0
				linearVel_l1 = coefficient_linear*self.linear_max

			linearVel_l2 = self.constrain(linearVel_l1, self.linear_min, self.linear_max)
			self.cmd_vel.linear.x = linearVel_l2

			if (delta_distance < 0.001):
				self.cmd_vel.linear.x = 0.0
				self.step_run_2_points = 5
				print ("OK!")

		if (self.step_run_2_points == 5):
			print ("2 - Tag info: X= " + str(round(self.tagInfo_now.pose.position.x, 3)) + " |Y= " +  str(round(self.tagInfo_now.pose.position.y, 3)) + " |R= " +  str(round(self.tagInfo_now.angle, 2)) )
			print ("--------")
			self.preTime_setPose = time.time()
			self.step_run_2_points = 6
			self.setpose_requir.data = 0

		elif (self.step_run_2_points == 6):
			# -- setpose
			self.cmd_vel = Twist()
			self.setpose_requir.data = 1
			# print ("seting poseTag!")
			delta_time = (time.time() - self.preTime_setPose)%60
			if (delta_time >= 3): # or thieu topic respond
				self.step_run_2_points = 7
				self.setpose_requir.data = 0

		elif (self.step_run_2_points == 7): # -- 
			sts_out = 1
			self.step_run_2_points = -1
			self.step_fineTuning = 0
			self.isRead_robotPose = 0

		self.pub_cmdVel.publish(self.cmd_vel)
		self.pub_setPose.publish(self.setpose_requir)
		return sts_out

	def process(self):
		if (self.step_run == 0):
			count = 0
			self.notification = 'Wait topic: '
			if (self.isRead_query):
				count += 1
			else:
				self.notification += "move_query | "

			if (self.isRead_zone):
				count += 1
			else:
				self.notification += "zone_safety | "

			if (self.isRead_robotPose):
				count += 1
			else:
				self.notification += "robot_pose"

			if (count == 3):
				self.step_run = 1
				self.notification = 'launch ok'

		elif (self.step_run == 1):
			
			# -- kiem tra du lieu yeu cau co dung dinh dang ko.
			if (self.check_listRight(self.move_query_readed) == 0):
				self.status = 3
				self.notification = 'Run: ERROR QUERY -> Stoping!'
				self.cmd_vel = Twist()

			else:
				if (self.requir_setpose == 0):
					# -- kiểm tra sự thay đổi của mục tiêu và đang ở giữa Tag -> cho đổi mục tiêu.
					# target = self.move_query.tag_taget
					target = self.move_query_readed.tag_taget

					if (self.pre_idTarget != target):
						self.is_queryChangeTarget = 1

					# -- 
					if (self.is_queryChangeTarget == 1):
						self.move_query = self.move_query_readed

						self.is_queryChangeTarget = 0
						self.is_completed_all = 0

						self.is_completed_listPoint = 0
						self.idTarget = target
						self.pre_idTarget = self.idTarget

						self.locateGoal = 0
						self.locateGoal_next = self.locateGoal + 1		

						self.tag_goal = self.move_query.tags[self.locateGoal]
						self.tag_goal_next = self.move_query.tags[self.locateGoal_next]
						# --
						self.requir_type = 1
						# -- 
						self.pre_moveQuery.tags = self.move_query.tags

					# if (self.pre_moveQuery.tags != self.move_query.tags):
					# 	# self.is_completed_listPoint = 0
					# 	print ("uuuuuu")
					# 	lc = -1
					# 	for i in range(5):
					# 		if (self.tag_goal.id == self.move_query.tags[i].id):
					# 			lc = i

					# 	if (lc < 0):
					# 		lc = 0

					# 		self.requir_type = 1
							
					# 	self.locateGoal = lc
						
					# 	if (self.locateGoal < 4):
					# 		self.locateGoal_next = self.locateGoal + 1
					# 	else:
					# 		self.locateGoal_next = 4

					# 	self.tag_goal = self.move_query.tags[self.locateGoal]
					# 	self.tag_goal_next = self.move_query.tags[self.locateGoal_next]

					# 	self.pre_moveQuery.tags = self.move_query.tags

					# 	print ("self.tag_goal.id: ", self.tag_goal.id)
						
						# self.is_queryChangeTarget = 1

					# --  di chuyển.
					if (self.is_completed_listPoint == 1):
						self.notification = 'Stop: moved past 5 points'
						self.status = 3
						self.cmd_vel = Twist()

					elif (self.is_completed_all == 1):
						self.notification = 'Stop: Completed'
						self.status = 6
						self.cmd_vel = Twist()
					else:
					
						# -- -- khai bao
						linearVel_l1 = 0.0
						linearVel_l2 = 0.0
						# --
						angulerVel_l1 = 0.0
						angulerVel_l2 = 0.0
						# --
						angle_goGoal = 0.0
						angle_robot = 0.0
						angle_final = 0.0
						angle_goGoal_secondary = 0.0
						angle_goalNext = 0.0
						# --
						deltaAngle_final = 0.0
						deltaAngle_normal_1 = 0.0
						deltaAngle_normal_2 = 0.0
						# --
						coefficient_angle = 0.0
						coefficient_linear = 0.0
						pose_target = self.tag_goal.pose
						# -- -- khoang cach.
						self.delta_distance = self.calculate_distance(self.robot_pose.pose.position, pose_target.position)

						# -- -- goc.
						angle_goGoal = self.calculateAngle_twoPoint(self.robot_pose.pose.position, pose_target.position)
						# --
						quat = ( self.robot_pose.pose.orientation.x, self.robot_pose.pose.orientation.y, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w )
						a, b, angle_robot = euler_from_quaternion(quat)
						# --
						quat_target = (pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w)
						a, b, angle_final = euler_from_quaternion(quat_target)
						# --

						# angle_goGoal_secondary = self.findAngle_targetSecondary(self.robot_pose.pose, pose_target, angle_goGoal, self.delta_distance, self.delta_distance/2.0)
						angle_goGoal_secondary = self.findAngle_targetSecondary_vs1(self.robot_pose.pose, pose_target, angle_goGoal, self.delta_distance, 0.0)
						# --
						angle_goalNext = self.calculateAngle_twoPoint(self.tag_goal.pose.position, self.tag_goal_next.pose.position)
						# -- -- do lech goc
						deltaAngle_normal_1 = angle_goGoal - angle_robot
						deltaAngle_normal_1 = self.limmit_corner(deltaAngle_normal_1)
						# -- 
						deltaAngle_normal_2 = angle_goGoal_secondary - angle_robot
						deltaAngle_normal_2 = self.limmit_corner(deltaAngle_normal_2)
						# -- 
						deltaAngle_final = angle_final - angle_robot
						deltaAngle_final = self.limmit_corner(deltaAngle_final)
						# -- 
						deltaAngle_goalNext = angle_goalNext - angle_goGoal

						# ----- ---
						if (self.requir_type == 1):
							ih = 0 
							if (self.locateGoal_next != 0 and self.locateGoal_next != -1):
								ih = 1
							else:
								ih = 0

							self.type_run = self.idenify_caseMove(angle_robot, angle_goGoal, angle_goalNext, self.tag_goal.id, self.move_query.tag_taget, ih)
							self.requir_type = 0

							print ("Calculated type_run: ", self.type_run)
										
						# ----- ----
						if (self.type_run == 1):
							if (abs(self.delta_distance) < self.valueCompleted_level2.distance and self.step_goGo == 1):
								print ("1 - ok position!")
								self.step_goGo = 2

							if (abs(deltaAngle_final) < self.valueCompleted_level2.angle and self.step_goGo == 2):
								print ("2- ok angle!")
								self.step_goGo = 3

						if (self.type_run == 5): # -- Điểm tiếp theo tạo thành góc 0 (5)
							if (abs(self.delta_distance) < self.valueCompleted_level1.distance):
								# print ("5 - ok position + ok angle!")
								if (self.step_goGo == 1):
									self.step_goGo = 3

						elif (self.type_run == 3 or self.type_run == 4): 
							# -- Điểm tiếp theo tạo thành góc vuông hoặc 180 độ (4)
							# -- Không tồn tại điểm tiếp theo (3)
							if (abs(self.delta_distance) < (self.valueCompleted_level4.distance) and self.step_goGo == 1 ):
								print ("3,4 - ok position")
								self.step_goGo = 3

						if (self.step_goGo == 0): # xoay dung huong
							linearVel_l1 = 0.0
							linearVel_l2 = 0.0
							
							coefficient_angle = abs(deltaAngle_normal_2)/self.deceleration_angle
							if (coefficient_angle > 1):
								coefficient_angle = 1.0

							# -- xac dinh do lon
							angulerVel_l1 = self.rotation_max*coefficient_angle
							angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)

							# -- xac dinh chieu
							if (deltaAngle_normal_2 >= 0):
								angulerVel_l2 = angulerVel_l1
							else:
								angulerVel_l2 = angulerVel_l1*(-1)

							if (abs(deltaAngle_normal_2) < radians(0.3)):	
								angulerVel_l2 = 0.0
								self.step_goGo = 1

						elif (self.step_goGo == 1): # di thang
							# -- -- Linear
							coefficient_linear = abs(self.delta_distance)/self.deceleration_distance
							if (coefficient_linear >= 1.0):
								coefficient_linear = 1.0
							linearVel_l1 = coefficient_linear*self.linear_max

							linearVel_l2 = self.constrain(linearVel_l1, self.linear_min, self.linear_max)
							# -- -- Angular
							# -- xac dinh do lon
							coefficient_angle = 2.1 # 2

							angulerVel_l1 = abs(deltaAngle_normal_2)*coefficient_angle
							angulerVel_l1 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)

							if (abs(deltaAngle_normal_2) < radians(0.4)):
								angulerVel_l1 = 0.0
							else:
								if (angulerVel_l1 < self.rotation_min):
									angulerVel_l1 = self.rotation_min

							# -- xac dinh chieu
							if (deltaAngle_normal_2 >= 0):
								angulerVel_l2 = angulerVel_l1
							else:
								angulerVel_l2 = angulerVel_l1*(-1)

							if (abs(deltaAngle_normal_2) >= radians(15)):
								self.step_goGo = 0
								self.requir_type = 1 # yeu cau tinh lai phuong phap di chuyen. Do lech nhieu.

						if (self.step_goGo == 2): #
							# print ("delta_distance: " + str(self.delta_distance) + " |linearVel_l2: " + str(linearVel_l2))
							if (abs(deltaAngle_final) >= pi):
								if (deltaAngle_final >= 0):
									deltaAngle_final = (pi*2 - abs(deltaAngle_final))*(-1)
								else:
									deltaAngle_final = pi*2 - abs(deltaAngle_final)

							if (abs(deltaAngle_final) < self.deceleration_angle):
								coefficient_angle = abs(deltaAngle_final)/self.deceleration_angle
							else:
								coefficient_angle = 1.
							
							angulerVel_l1 = self.rotation_max*coefficient_angle
							# -- Điều chỉnh tốc độ theo ngưỡng.
							angulerVel_l2 = self.constrain(angulerVel_l1, self.rotation_min, self.rotation_max)

							# -- Xác định chiều quay.
							if (deltaAngle_final >= 0):
								angulerVel_l2 = angulerVel_l2
							else:
								angulerVel_l2 = angulerVel_l2*(-1)

						if (self.step_goGo == 3): # xoay dung huong
							print ("-----------------------------------------------")
							print ("self.locateGoal: ", self.locateGoal)
							print ("tag_goal.id: ", self.tag_goal.id)
							print ("tag_goal x: ", self.tag_goal.pose.position.x)
							print ("tag_goal y: ", self.tag_goal.pose.position.y)

							if (self.tag_goal.id == self.move_query.tag_taget):
								self.is_completed_all = 1
							else:
								self.locateGoal += 1
							
							if (self.locateGoal > 4):
								self.locateGoal = 4
								self.is_completed_listPoint = 1

							if (self.locateGoal >= 4):
								self.locateGoal_next = -1
							else:
								self.locateGoal_next = self.locateGoal + 1

							if (self.move_query.tags[self.locateGoal].id == 0):
								self.is_completed_listPoint = 1
								self.locateGoal += -1

							if (self.move_query.tags[self.locateGoal_next].id == 0):
								self.locateGoal_next = -1

							# ----- cập nhật dữ liệu điểm di chuyển hiện tại. 
							self.tag_goal = self.move_query.tags[self.locateGoal]
							if (self.locateGoal_next == -1):
								self.tag_goal_next = Tag_node()
							else:
								self.tag_goal_next = self.move_query.tags[self.locateGoal_next]

							self.step_goGo = 0
							self.requir_type = 1

							""" -- loai bo chinh lai huong khi co diem moi
							1, Co diem tiep theo
							2, Kieu di chuyen truoc do = 5
							3, Co 3 huong trung nhau:
								- huong robot
								- huong tu goal hien tai den goal tiep theo
								- huong hien robot den goal hien tai
							"""

							print ("completed type_run: ", self.type_run)
							if (self.locateGoal_next != -1 and self.locateGoal_next != 0) or (self.tag_goal.id == self.move_query.tag_taget):
								print ("Loai bo: 1")
								if (self.type_run == 5):
									print ("TH1 loai bo: 2")
									if (abs(deltaAngle_normal_1) <= radians(15) and abs(deltaAngle_goalNext) <= radians(15) ):
										print ("TH1 loai bo: 3")
										self.step_goGo = 1
										print ("TH1 loai bo xoay hihi")

							if (self.type_run == 1 or self.type_run == 4 or self.type_run == 3):
								self.requir_setpose = 1
								print ("requir setpose!")
								self.preTime_setPose = time.time()

							# --- 
							if (self.pre_moveQuery.tags != self.move_query_readed.tags):
								# self.is_completed_listPoint = 0
								self.move_query = self.move_query_readed
								self.pre_moveQuery.tags = self.move_query.tags
								print ("uuuuuu")
								lc = -1
								for i in range(5):
									if (self.tag_goal.id == self.move_query.tags[i].id):
										lc = i

								if (lc < 0):
									lc = 0

									self.requir_type = 1
									
								self.locateGoal = lc
								
								if (self.locateGoal < 4):
									self.locateGoal_next = self.locateGoal + 1
								else:
									self.locateGoal_next = 4

								self.tag_goal = self.move_query.tags[self.locateGoal]
								self.tag_goal_next = self.move_query.tags[self.locateGoal_next]

								print ("self.tag_goal.id: ", self.tag_goal.id)

								# -- add new
								
							# -- 

						self.notification = "id: " + str(self.tag_goal.id) + " | " + "typeRun: " + str(self.type_run)
						self.status = self.step_goGo
						self.cmd_vel.linear.x = linearVel_l2
						self.cmd_vel.angular.z = angulerVel_l2
						# -- 
						self.setpose_requir.data = 0

				else:
					self.cmd_vel = Twist()
					self.setpose_requir.data = 1
					# print ("seting poseTag!")
					delta_time = (time.time() - self.preTime_setPose)%60
					if (delta_time >= 0.6): # or thieu topic respond
						self.requir_setpose = 0

			self.show_plan()
		self.move_respond.status = self.status
		self.move_respond.message = self.notification

		self.pub_cmdVel.publish(self.cmd_vel)
		self.pub_setPose.publish(self.setpose_requir)
		self.pub_moveRespond.publish(self.move_respond)
		
	def run(self):
		print ("Launch ALL!")

		# self.tester = 1
		self.poseA = self.pose1
		self.poseB = self.pose2

		while not rospy.is_shutdown():
			# self.process()
			if (self.isRead_robotPose == 1):
				# self.backward_to_charger(0.20)

				# sts = self.move_betweent_points(self.pose1, self.pose2)
				# if (sts == 1):
				# 	self.tester = 1



				sts = self.move_betweent_points(self.poseA, self.poseB)
				if (sts == 1):

					if (self.tester == 0):
						self.tester = 1
					else:
						self.tester = 0

					if (self.tester == 0):
						self.poseA = self.pose1
						self.poseB = self.pose2
						# print ("now 0")
					else:
						self.poseA = self.pose3
						self.poseB = self.pose4
						# print ("now 1")
				# self.fineTuning_position_vs2(self.poseExample, self.valueCompleted_level2)
				# self.pub_cmdVel.publish(self.cmd_vel)

			self.rate.sleep()
		self.pub_cmdVel.publish(Twist())
		time.sleep(0.01)
		self.pub_cmdVel.publish(Twist())
		time.sleep(0.01)
		self.pub_cmdVel.publish(Twist())
		

def main():
	print('Starting main program')
	program = navigation()
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
