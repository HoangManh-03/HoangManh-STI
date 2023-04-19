#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date  : 15/4/2022
Update: 24/10/2022
"""
import roslib

import sys
import time
from decimal import *
import math
import rospy

# -- add 19/01/2022
import subprocess
import re
import os

from sti_msgs.msg import *

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int8, Bool
from message_pkg.msg import *
from ros_canBus.msg import *
from sensor_msgs.msg import Imu
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

import json

class MisssionCommand:
	def __init__(self):
		self.unknown = 0
		self.charger = 10
		self.getItem_1 = 1
		self.getItem_2 = 2
		self.getItem_3 = 3
		self.getItem_4 = 4
		self.getItem_5 = 5
		self.getItem_6 = 6
		self.returnItem_1 = -1
		self.returnItem_2 = -2
		self.returnItem_3 = -3
		self.returnItem_4 = -4
		self.returnItem_5 = -5
		self.returnItem_6 = -6

#--------------------------------------------------------------------------------- ROS
class ros_control():
	def __init__(self):
		rospy.init_node('stiControl', anonymous=False)
		self.rate = rospy.Rate(30)
		# -------------- Parameter -------------- #
		self.name_card = "wlo2"
		self.address = "172.21.16.224" # "172.21.15.224"

		# -- App
		rospy.Subscriber("/app_button", App_button, self.callback_appButton) # lay thong tin trang thai nut nhan tren man hinh HMI.
		self.app_button = App_button()

		# -- Cancel mission
		rospy.Subscriber("/cancelMission_control", Int16, self.callback_cancelMission)
		self.cancelMission_control = Int16()		
		self.flag_cancelMission = 0
		self.status_cancel = 0
		# -
		self.pub_cancelMission = rospy.Publisher("/cancelMission_status", Int16, queue_size=100)	
		self.cancelMission_status = Int16()

		# -------------- Cac ket noi ngoai vi -------------- #
		# -- Reconnect
		rospy.Subscriber("/status_reconnect", Status_reconnect, self.callback_reconnect)
		self.status_reconnect = Status_reconnect()

		# -- Board RTC
		rospy.Subscriber("/CAN_received", CAN_received, self.callback_RTC) 
		self.timeStampe_RTC = rospy.Time.now()

		# -- Board MAIN - POWER
		rospy.Subscriber("/POWER_info", POWER_info, self.callback_Main) 
		self.main_info = POWER_info()
		self.timeStampe_main = rospy.Time.now()
		self.voltage = 24.5
		# -
		self.pub_requestMain = rospy.Publisher("/POWER_request", POWER_request, queue_size=100)	
		self.power_request = POWER_request()

		# -- Board HC 82
		rospy.Subscriber("/HC_info", HC_info, self.callback_HC) # lay thong tin trang thai cua node va cam bien sick an toan.
		self.HC_info = HC_info()
		self.timeStampe_HC = rospy.Time.now()
		# -
		self.pub_controlHC = rospy.Publisher("/HC_request", HC_request, queue_size=100)	# dieu khien den bao va den ho tro camera
		self.HC_request = HC_request()

		# -- Board OC 12 | conveyor1
		rospy.Subscriber("/status_conveyor1", Status_conveyor, self.callback_conveyor1) # 
		self.status_conveyor1 = Status_conveyor()
		self.timeStampe_conveyor12 = rospy.Time.now()

		# -- Board OC 12 | conveyor2
		rospy.Subscriber("/status_conveyor2", Status_conveyor, self.callback_conveyor2) # 
		self.status_conveyor2 = Status_conveyor()

		# -- Board OC 34 | conveyor3
		rospy.Subscriber("/status_conveyor3", Status_conveyor, self.callback_conveyor3) # 
		self.status_conveyor3 = Status_conveyor()
		self.timeStampe_conveyor34 = rospy.Time.now()

		# -- Board OC 34 | conveyor4
		rospy.Subscriber("/status_conveyor4", Status_conveyor, self.callback_conveyor4) # 
		self.status_conveyor4 = Status_conveyor()

		# -- Board OC 56 | conveyor5
		rospy.Subscriber("/status_conveyor5", Status_conveyor, self.callback_conveyor5) # 
		self.status_conveyor5 = Status_conveyor()
		self.timeStampe_conveyor56 = rospy.Time.now()

		# -- Board OC 56 | conveyor6
		rospy.Subscriber("/status_conveyor6", Status_conveyor, self.callback_conveyor6) # 
		self.status_conveyor6 = Status_conveyor()

		# - Board OC | Control Conveyors
		self.pub_controlConveyors = rospy.Publisher("/control_conveyors", Control_conveyors, queue_size= 10)	# Dieu khien ban nang.
		self.control_conveyors = Control_conveyors()

		# -- Board CPD | Toyo
		rospy.Subscriber("/CPD_read", CPD_read, self.callback_CPD) # 
		self.status_CPD = CPD_read()
		self.timeStampe_CPD = rospy.Time.now()
		# -
		self.pub_controlCPD = rospy.Publisher("/CPD_write", CPD_write, queue_size= 10)	# Dieu khien Toyo.
		self.control_CPD = CPD_write()
		# -------------------

		# -- data nav
		rospy.Subscriber("/nav350_data", Nav350_data, self.callback_nav350) 
		self.nav350_data = Nav350_data()

		# -- Data safety NAV
		rospy.Subscriber("/safety_NAV", Int8, self.callback_safety_NAV) 
		self.safety_NAV = Int8()

		# -- robotPose_nav - POSE
		rospy.Subscriber('/robotPose_nav', PoseStamped, self.callback_getPoseRobot, queue_size = 100)
		self.robotPose_nav = PoseStamped()

		# -- Port physical
		rospy.Subscriber("/status_port", Status_port, self.callback_port)
		self.status_port = Status_port()

		# -------------- Cac node thuat toan dieu khien --------------
		# -- Communicate with Server
		rospy.Subscriber("/NN_infoRequest", NN_infoRequest, self.callback_Traffic_infoRequest)
		self.Traffic_infoRequest = NN_infoRequest()
		self.timeStampe_TrafficReceived = rospy.Time.now()
		# -
		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.callback_Traffic_cmdRequest)
		self.Traffic_cmdRequest = NN_cmdRequest()
		# -- add 18/01/2022 : Sua loi di lai cac diem cu khi mat ket noi server.
		self.list_id_unknown = [0, 0, 0, 0, 0]
		self.flag_listPoint_ok = 0
		# -
		self.pub_infoRespond = rospy.Publisher("/NN_infoRespond", NN_infoRespond, queue_size=100)
		self.Traffic_infoRespond = NN_infoRespond()

		rospy.Subscriber("/status_goal_control", Status_goal_control, self.callback_goalControl)
		self.status_goalControl = Status_goal_control() # sub from move_base
		self.timeStampe_statusGoalControl = rospy.Time.now()
		# -
		self.pub_moveReq = rospy.Publisher("/request_move", Move_request, queue_size=100)
		self.move_req = Move_request()
		self.enable_moving = 0                    # cho phep navi di chuyen

		# -- Parking
		rospy.Subscriber("/parking_respond", Parking_respond, self.callback_parking)
		self.parking_status = Parking_respond()
		self.parking_offset = 0.0
		self.parking_poseBefore = Pose()
		self.parking_poseTarget = Pose()
		# -
		self.pub_parking = rospy.Publisher("/parking_request", Parking_request, queue_size= 20)
		self.enable_parking = 0
		self.parking_poseBefore = Pose()
		self.parking_poseAfter = Pose()
		self.parking_offset = 0.0
		# - Backward
		self.flag_requirBackward = 0
		self.completed_backward = 0
		self.backward_x = 0.0
		self.backward_y = 0.0
		self.backward_z = 0.0

		# -- Cmd_vel
		self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.time_ht = rospy.get_time()
		self.time_tr = rospy.get_time()
		self.rate_cmdvel = 10. 

		# -- Driver Motor
		rospy.Subscriber("/driver1_respond", Driver_respond, self.callback_driver1)
		self.driver1_respond = Driver_respond()
		# - 
		rospy.Subscriber("/driver2_respond", Driver_respond, self.callback_driver2)
		self.driver2_respond = Driver_respond()
		self.timeStampe_driver = rospy.Time.now()
		# -
		self.pub_taskDriver = rospy.Publisher("/task_driver", Int16, queue_size= 20)
		self.task_driver = Int16()
		# -
		self.pub_disableBrake = rospy.Publisher("/disable_brake", Bool, queue_size= 20)
		self.disable_brake = Bool()
		# - Task driver
		self.taskDriver_nothing = 0
		self.taskDriver_resetRead = 1
		self.taskDriver_Read = 2
		self.task_driver.data = self.taskDriver_Read

		# --------------------- Parameter p --------------------- #
		# -- Hz
		self.FrequencePubBoard = 10.
		self.pre_timeBoard = rospy.get_time()
		# -- Mode operate
		self.md_by_hand = 1
		self.md_auto = 2
		self.mode_operate = self.md_by_hand    # Lưu chế độ hoạt động.
		# -- Target
		self.target_x = 0.		 # lưu tọa độ điểm đích hiện tại.			
		self.target_y = 0.
		self.target_z = 0.
		self.target_tag = 0.
		# - 
		self.process = 0         # - Tiến trình đang xử lý
		self.mission_before = 0  # - Nhiệm vụ trước khi di chuyển.
		self.mission_after  = 0  # - Nhiệm vụ sau khi di chuyển.
		self.enable_mission = 0
		# -
		self.completed_before_mission = 0	 # Báo nhiệm vụ trước đã hoàn thành.
		self.completed_after_mission = 0	 # Báo nhiệm vụ sau đã hoàn thành.
		self.completed_move = 0			 	 # Báo di chuyển đã hoàn thành.
		self.completed_moveSimple = 0      # bao da den dich.
		self.completed_moveSpecial = 0     # bao da den aruco.
		self.completed_reset = 0             # hoan thanh reset.
		self.completed_checkConveyors = 0 	# kiem tra ke co hay ko sau khi nang. 
		# -- Flag
		self.flag_afterChager = 0
		self.flag_Auto_to_Byhand = 0
		self.flag_read_client = 0
		self.flag_error = 0
		self.flag_warning = 0
		# --
		self.pre_mess = ""               # lưu tin nhắn hiện tại.
		# -- Status to server:
		self.statusAGV = 0
		self.statusAGV_allRight = 0
		self.statusAGV_warning = 1
		self.statusAGV_error = 2	
		self.statusAGV_cancelMission = 5	
		# -- Status to detail to follow:
		self.stf = 0
		self.stf_wakeup = 0
		self.stf_running_simple = 1
		self.stf_stop_obstacle = 2
		self.stf_running_speial = 3
		self.stf_running_backward = 4
		self.stf_performUp = 5
		self.stf_performDown = 6
		# -- EMC reset
		self.EMC_resetOn = 1
		self.EMC_resetOff = 0
		self.EMC_reset = self.EMC_resetOff
		# -- EMC write
		self.EMC_writeOn = 1
		self.EMC_writeOff = 0
		self.EMC_write = self.EMC_writeOff
		# -- Led
		self.led_effect = 0
		self.led_error = 1 			# 1
		self.led_simpleRun = 2 		# 2
		self.led_specialRun = 3 	# 3
		self.led_perform = 4 		# 4
		self.led_completed = 5  	# 5	
		self.led_stopBarrier = 6  	# 6
		# -- Mission server
		self.statusTask_liftError = 64 # trang thái nâng kệ nhueng ko có kệ.
		self.serverMission_liftUp = 65
		self.serverMission_liftDown = 66
		self.serverMission_charger = 5
		self.serverMission_unknown = 0
		self.serverMission_liftDown_charger = 6
		# -- Lift task.
		self.conveyorTask_received = 1
		self.conveyorTask_transmit = 2
		self.conveyorTask_stop = 0
		self.conveyor1_taskByHand = self.conveyorTask_stop
		self.conveyor2_taskByHand = self.conveyorTask_stop
		self.conveyor3_taskByHand = self.conveyorTask_stop
		self.conveyor4_taskByHand = self.conveyorTask_stop
		self.conveyor5_taskByHand = self.conveyorTask_stop
		self.conveyor6_taskByHand = self.conveyorTask_stop
		# -- Speaker
		self.speaker_effect = 0
		self.speaker_requir = 0 # luu trang thai cua loa
		self.spk_error = 3
		self.spk_move = 1
		self.spk_warn = 2
		self.spk_not = 4		
		self.spk_off = 0
		self.enable_speaker = 1
		# -- Charger
		self.charger_on = 1
		self.charger_off = 0		
		self.charger_requir = self.charger_off
		self.charger_write = self.charger_requir
		self.charger_valueOrigin = 0.1
		# -- Voltage
		self.timeCheckVoltage_charger = 1800 # s => 30 minutes.
		self.timeCheckVoltage_normal = 60     # s
		self.pre_timeVoltage = 0   # s
		self.valueVoltage = 0
		self.step_readVoltage = 0
		# -- Error Type
		self.error_move = 0
		self.error_perform = 0
		self.error_device = 0  # camera(1) - MC(2) - Main(3) - SC(4)

		self.numberError = 0
		self.lastTime_checkLift = 0.0
		# -- add new
		self.enb_debug = 0
		# --
		self.listError = []
		self.job_doing = 0
		# -- -- -- Su dung cho truong hop khi AGV chuyen Che do bang tay, bi keo ra khoi vi tri => AGV se chay lai.
		# -- Pose tai vi tri Ke, sac
		self.poseWait = Pose()
		self.distance_resetMission = 0.1
		self.flag_resetFramework = 0
		# --
		self.flag_stopMove_byHand = 0
		# --
		self.timeStampe_reflectors = rospy.Time.now()
		self.pose_parkingRuning = Pose()
		# --
		self.cancelbackward_pose = Pose()
		self.cancelbackward_offset = 0.0

		# -- add 19/01/2022 : Check error lost server.
		self.saveTime_checkServer = rospy.Time.now()
		self.saveStatus_server = 0
		# -- add 22/01/2022
		self.flag_notCharger = 0
		# -- add 15/04/2022
		self.flag_listPointEmpty = 0
		# --
		self.misssionCommand = MisssionCommand()
		print (" --- Launch ---")
	# ------------------------ Call Back ------------------------ #
	def callback_appButton(self, data):
		self.app_button = data

	def callback_cancelMission(self, data):
		self.cancelMission_control = data

	def callback_reconnect(self, data):
		self.status_reconnect = data

	def callback_RTC(self, data):
		self.timeStampe_RTC = rospy.Time.now()

	def callback_Main(self, data):
		self.main_info = data
		self.voltage = round(self.main_info.voltages, 2)
		self.timeStampe_main = rospy.Time.now()

	def callback_HC(self, data):
		self.HC_info = data
		self.timeStampe_HC = rospy.Time.now()

	def callback_conveyor1(self, data):
		self.status_conveyor1 = data
		self.timeStampe_conveyor12 = rospy.Time.now()

	def callback_conveyor2(self, data):
		self.status_conveyor2 = data
		self.timeStampe_conveyor12 = rospy.Time.now()

	def callback_conveyor3(self, data):
		self.status_conveyor3 = data
		self.timeStampe_conveyor34 = rospy.Time.now()

	def callback_conveyor4(self, data):
		self.status_conveyor4 = data
		self.timeStampe_conveyor34 = rospy.Time.now()

	def callback_conveyor5(self, data):
		self.status_conveyor5 = data
		self.timeStampe_conveyor56 = rospy.Time.now()

	def callback_conveyor6(self, data):
		self.status_conveyor6 = data
		self.timeStampe_conveyor56 = rospy.Time.now()

	def callback_CPD(self, data):
		self.status_CPD = data
		self.timeStampe_CPD = rospy.Time.now()

	def callback_nav350(self, data):
		self.nav350_data = data

	def callback_safety_NAV(self, data):
		self.safety_NAV = data

	def callback_getPoseRobot(self, data):
		self.robotPose_nav = data
		# doi quaternion -> rad    
		quaternion1 = (data.pose.orientation.x, data.pose.orientation.y,\
					data.pose.orientation.z, data.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion1)

		self.Traffic_infoRespond.x = round(data.pose.position.x, 3)
		self.Traffic_infoRespond.y = round(data.pose.position.y, 3)
		self.Traffic_infoRespond.z = round(euler[2], 3)

	def callback_port(self, data):
		self.status_port = data

	def callback_Traffic_infoRequest(self, data):
		self.Traffic_infoRequest = data
		self.timeStampe_TrafficReceived = rospy.Time.now()

	def callback_Traffic_cmdRequest(self, dat):
		self.Traffic_cmdRequest = dat
		self.timeStampe_TrafficReceived = rospy.Time.now()
		self.flag_listPoint_ok = 0

	def callback_goalControl(self, dataa):
		self.status_goalControl = dataa
		self.timeStampe_statusGoalControl = rospy.Time.now()

	def callback_parking(self, data):
		self.parking_status = data

	def callback_driver1(self, data):
		self.driver1_respond = data
		self.timeStampe_driver = rospy.Time.now()

	def callback_driver2(self, data):
		self.driver2_respond = data
		self.timeStampe_driver = rospy.Time.now()

	# ------------------------   ------------------------ #
	def pub_park(self, modeRun, poseBefore, poseTarget, offset):
		park = Parking_request()
		park.modeRun = modeRun
		park.poseBefore = poseBefore
		park.poseTarget = poseTarget
		park.offset = offset

		self.pub_parking.publish(park)

	def pub_move_req(self, ena, ls):
		req_move = Move_request()

		req_move.enable = ena
		req_move.target_x = ls.target_x
		req_move.target_y = ls.target_y
		req_move.target_z = ls.target_z
		req_move.tag = ls.tag
		req_move.offset = ls.offset
		req_move.list_id = ls.list_id
		req_move.list_x = ls.list_x
		req_move.list_y = ls.list_y
		req_move.list_speed = ls.list_speed
		req_move.mission = ls.mission

		self.pub_moveReq.publish(req_move)

	def pub_cmdVel(self, twist , rate , time):
		self.time_ht = time 
		# print self.time_ht - self.time_tr
		# print 1/float(rate)
		if self.time_ht - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = self.time_ht
			self.pub_vel.publish(twist)
		# else :
			# rospy.logwarn("Hz /cmd_vel OVER !! - %f", 1/float(self.time_ht - self.time_tr) )  

	def pub_Main(self, charge, sound, EMC_write, EMC_reset):
		# charge, sound_on, sound_type, EMC_write, EMC_reset, OFF_5v , OFF_22v, led_button1, led_button2, a_coefficient , b_coefficient 
		mai = POWER_request()
		mai.charge = charge
		if sound == 0:
			mai.sound_on = 0	
		else:
			mai.sound_on = 1
			mai.sound_type = sound
		mai.EMC_write = EMC_write
		mai.EMC_reset = EMC_reset
		
		self.pub_requestMain.publish(mai)

	# -- add 15/04/2022
	def check_listPoints(self, list_id):
		count = 0
		try: 
			for i in range(5):
				if (list_id[i] != 0):
					count += 1

			if count != 0:
				return 1
			else: 
				return 0
		except:
			return 0

	def log_mess(self, typ, mess, val):
		# -- add new
		if (self.enb_debug):
			if self.pre_mess != mess:
				if typ == "info":
					rospy.loginfo (mess + ": %s", val)
				elif typ == "warn":
					rospy.logwarn (mess + ": %s", val)
				else:
					rospy.logerr (mess + ": %s", val)
			self.pre_mess = mess

# 	def do_cancelMisssion(self):
# 		if self.flag_cancelMission == 0:
# 			self.status_cancel = 0
# 		else:
# 			self.status_cancel = 1

	# -- add 19/01/2020
	def get_ipAuto(self, name_card): # name_card : str()
		try:
			address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
			# print ("address: ", address)
			return 1
		except Exception:
			return 0

	def pingServer(self, address):
			# - ping server
			try:
				ping = subprocess.check_output("ping -{} 1 {}".format('c',address), shell=True)
				# print(ping)
				vitri = str(ping).find("time")
				time_ping = str(ping)[(vitri+5):(vitri+9)]
				return float(time_ping)
			except Exception:
				# print("no ping")			
				return -1

	def check_server(self):
		is_ip = self.get_ipAuto(self.name_card)
		# is_ip = 1
		# time_ping = self.pingServer(self.address)
		time_ping = 0
		if (is_ip == 1):
			if (time_ping == -1):
				return 1 # khong Ping dc server
			else:
				return 0 # oki
		else:
			return 2 # khong lay dc IP

	def point_same_point(self, x1, y1, z1, x2, y2, z2):
		# tọa độ
		x = x2 - x1
		y = y2 - y1
		d = math.sqrt(x*x + y*y)
		# góc 
		if z2*z1 >= 0:
			z = z2 - z1
		else:
			z = z2 + z1
		if d > 0.2 or abs(z) > 0.14:  # 20 cm - ~ 20*C
			return 1
		else:
			return 0

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def readbatteryVoltage(self): # 
		time_curr = rospy.get_time()
		delta_time = (time_curr - self.pre_timeVoltage)
		if self.charger_requir == self.charger_on:
			self.flag_afterChager = 1
			if self.step_readVoltage == 0:  # bat sac.
				self.charger_write = self.charger_on
				if (delta_time > self.timeCheckVoltage_charger):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 1

			elif self.step_readVoltage == 1: # tat sac va doi.
				self.charger_write = self.charger_off
				if (delta_time > self.timeCheckVoltage_normal*3):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 2

			elif self.step_readVoltage == 2: # do pin.	
				bat = round(self.main_info.voltages, 1)*10
				# print "charger --"
				if  bat > 255:
					self.valueVoltage = 255
				elif bat < 0:
					self.valueVoltage = 0
				else:
					self.valueVoltage = bat

				self.pre_timeVoltage = time_curr
				self.step_readVoltage = 3

			elif self.step_readVoltage == 3: # doi.
				if (delta_time > 2):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 0

		elif self.charger_requir == self.charger_off:
			if self.flag_afterChager == 1:   # sau khi tat sac doi T s roi moi do dien ap.
				self.pre_timeVoltage = time_curr
				self.flag_afterChager = 0
				self.charger_write = self.charger_off
				self.step_readVoltage = 0
			else:
				if (delta_time > self.timeCheckVoltage_normal):
					self.pre_timeVoltage = time_curr
					bat = round(self.main_info.voltages, 1)*10
					# print "normal --"
					if  bat > 255:
						self.valueVoltage = 255
					elif bat < 0:
						self.valueVoltage = 0
					else:
						self.valueVoltage = int(bat)

	def calculate_angle(self, qua1, qua2): # geometry_msgs/Orientation
		euler1 = self.quaternion_to_euler(qua1)
		euler2 = self.quaternion_to_euler(qua2)

		delta_angle = euler2 - euler1
		if (abs(delta_angle) >= pi):
			if (delta_angle >= 0):
				delta_angle = (pi*2 - abs(delta_angle))*(-1)
			else:
				delta_angle = pi*2 - abs(delta_angle)
		return delta_angle

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def getPose_from_offset(self, pose_in, offset):
		pose_out = Pose()
		angle = self.quaternion_to_euler(pose_in.orientation)

		if (angle >= 0):
			angle_target = angle - pi
		else:
			angle_target = pi + angle

		pose_out.position.x = pose_in.position.x + cos(angle_target)*offset
		pose_out.position.y = pose_in.position.y + sin(angle_target)*offset

		pose_out.orientation = self.euler_to_quaternion(angle_target)
		return pose_out

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def find_element(self, value_find, list_in):
		lenght = len(list_in)
		for i in range(lenght):
			if (value_find == list_in[i]):
				return 1
		return 0

	# ------------------------ Detect Lost ------------------------ #
	def detectLost_goalControl(self):
		delta_t = rospy.Time.now() - self.timeStampe_statusGoalControl
		if (delta_t.to_sec() > 1.0):
			return 1
		return 0

	def detectLost_driver(self):
		delta_t = rospy.Time.now() - self.timeStampe_driver
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_RTC(self):
		delta_t = rospy.Time.now() - self.timeStampe_RTC
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_HC(self):
		delta_t = rospy.Time.now() - self.timeStampe_HC
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_OC12(self):
		delta_t = rospy.Time.now() - self.timeStampe_conveyor12
		if (delta_t.to_sec() > 1.2):
			return 1
		return 0

	def detectLost_OC34(self):
		delta_t = rospy.Time.now() - self.timeStampe_conveyor34
		if (delta_t.to_sec() > 1.2):
			return 1
		return 0

	def detectLost_OC56(self):
		delta_t = rospy.Time.now() - self.timeStampe_conveyor56
		if (delta_t.to_sec() > 1.2):
			return 1
		return 0

	def detectLost_Main(self):
		delta_t = rospy.Time.now() - self.timeStampe_main
		if (delta_t.to_sec() > 1.0):
			return 1
		return 0

	def detectLost_CPD(self):
		delta_t = rospy.Time.now() - self.timeStampe_CPD
		if (delta_t.to_sec() > 1.2):
			return 1
		return 0

	def detectLost_nav(self):
		delta_t = rospy.Time.now() - self.nav350_data.header.stamp
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_poseRobot(self):
		delta_t = rospy.Time.now() - self.robotPose_nav.header.stamp
		if (delta_t.to_sec() > 0.5):
			return 1
		return 0

	# -- add 19/01/2022
	def detectLost_server(self):
		delta_t = rospy.Time.now() - self.timeStampe_TrafficReceived
		if (delta_t.to_sec() > 15):
			delta_s = rospy.Time.now() - self.saveTime_checkServer
			if (delta_s.to_sec() > 5):
				self.saveTime_checkServer = rospy.Time.now()
				self.saveStatus_server = self.check_server()

			if (self.saveStatus_server == 1):
				return 2
			elif (self.saveStatus_server == 2):
				return 3
			return 1
		return 0

	def detectLost_reflectors(self):
		# -- so luong guong
		if (self.nav350_data.number_reflectors >= 3): # loi mat guong
			self.timeStampe_reflectors = rospy.Time.now()

		delta_t = rospy.Time.now() - self.timeStampe_reflectors
		if (delta_t.to_sec() > 1.2):
			return 1
		return 0

	# ------------------------ ------------------------ #
	def synthetic_error(self):
		listError_now = []
		# -- EMG
		if self.main_info.EMC_status == 1:
			listError_now.append(121)
		else:
			# -- Lost Driver 1
			if self.detectLost_driver() == 1: 
				listError_now.append(251)

			# -- Error Driver 1
			summation1 = self.driver1_respond.alarm_all + self.driver1_respond.alarm_overload + self.driver1_respond.warning
			if (summation1 != 0):
				listError_now.append(252)

			# -- Lost Driver 2
			if self.detectLost_driver() == 1: 
				listError_now.append(261)

			# -- Error Driver 2
			summation2 = self.driver2_respond.alarm_all + self.driver2_respond.alarm_overload + self.driver2_respond.warning
			if (summation2 != 0):
				listError_now.append(262)

		# -- Va cham
		if self.HC_info.vacham == 1:
			listError_now.append(122)

		# -- Lost RTC Board.
		if (self.detectLost_RTC() == 1):
			listError_now.append(311)
		else:
			# -- Error RTC Board: CAN not Send.
			# if (self.() == 1):
			# 	listError_now.append(312)

			# -- Lost Main Board
			if (self.detectLost_Main() == 1):
				listError_now.append(321)

			# -- Lost HC Board.
			if (self.detectLost_HC() == 1):
				listError_now.append(351)

			# -- Lost OC Board No.12
			if (self.detectLost_OC12() == 1):
				listError_now.append(344)

			# -- Lost OC Board No.34
			if (self.detectLost_OC34() == 1):
				listError_now.append(345)

			# -- Lost OC Board No.56
			if (self.detectLost_OC56() == 1):
				listError_now.append(346)

			# -- Lost CPD Board
			if (self.detectLost_CPD() == 1):
				listError_now.append(361)

		# -- Goal Control
		# if self.detectLost_goalControl() == 1:
		# 	listError_now.append(282)

		# -- Lost Nav350
		if (self.detectLost_nav() == 1):
			listError_now.append(221)

		# -- Lost pose robot
		if (self.detectLost_poseRobot() == 1):
			listError_now.append(222)

		# -- Lỗi không thể định vị tọa độ.
		# if self.detectLost_reflectors() == 1: # 
		# 	listError_now.append(272)

		# -- Loi Code Parking:
		if (self.parking_status.warning == 2):
			listError_now.append(281)

		# -- 19/01/2022 - Mat giao tiep voi Server 
		# sts_sr = self.detectLost_server()
		# if sts_sr == 1: # lost server
		# 	listError_now.append(431)
		# elif sts_sr == 2: # lost server: Ping
		# 	listError_now.append(432)
		# elif sts_sr == 3: # lost server: IP
		# 	listError_now.append(433)

		# -- Cảnh báo: Low battery
		if self.voltage < 23:
			listError_now.append(451)

		# -- Cảnh báo: Co vat can
		if (self.status_goalControl.safety == 1):
			listError_now.append(411)

		# -- Cảnh báo: Co vat can khi di chuyen parking
		if (self.parking_status.warning == 1):
			listError_now.append(412)
			
		# -- Cảnh báo: AGV dung do da di het danh sach diem.
		if self.status_goalControl.complete_misson == 2:
			if self.status_goalControl.misson == 1 or self.status_goalControl.misson == 3:
				listError_now.append(441)
				self.flag_listPoint_ok = 1

		# -- Cảnh báo: Không sạc được Pin.
		if self.flag_notCharger == 1:
			listError_now.append(452)

		# -- Cảnh báo: Băng tải không nhận được thùng hàng..
		# 	listError_now.append(481)

		# -- Cảnh báo: Băng tải không trả được thùng hàng.
		# 	listError_now.append(482)

		# -- Cảnh báo: Vị Trí Trả Đang Còn Thùng Hàng.
		# 	listError_now.append(483)

		# -- Cảnh báo: AGV Đang Tạm Dừng Để Nhường Đường Cho AGV Khác
		# 	listError_now.append(442)

		return listError_now

	def resetAll_variable(self):
		self.enable_moving  = 0
		self.enable_parking = 0
		self.enable_mission = 0

		self.completed_before_mission = 0
		self.completed_after_mission = 0
		self.completed_move = 0
		self.completed_moveSimple = 0
		self.completed_moveSpecial = 0
		self.completed_backward = 0
		self.completed_checkConveyors = 0

		self.mission = 0
		self.flag_listPoint_ok = 0

		# rospy.logwarn("Update new target from: X= %s | Y= %s to X= %s| Y= %s", self.target_x, self.target_y, self.Traffic_cmdRequest.target_x, self.Traffic_cmdRequest.target_y)
		self.log_mess("info", "Update new target: X_new = ", self.Traffic_cmdRequest.target_x)
		self.target_x = self.Traffic_cmdRequest.target_x
		self.target_y = self.Traffic_cmdRequest.target_y
		self.target_z = self.Traffic_cmdRequest.target_z
		self.target_tag = self.Traffic_cmdRequest.tag
		self.mission_before = self.Traffic_cmdRequest.before_mission
		self.mission_after = self.Traffic_cmdRequest.after_mission
		# -- add 12/11/2021
		self.flag_resetFramework = 0
		self.flag_Auto_to_Byhand = 0
		# -- add 22/01/2022
		self.flag_notCharger = 0

	def run_maunal(self):
		cmd_vel = Twist()
		# - Tiến.
		if (self.app_button.bt_forwards == True):
			if (self.HC_info.zone_sick_ahead == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0
			else:
				if (self.HC_info.zone_sick_ahead == 2):
					cmd_vel.linear.x = 0.12
					cmd_vel.angular.z = 0.0
				else:
					cmd_vel.linear.x = 0.24
					cmd_vel.angular.z = 0.0

		# - Lùi.
		if (self.app_button.bt_backwards == True):
			if (self.HC_info.zone_sick_behind == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0
			else:
				if (self.HC_info.zone_sick_behind == 2):
					cmd_vel.linear.x = -0.12
					cmd_vel.angular.z = 0.0
				else:
					cmd_vel.linear.x = -0.24
					cmd_vel.angular.z = 0.0

		# - Xoay Trái.
		if (self.app_button.bt_rotation_left == True):
			if (self.HC_info.zone_sick_ahead == 1 or self.HC_info.zone_sick_behind == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0				
			else:
				if (self.HC_info.zone_sick_ahead == 2 or self.HC_info.zone_sick_behind == 2):
					cmd_vel.linear.x = 0
					cmd_vel.angular.z = 0.12
				else:
					cmd_vel.linear.x = 0
					cmd_vel.angular.z = 0.24

		# - Xoay Phải.
		if (self.app_button.bt_rotation_right == True):
			if (self.HC_info.zone_sick_ahead == 1 or self.HC_info.zone_sick_behind == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0				
			else:
				if (self.HC_info.zone_sick_ahead == 2 or self.HC_info.zone_sick_behind == 2):
					cmd_vel.linear.x = 0
					cmd_vel.angular.z = -0.12
				else:
					cmd_vel.linear.x = 0
					cmd_vel.angular.z = -0.24

		# - Dừng.
		if (self.app_button.bt_stop == True):
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = 0.0

		if (self.safety_NAV.data == 1):
			cmd_vel = Twist()

		return cmd_vel

	def run(self):
		# ------ 
		self.listError =  self.synthetic_error()
		self.numberError = len(self.listError)
		lenght = len(self.listError)
		

		count_error = 0
		count_warning = 0
		for i in range(lenght):
			if (self.listError[i] < 400):
				count_error += 1
			else:
				count_warning += 1
		# --
		if count_error == 0 and count_warning == 0:
			self.flag_error = 0
			self.flag_warning = 0

		elif count_error == 0 and count_warning > 0:
			self.flag_error = 0
			self.flag_warning = 1

		else:
			self.flag_error = 1
			self.flag_warning = 0

		# -- 
		if self.flag_cancelMission == 0:
			if (self.flag_error == 1):
				self.statusAGV = self.statusAGV_error
			else:
				if (self.flag_warning == 1):
					self.statusAGV = self.statusAGV_warning
				else:
					self.statusAGV = self.statusAGV_allRight
		else:
			self.statusAGV = self.statusAGV_cancelMission

		# -- ERROR
		if self.app_button.bt_clearError == 1 or self.main_info.stsButton_reset == 0:
			# -- Send clear error
			self.flag_error = 0
			self.error_device = 0
			self.error_move = 0
			self.error_perform = 0
			# -
			if self.parking_status.warning == 2: # - Nếu Parking lỗi -> Cho phép Rest.
				self.enable_parking = 0
			# -
			self.EMC_write = self.EMC_writeOff
			self.EMC_reset = self.EMC_resetOn
			self.task_driver.data = self.taskDriver_resetRead
			
			# -- Xóa lỗi băng tải.
			if self.status_conveyor1.status < 0: # or self.status_conveyor1.status.data >= 3):
				self.control_conveyors.No1_mission = 0

			if self.status_conveyor2.status < 0: #  or self.status_conveyor2.status.data >= 3):
				self.control_conveyors.No2_mission = 0

			if self.status_conveyor3.status < 0: #  or self.status_conveyor3.status.data >= 3):
				self.control_conveyors.No3_mission = 0

			if self.status_conveyor4.status < 0: #  or self.status_conveyor4.status.data >= 3):
				self.control_conveyors.No4_mission = 0

			if self.status_conveyor5.status < 0: #  or self.status_conveyor5.status.data >= 3):
				self.control_conveyors.No5_mission = 0

			if self.status_conveyor6.status < 0: #  or self.status_conveyor6.status.data >= 3):
				self.control_conveyors.No6_mission = 0

			# -- add 22/01/2022
			self.flag_notCharger = 0

		else:
			self.task_driver.data = self.taskDriver_Read
			self.EMC_reset = self.EMC_resetOff	

		if self.process == 0: # khi moi khoi dong len
			self.enable_moving  = 0
			self.enable_parking = 0
			self.enable_mission = 0

			self.mode_operate = self.md_by_hand
			self.control_conveyors = Control_conveyors()

			self.led_effect = 0
			self.speaker_requir = self.spk_warn
			
			# -
			self.flag_error = 0
			self.error_device = 0
			self.error_move = 0
			self.error_perform = 0
			# -
			self.completed_before_mission = 0
			self.completed_after_mission = 0
			self.completed_checkConveyors = 0
			self.completed_move = 0
			# -
			self.control_conveyors.No1_mission = self.conveyorTask_stop
			self.control_conveyors.No2_mission = self.conveyorTask_stop
			self.control_conveyors.No3_mission = self.conveyorTask_stop
			self.control_conveyors.No4_mission = self.conveyorTask_stop
			self.control_conveyors.No5_mission = self.conveyorTask_stop
			self.control_conveyors.No6_mission = self.conveyorTask_stop
			
			self.process = 1

		elif self.process == 1:	# chờ cac node khoi dong xong.
			ct = 8
			if ct == 8:
				self.process = 2

		elif self.process == 2: # - Read app
			if self.app_button.bt_passHand == 1:
				if self.mode_operate == self.md_auto: # - Kéo cờ báo kiểm tra trạng thái Lệnh tự động sau khi chuyển chế độ. 
					self.flag_Auto_to_Byhand = 1				
				self.mode_operate = self.md_by_hand
				
			if self.app_button.bt_passAuto == 1:
				self.mode_operate = self.md_auto

			if self.mode_operate == self.md_by_hand:
				self.process = 30
			elif self.mode_operate == self.md_auto:
				self.process = 40
	# ------------------------------------------------------------------------------------
	# -- BY HAND:
		elif self.process == 30:
			self.job_doing = 20
			# ------------ Stop Auto ------------ #
			self.enable_moving = 0
			self.enable_parking = 0
			self.enable_mission = 1

			# ------------ Navigation ------------ #
			if self.flag_error == 0:
			  	# -- Send vel
				if self.status_conveyor1.status == 0 or self.status_conveyor1.status >= 3:  
					stsRun_cy1 = 0
				else:
					stsRun_cy1 = 1

				if self.status_conveyor2.status == 0 or self.status_conveyor2.status >= 3:  
					stsRun_cy2 = 0
				else:
					stsRun_cy2 = 1

				if self.status_conveyor3.status == 0 or self.status_conveyor3.status >= 3:  
					stsRun_cy3 = 0
				else:
					stsRun_cy3 = 1

				if self.status_conveyor4.status == 0 or self.status_conveyor4.status >= 3:  
					stsRun_cy4 = 0
				else:
					stsRun_cy4 = 1

				if self.status_conveyor5.status == 0 or self.status_conveyor5.status >= 3:  
					stsRun_cy5 = 0
				else:
					stsRun_cy5 = 1

				if self.status_conveyor6.status == 0 or self.status_conveyor6.status >= 3:  
					stsRun_cy6 = 0
				else:
					stsRun_cy6 = 1

				# - >> Băng tải đang vận hành -> ko cho phép di chuyển <<
				if stsRun_cy1 == 0 and stsRun_cy2 == 0 and stsRun_cy3 == 0 and stsRun_cy4 == 0 and stsRun_cy5 == 0 and stsRun_cy6 == 0:  
					# -- Move
					vel_manual = self.run_maunal()
					self.pub_cmdVel(vel_manual, self.rate_cmdvel, rospy.get_time())
				else:
					self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

			else: # -- Has error
				self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())
				self.control_CPD.output1 = 0
				self.control_CPD.output2 = 0
				self.control_CPD.output3 = 0
				self.control_CPD.output4 = 0
				self.control_CPD.output5 = 0
				self.control_CPD.output6 = 0
				self.control_CPD.output7 = 0
				self.control_CPD.output8 = 0

			# ------------ Conveyor ------------ #
			# - Conveyor No.1
			if self.app_button.bt_cy1_received == True:
				self.control_conveyors.No1_mission = self.conveyorTask_received

			if self.app_button.bt_cy1_transmit == True:
				self.control_conveyors.No1_mission = self.conveyorTask_transmit

			if self.app_button.bt_cy1_stop == True:
				self.control_conveyors.No1_mission = self.conveyorTask_stop

			# - Conveyor No.2
			if self.app_button.bt_cy2_received == True:
				self.control_conveyors.No2_mission = self.conveyorTask_received

			if self.app_button.bt_cy2_transmit == True:
				self.control_conveyors.No2_mission = self.conveyorTask_transmit

			if self.app_button.bt_cy2_stop == True:
				self.control_conveyors.No2_mission = self.conveyorTask_stop

			# - Conveyor No.3
			if self.app_button.bt_cy3_received == True:
				self.control_conveyors.No3_mission = self.conveyorTask_received

			if self.app_button.bt_cy3_transmit == True:
				self.control_conveyors.No3_mission = self.conveyorTask_transmit

			if self.app_button.bt_cy3_stop == True:
				self.control_conveyors.No3_mission = self.conveyorTask_stop

			# - Conveyor No.4
			if self.app_button.bt_cy4_received == True:
				self.control_conveyors.No4_mission = self.conveyorTask_received

			if self.app_button.bt_cy4_transmit == True:
				self.control_conveyors.No4_mission = self.conveyorTask_transmit

			if self.app_button.bt_cy4_stop == True:
				self.control_conveyors.No4_mission = self.conveyorTask_stop

			# - Conveyor No.5
			if self.app_button.bt_cy5_received == True:
				self.control_conveyors.No5_mission = self.conveyorTask_received

			if self.app_button.bt_cy5_transmit == True:
				self.control_conveyors.No5_mission = self.conveyorTask_transmit

			if self.app_button.bt_cy5_stop == True:
				self.control_conveyors.No5_mission = self.conveyorTask_stop

			# - Conveyor No.6
			if self.app_button.bt_cy6_received == True:
				self.control_conveyors.No6_mission = self.conveyorTask_received

			if self.app_button.bt_cy6_transmit == True:
				self.control_conveyors.No6_mission = self.conveyorTask_transmit

			if self.app_button.bt_cy6_stop == True:
				self.control_conveyors.No6_mission = self.conveyorTask_stop

			# ------------ Speaker ------------ #
			# -- Speaker
			if self.app_button.bt_speaker == True:
				self.enable_speaker = 1
			else:
				self.enable_speaker = 0

			# ------------ Charger ------------ #
			if self.app_button.bt_charger == True:
				self.charger_requir = self.charger_on
			else:
				self.charger_requir = self.charger_off

			# ------------ Brake ------------ #
			if self.app_button.bt_brake == True:
				self.disable_brake.data = 1
			else:
				self.disable_brake.data = 0

			# ------------ Toyo Write ------------ #
			self.control_CPD.output1 = self.app_button.bt_toyoWrite_1
			self.control_CPD.output2 = self.app_button.bt_toyoWrite_2
			self.control_CPD.output3 = self.app_button.bt_toyoWrite_3
			self.control_CPD.output4 = self.app_button.bt_toyoWrite_4
			self.control_CPD.output5 = self.app_button.bt_toyoWrite_5
			self.control_CPD.output6 = self.app_button.bt_toyoWrite_6
			self.control_CPD.output7 = self.app_button.bt_toyoWrite_7
			self.control_CPD.output8 = self.app_button.bt_toyoWrite_8

			self.process = 2

	# -- RUN AUTO:
		elif self.process == 40: # -- kiem tra loi
			if self.flag_error == 1: # 
				self.enable_moving = 0
				self.enable_parking = 0
				self.enable_mission = 0
				self.process = 2

			else:
				self.process = 2 # 41

		elif self.process == 41:    # kiem tra muc tieu thay doi
			pass
			# if ( self.target_x != self.Traffic_cmdRequest.target_x ) or ( self.target_y != self.Traffic_cmdRequest.target_y) or ( self.target_tag != self.Traffic_cmdRequest.tag):
			# 	if (self.Traffic_cmdRequest.target_x < 500) and (self.Traffic_cmdRequest.target_y < 500):
			# 		# self.move_req = Move_request()

			# 		# Khong che phep doi len khi dang:
			# 		# - 1, Nang hoac Ha.
			# 		# - 2, Dang di vao ke.
			# 		# - 3, Dang di ra khoi ke.
			# 		a1 = 0
			# 		a2 = 0
			# 		if self.lift_status.status == -1 or self.lift_status.status == 1 or self.lift_status.status == 2:  # Ban nang: Dung hoac Hoan thanh.
			# 			a1 = 1
			# 		else:
			# 			a1 = 0

			# 		if self.parking_status.status > 8 and self.parking_status.status <= 11: # Dang di vao trong ke -> ko cho doi lenh.
			# 			a2 = 1 # thay doi
			# 			# if (self.flag_error == 1 and self.numberError == 121): # EMC
			# 			# 	a2 = 0
			# 				# self.enable_parking = 0
			# 			# else:
			# 			# 	a2 = 1
			# 		else:
			# 			a2 = 0

			# 		if a1 == 1 or a2 == 1:
			# 			self.log_mess("warn", "Have new target but must Waiting perform done ....", 0)
			# 			self.process = 42
			# 		else:
			# 			self.resetAll_variable()
			# 			self.process = 42
			# 	else:
			# 		self.log_mess("info", "Have new target but Not fit: X= ", self.Traffic_cmdRequest.target_x)
			# 		self.log_mess("info", "Have new target but Not fit: y= ", self.Traffic_cmdRequest.target_y)
			# 		self.process = 2
			# 	self.Traffic_infoRespond.offset = 0 # --
			# else:
			# 	# Sử dụng trong trường hợp Lỗi vẫn hành: đã thực hiện xong nhiệm vụ di chuyển -> lái tay sang vị trí khác -> AGV đứng im (lẽ ra phải di chuyển đến đích)
			# 	# if self.completed_move == 1:
			# 	# 	if self.target_x < 500 and self.target_y < 500:
			# 	# 		if self.point_same_point(self.target_x, self.target_y, self.target_z, self.Traffic_infoRespond.x, self.Traffic_infoRespond.y, self.Traffic_infoRespond.z) == 1:
			# 	# 			self.completed_move = 0

			# 	# Nếu target ko đổi mà nhiện vụ muốn thay đổi (lấy hoặc trả hàng luôn tại đó).
			# 	if self.completed_after_mission == 1:
			# 		if self.mission_after != self.Traffic_cmdRequest.after_mission:
			# 			self.completed_after_mission = 0
			# 			self.log_mess("info", "After mission change to ", self.Traffic_cmdRequest.after_mission)
			# 			self.mission_after = self.Traffic_cmdRequest.after_mission

				# self.process = 42

		elif self.process == 42: 
			pass
			# if self.flag_Auto_to_Byhand == 1: 
			# 	# -- add 12/11/2021 : Chay lai quy trinh Vao Sac khi Chuuyen che do.
			# 	if self.completed_after_mission == 1:
			# 		if self.mission_after == self.serverMission_liftDown_charger or self.mission_after == self.serverMission_charger: #
			# 			delta_distance = self.calculate_distance(self.robotPose_nav.pose.position, self.poseWait.position)
			# 			# print ("poseWait: " + str(self.poseWait.position.x) + " | " + str(self.poseWait.position.y))
			# 			# print ("robotPose_nav: " + str(self.robotPose_nav.pose.position.x) + " | " + str(self.robotPose_nav.pose.position.y))
			# 			# print ("delta_distance: ", delta_distance)
			# 			if (delta_distance > self.distance_resetMission):
			# 				self.resetAll_variable()
			# 				self.completed_backward = 1
			# 	else:
			# 		# -- add 23/12/2021: Xu ly loi Dang Parking thi bi chuyen che do -> AGV cu parking.
			# 		if self.completed_before_mission == 1 and self.completed_moveSimple == 1 and self.completed_moveSpecial == 0:
			# 			delta_distance = self.calculate_distance(self.robotPose_nav.pose.position, self.pose_parkingRuning.position)
			# 			# -- phien ban 1: 
			# 			# delta_angle = self.calculate_angle(self.robotPose_nav.pose.orientation, self.pose_parkingRuning.orientation)
			# 			# -- phien ban 2: Xac dinh do lech giua goc cua AGV voi goc cua diem vao Tag
			# 			delta_angle = self.calculate_angle(self.robotPose_nav.pose.orientation, self.parking_poseTarget.orientation)
			# 			# print ("--------------------")
			# 			# print ("delta_distance: ", delta_distance)
			# 			# print ("delta_angle: ", degrees(delta_angle) )

			# 			if (delta_distance > 0.2 or abs(delta_angle) > radians(20)):
			# 				self.completed_moveSimple = 0

			# 	# -- add 27/12/2021: Sua loi cu di thang ra sau khi parking
			# 	if self.completed_before_mission == 1 and self.completed_checkConveyors == 1 and self.completed_backward == 0:
			# 		delta_distance = self.calculate_distance(self.robotPose_nav.pose.position, self.cancelbackward_pose.position)
			# 		delta_angle = self.calculate_angle(self.robotPose_nav.pose.orientation, self.cancelbackward_pose.orientation)

			# 		if delta_distance > self.cancelbackward_offset*0.85 or abs(delta_angle) > radians(20) :
			# 			self.completed_backward = 1

			# 	# -- add 18/01/2022
			# 	if (self.flag_listPoint_ok == 1):
			# 		self.Traffic_cmdRequest.list_id = self.list_id_unknown
			# 		self.flag_listPoint_ok = 0

			# 	self.job_doing = 1
			# 	# thuc hien lai nhiem vu nang, ha, sac sau khi chuyen che do tu tu dong sang bang tay.
			# 	if self.completed_after_mission == 0 and self.completed_before_mission == 0:
			# 		self.flag_Auto_to_Byhand = 0

			# 	elif self.completed_after_mission == 0 and self.completed_before_mission == 1:
			# 		if self.mission_before == self.serverMission_unknown:
			# 			self.flag_Auto_to_Byhand = 0

			# 			self.completed_checkConveyors = 1
			# 			self.Traffic_infoRespond.task_status = self.serverMission_unknown

			# 		elif self.mission_before == self.serverMission_liftDown: # Hạ
			# 			self.liftTask = self.liftDown	

			# 			if self.lift_status.status.data == 3:  # Hoàn thành
			# 				self.liftTask = self.liftStop
			# 				self.flag_Auto_to_Byhand = 0

			# 				self.completed_checkConveyors = 1
			# 				self.Traffic_infoRespond.task_status = self.serverMission_liftDown

			# 		elif self.mission_before == self.serverMission_liftUp: # Nâng
			# 			self.liftTask = self.liftUp				
			# 			if self.lift_status.status.data == 4:  # Hoàn thành
			# 				self.liftTask = self.liftStop
			# 				self.flag_Auto_to_Byhand = 0

			# 				self.completed_checkConveyors = 0
			# 				self.Traffic_infoRespond.task_status = self.statusTask_liftError

			# 		else: 
			# 			self.flag_Auto_to_Byhand = 0

			# 			self.completed_checkConveyors = 1
			# 			self.Traffic_infoRespond.task_status = self.mission_before

			# 	elif self.completed_after_mission == 1 and self.completed_before_mission == 1:
			# 		if self.mission_after == self.serverMission_unknown:
			# 			self.flag_Auto_to_Byhand = 0

			# 			self.completed_checkConveyors = 1
			# 			self.Traffic_infoRespond.task_status = self.serverMission_unknown

			# 		elif self.mission_after == self.serverMission_liftDown: # Hạ
			# 			self.liftTask = self.liftDown	

			# 			if self.lift_status.status.data == 3:  # Hoàn thành
			# 				self.liftTask = self.liftStop
			# 				self.flag_Auto_to_Byhand = 0

			# 				self.completed_checkConveyors = 1
			# 				self.Traffic_infoRespond.task_status = self.serverMission_liftDown


			# 		elif self.mission_after == self.serverMission_liftUp: # Nâng
			# 			self.liftTask = self.liftUp				
			# 			if self.lift_status.status.data == 4:  # Hoàn thành
			# 				self.liftTask = self.liftStop
			# 				self.flag_Auto_to_Byhand = 0

			# 				self.completed_checkConveyors = 0
			# 				self.Traffic_infoRespond.task_status = self.statusTask_liftError

			# 		elif self.mission_after == self.serverMission_liftDown_charger: # Hạ
			# 			self.liftTask = self.liftDown				
			# 			if self.lift_status.status.data == 3:  # Hoàn thành
			# 				self.charger_requir = self.charger_on
			# 				self.liftTask = self.liftStop
			# 				self.flag_Auto_to_Byhand = 0
		
			# 				self.completed_checkConveyors = 1
			# 				self.Traffic_infoRespond.task_status = self.serverMission_liftDown_charger

			# 		elif self.mission_after == self.serverMission_charger: # - Sac
			# 			self.charger_requir = self.charger_on
			# 			self.flag_Auto_to_Byhand = 0
			# 			self.completed_checkConveyors = 1
			# 			self.Traffic_infoRespond.task_status = self.serverMission_charger

			# 		else: 
			# 			self.flag_Auto_to_Byhand = 0
			# 			self.completed_checkConveyors = 1
			# 			self.Traffic_infoRespond.task_status = self.mission_after
			# 	# -- add 18/11
			# 	else:
			# 		self.flag_Auto_to_Byhand = 0

			# 	self.process = 2
			# else:
			# 	self.process = 43
		
		elif self.process == 43: 	# Thực hiện nhiệm vụ trước.
			pass
			# if self.completed_before_mission == 0: # chua thuc hien
				
			# 	self.job_doing = 2 
			# 	if self.mission_before == 0:
			# 		self.charger_requir = self.charger_off
			# 		self.log_mess("info", "Before mission Not have", self.mission_before)
			# 		self.completed_before_mission = 1
			# 		# -- add new
			# 		self.completed_checkConveyors = 1
			# 		self.Traffic_infoRespond.task_status = 0

			# 	elif self.mission_before == self.serverMission_liftDown: # Hạ
			# 		self.charger_requir = self.charger_off
			# 		self.liftTask = self.liftDown
					
			# 		if self.lift_status.status.data == 3:  # Hoàn thành
			# 			self.log_mess("info", "Before mission completed", self.serverMission_liftDown)
			# 			self.liftTask = self.liftStop
			# 			self.completed_before_mission = 1

			# 			self.completed_checkConveyors = 1
			# 			self.Traffic_infoRespond.task_status = self.serverMission_liftDown


			# 	elif self.mission_before == self.serverMission_liftUp: # Nâng
			# 		# -- add 14/11 Them phan kiem tra tai

			# 		self.charger_requir = self.charger_off
			# 		self.liftTask = self.liftUp		
			# 		if self.lift_status.status.data == 4:  # Hoàn thành
			# 			self.log_mess("info", "Before mission completed", self.serverMission_liftUp)
			# 			self.liftTask = self.liftStop
			# 			self.completed_before_mission = 1

			# 			self.completed_checkConveyors = 0
			# 			self.Traffic_infoRespond.task_status = self.statusTask_liftError

			# 	# -- add 20/01/2022
			# 	else:
			# 		self.completed_before_mission = 1

			# 		self.completed_checkConveyors = 1
			# 		self.Traffic_infoRespond.task_status = self.mission_before

			# 	self.process = 2
			# else:
			# 	self.process = 44

		elif self.process == 44:	# Thuc hien kiểm tra kệ có trên bàn nâng ko.
			pass
			# if self.completed_checkConveyors == 0:
			# 	self.job_doing = 3
			# 	if self.mission_before == self.serverMission_liftUp or self.mission_after == self.serverMission_liftUp: # Nâng
			# 		if self.lift_status.sensorLift.data == 0:
			# 			self.lastTime_checkLift = time.time()

			# 		t = (time.time() - self.lastTime_checkLift)%60
			# 		if (t > 2): # 2 s						
			# 			self.completed_checkConveyors = 1
			# 			self.Traffic_infoRespond.task_status = self.serverMission_liftUp
			# 	else:
			# 		self.completed_checkConveyors = 1
			# 	self.process = 2
			# else:
			# 	self.process = 45

		elif self.process == 45:	# Thuc hien di chuyen lui.
			pass
			# if self.completed_backward == 1:
			# 	self.process = 46
			# else:
			# 	self.job_doing = 4
			# 	# -- add 15/04/2022
			# 	if self.check_listPoints(self.Traffic_cmdRequest.list_id) == 1:
			# 		if self.flag_requirBackward == 1:
			# 			self.move_req.target_x = self.backward_x
			# 			self.move_req.target_y = self.backward_y
			# 			self.move_req.target_z = self.backward_z
			# 			if self.status_goalControl.misson == 2 and self.status_goalControl.complete_misson == 1:  # Hoan thanh di chuyen lui.
			# 				self.flag_requirBackward = 0
			# 				self.enable_moving = 0
			# 				self.completed_backward = 1	
			# 			else:
			# 				self.enable_moving = 2
			# 		else:
			# 			self.completed_backward = 1	
			# 			self.enable_moving = 0
			# 		# -- add 15/04/2022
			# 		self.flag_listPointEmpty = 0
			# 		self.process = 2
			# 	else:
			# 		self.enable_moving = 0
			# 		self.flag_listPointEmpty = 1
			# 		self.log_mess("info", "process = 43 - Buoc lui ra, Danh sach diem Null -> ko chay", 0)
			# 		self.process = 2

		elif self.process == 46:	# Thuc hien di chuyen diem thuong.
			pass
			# if self.completed_moveSimple == 1:      # 
			# 	self.process = 47
			# 	self.enable_moving = 0
			# else:
			# 	self.job_doing = 5
			# 	if (len(self.Traffic_cmdRequest.list_x) != 0) and (len(self.Traffic_cmdRequest.list_y) != 0) and (self.target_x < 500) and (self.target_y < 500):
			# 		self.move_req.target_x = self.target_x
			# 		self.move_req.target_y = self.target_y
			# 		self.move_req.target_z = self.target_z
					
			# 		self.parking_offset = self.Traffic_cmdRequest.offset

			# 		self.move_req.tag = self.Traffic_cmdRequest.tag
			# 		self.move_req.offset = self.Traffic_cmdRequest.offset

			# 		self.move_req.list_id = self.Traffic_cmdRequest.list_id
			# 		self.move_req.list_x = self.Traffic_cmdRequest.list_x
			# 		self.move_req.list_y = self.Traffic_cmdRequest.list_y
			# 		self.move_req.list_speed = self.Traffic_cmdRequest.list_speed

			# 		if self.mission_before == self.serverMission_liftUp: # Nâng
			# 			self.move_req.mission = 1
			# 		else:
			# 			self.move_req.mission = 0
					
			# 		# -- add 19/01/2022 : chuyen vung sick.
			# 		if self.mission_before == self.serverMission_liftUp: # Nâng
			# 			self.enable_moving = 1 # -- vung To
			# 		else:
			# 			self.enable_moving = 3 # -- vung Nho

			# 		# self.enable_moving = 1
			# 	else:
			# 		self.log_mess("warn", "ERROR: Target of List point wrong !!!", 0)

			# 	if self.status_goalControl.complete_misson == 1 :  # Hoan thanh di chuyen.
			# 		if self.status_goalControl.misson == 1 or self.status_goalControl.misson == 3:
			# 			self.completed_moveSimple = 1
			# 			self.enable_moving = 0
			# 			self.log_mess("info", "Move completed", 0)
			# 	self.process = 2

	 	# -- Parking	
		elif self.process == 47:   #  
			pass
		  	# print "46 --"
			# if self.completed_moveSpecial == 1:
			# 	self.completed_move = 1
			# 	self.process = 34
			# else:
			# 	self.job_doing = 6
			# 	self.process = 48 # kiem tra diem vao ke

		elif self.process == 48:   # Parking
			pass
			# if self.completed_moveSpecial == 0: # chua hoan thanh di chuyen
			# 	# sau sẽ thêm phần khi đổi mã tag thì tự động reset paking.
			# 	if self.parking_status.status == 1: # Free
			# 		self.process = 50
			# 		# self.log_mess("info", "Special point: readly", self.parking_status.status)

			# 	elif self.parking_status.status == 51: # -- Completed Run
			# 		# self.log_mess("info", "Special point: Completed to point: ", self.parking_status.status)
			# 		self.enable_parking = 0
			# 		self.completed_moveSpecial = 1
			# 		self.flag_requirBackward = 1   # 1 - yeu cau 
			# 		self.backward_x = self.target_x
			# 		self.backward_y = self.target_y
			# 		self.backward_z = self.target_z
			# 		self.process = 2

			# 		# -- add 12/11/2021
			# 		self.poseWait = self.robotPose_nav.pose

			# 		# -- add 27/12/2021
			# 		self.cancelbackward_pose = self.robotPose_nav.pose
			# 		self.cancelbackward_offset = self.parking_offset
			# 	else:
			# 		self.process = 2

			# 	# -- add 23/12/2021:
			# 	self.pose_parkingRuning = self.robotPose_nav.pose
			# else:
			# 	self.process = 2

		elif self.process == 50: 	# Yêu cầu di chuyển.
			pass
			# if self.mission_after == self.serverMission_liftDown_charger or self.mission_after == self.serverMission_charger:
			# 	self.enable_parking = 2
			# elif self.mission_after == self.serverMission_liftDown:
			# 	self.enable_parking = 1
			# else:
			# 	self.enable_parking = 3

			# self.parking_offset = self.Traffic_cmdRequest.offset
			# # -
			# self.parking_poseBefore.position.x = self.Traffic_cmdRequest.target_x
			# self.parking_poseBefore.position.y = self.Traffic_cmdRequest.target_y
			# self.parking_poseBefore.orientation = self.euler_to_quaternion(self.Traffic_cmdRequest.target_z)
			# self.parking_poseTarget = self.getPose_from_offset(self.parking_poseBefore, self.parking_offset)

			# self.log_mess("info", "Tag offset requir: ", self.parking_offset)
			# self.process = 2
	# ------------------------------------------------------------------------------------
		elif self.process == 34:	# -- Thực hiện nhiệm vụ sau.
			pass
			# if self.completed_after_mission == 0: # chua thuc hien
			# 	self.job_doing = 7
			# 	if self.mission_after == 0:
			# 		self.log_mess("info", "Last mission Not have Suf: ", self.mission_after)
			# 		self.completed_after_mission = 1

			# 		self.completed_checkConveyors = 1
			# 		self.Traffic_infoRespond.task_status = self.mission_after
					
			# 	elif self.mission_after == self.serverMission_liftDown: # Hạ

			# 		self.liftTask = self.liftDown

			# 		if self.lift_status.status.data == 3: # Hoàn thành
			# 			self.log_mess("info", "Last mission completed: ", self.serverMission_liftDown)
			# 			self.liftTask = self.liftStop
			# 			self.completed_after_mission = 1

			# 			self.completed_checkConveyors = 1
			# 			self.Traffic_infoRespond.task_status = self.serverMission_liftDown

			# 	elif self.mission_after == self.serverMission_liftUp: # Nâng

			# 		self.liftTask = self.liftUp				
			# 		if self.lift_status.status.data == 4: # Hoàn thành
			# 			self.log_mess("info", "Last mission completed: ", self.serverMission_liftUp)
			# 			self.liftTask = self.liftStop
			# 			self.completed_after_mission = 1
						
			# 			self.completed_checkConveyors = 0
			# 			self.Traffic_infoRespond.task_status = self.statusTask_liftError

			# 	elif self.mission_after == self.serverMission_charger: # sac
			# 		self.charger_requir = self.charger_on
			# 		self.log_mess("info", "Last mission completed: ", self.serverMission_charger)
			# 		self.completed_after_mission = 1

			# 		self.completed_checkConveyors = 1
			# 		self.Traffic_infoRespond.task_status = self.serverMission_charger
					
			# 	elif self.mission_after == self.serverMission_liftDown_charger: # sac

			# 		self.liftTask = self.liftDown
			# 		if self.lift_status.status.data == 3: # Hoàn thành
			# 			self.log_mess("info", "Last mission completed", self.serverMission_liftDown_charger)
			# 			self.liftTask = self.liftStop
			# 			self.completed_after_mission = 1
			# 			self.charger_requir = self.charger_on # turn on charger	

			# 			self.completed_checkConveyors = 1
			# 			self.Traffic_infoRespond.task_status = self.serverMission_liftDown_charger

			# 	# -- add 20/01/2022			
			# 	else:
			# 		self.completed_after_mission = 1

			# 		self.completed_checkConveyors = 1
			# 		self.Traffic_infoRespond.task_status = self.mission_after
					
			# 	self.process = 2
			# else:
			# 	self.process = 35

		elif self.process == 35:
			self.job_doing = 8
			self.process = 2
			self.log_mess("warn", "Wating new Target ...", 0)
			
	# ------------------------------------------------------------------------------------
		# -- Tag + Offset:
		if self.mode_operate == self.md_auto:
			if self.completed_move == 1:
				self.Traffic_infoRespond.tag = self.Traffic_cmdRequest.tag
				self.Traffic_infoRespond.offset = self.Traffic_cmdRequest.offset
			else:
				self.Traffic_infoRespond.tag = 0
				self.Traffic_infoRespond.offset = 0

		self.Traffic_infoRespond.status = self.statusAGV    # Status: Error
		self.Traffic_infoRespond.error_perform = self.process
		self.Traffic_infoRespond.error_moving = self.flag_error
		self.Traffic_infoRespond.error_device = self.numberError
		self.Traffic_infoRespond.listError = self.listError
		self.Traffic_infoRespond.process = self.job_doing

		# -- Battery - ok
		self.readbatteryVoltage()
		self.Traffic_infoRespond.battery = int(self.valueVoltage)
		# self.Traffic_infoRespond.battery = 255

		# -- mode respond server
		if self.mode_operate == self.md_by_hand:         # Che do by Hand
			self.Traffic_infoRespond.mode = 1

		elif self.mode_operate == self.md_auto:          # Che do Auto
			self.Traffic_infoRespond.mode = 2

		# ---------------- Speaker ---------------- #
		if self.flag_error == 1 and self.flag_warning == 1:
			self.speaker_requir = self.spk_error
		elif self.flag_error == 1 and self.flag_warning == 0:
			self.speaker_requir = self.spk_error
		elif self.flag_error == 0 and self.flag_warning == 1:
			self.speaker_requir = self.spk_warn			
		else:
			if self.completed_backward == 0 and self.completed_before_mission == 0 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 0 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 1 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:	
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 0:	
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 1:
				self.speaker_requir = self.spk_move
			else:
				self.speaker_requir = self.spk_move

		# ---------------- Board HC - LED ---------------- #
		if self.flag_error == 1:
			self.led_effect = self.led_error
		else:
			if self.completed_before_mission == 0 and self.completed_backward == 0 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				self.led_effect = self.led_perform

			elif self.completed_before_mission == 1 and self.completed_backward == 0 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				if (self.status_goalControl.safety == 1):
					self.led_effect = self.led_stopBarrier
				else:
					self.led_effect = self.led_simpleRun

			elif self.completed_before_mission == 1 and self.completed_backward == 1 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				if (self.status_goalControl.safety == 1):
					self.led_effect = self.led_stopBarrier
				else:
					self.led_effect = self.led_simpleRun

			elif self.completed_before_mission == 1 and self.completed_backward == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:	
				# self.led = self.led_specialRun
				if (self.parking_status.warning == 1):
					self.led_effect = self.led_stopBarrier
				else:
					self.led_effect = self.led_specialRun

			elif self.completed_before_mission == 1 and self.completed_backward == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 0:	
				self.led_effect = self.led_perform

			elif self.completed_before_mission == 1 and self.completed_backward == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 1:
				self.led_effect = self.led_completed

			else:
				self.led_effect = self.led_simpleRun
		
		# ---------------- Board Control - Publish ---------------- #
		time_curr = rospy.get_time()
		d = (time_curr - self.pre_timeBoard)
		if (d > float(1/self.FrequencePubBoard)): # < 20hz 
			self.pre_timeBoard = time_curr

			# -------- Request Board OC - Conveyors -------- #
			self.pub_controlConveyors.publish(self.control_conveyors)

			# -------- Request Board Main -------- #
			if self.flag_error == 0:
				# tat Loa khi sac thanh cong!
				if self.completed_after_mission == 1 and self.mode_operate == self.md_auto and (self.mission_after == self.serverMission_charger or self.mission_after == self.serverMission_liftDown_charger):
					if self.charger_write == self.charger_on:
						if self.main_info.charge_current >= self.charger_valueOrigin :
							self.speaker_effect = self.spk_off
							self.flag_notCharger = 0
						else:
							self.flag_notCharger = 1
							if self.enable_speaker == 1:
								self.speaker_effect = self.speaker_requir
							else:
								self.speaker_effect = self.spk_off	
					else:
						self.speaker_effect = self.spk_off
						self.flag_notCharger = 0
				else:
					self.flag_notCharger = 0
					if self.enable_speaker == 1:
						self.speaker_effect = self.speaker_requir
					else:
						self.speaker_effect = self.spk_off	
			else:
				self.flag_notCharger = 0
				if self.enable_speaker == 1:
					self.speaker_effect = self.speaker_requir
				else:
					self.speaker_effect = self.spk_off

			self.pub_Main(self.charger_write, self.speaker_effect, self.EMC_write, self.EMC_reset)  # MISSION

			# -------- Request Board HC -------- #
			self.HC_request.RBG1 = self.led_effect 
			self.HC_request.RBG2 = self.led_effect 
			self.pub_controlHC.publish(self.HC_request)

			# -------- Request CPD Board -------- #
			self.pub_controlCPD.publish(self.control_CPD)

			# -------- Request Task Driver Motor -------- #
			self.pub_taskDriver.publish(self.task_driver)
			
		# ---------------- Cancel Mission ---------------- #
		if self.cancelMission_control.data == 1:
			self.flag_cancelMission = 1

		if self.Traffic_cmdRequest.id_command == 0:
			self.flag_cancelMission = 0

		self.cancelMission_status = self.cancelMission_control
		self.pub_cancelMission.publish(self.cancelMission_status)

		# ---------------- Parking Control ---------------- #
		self.pub_park(self.enable_parking, self.parking_poseBefore, self.parking_poseTarget, self.parking_offset)

		# ---------------- Respond Client ---------------- #
		self.pub_infoRespond.publish(self.Traffic_infoRespond)    # Pub Client

		# ---------------- Request Navigation ---------------- #
		self.pub_move_req(self.enable_moving, self.move_req)  # Pub Navigation

		# ---------------- Brake ---------------- #
		self.pub_disableBrake.publish(self.disable_brake)

		self.rate.sleep()


def main():
	# Start the job threads
	class_1 = ros_control()
	# Keep the main thread running, otherwise signals are ignored.
	while not rospy.is_shutdown():
		class_1.run()

if __name__ == '__main__':
	main()

"""
Stt :
0: chờ đủ dữ liệu để parking
1: chờ tín hiệu parking
21:  tính khoảng cách tiến lùi
-21: thực hiện di chuyen tiến lùi
-210: thực hiện quay trước nếu gặp TH AGV bị lệch góc lớn
31: tính góc quay để lùi vào kệ
-31: thực hiện quay
41: Parking
51: completed - Đợi Reset
52: error: bien doi tf loi

"""