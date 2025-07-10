#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Phùng Quý Dương
Company: STI Viet Nam
Date: 13/7/2024

Update 13/7:
  + Layout lại các khối chức năng trong UI
  + Bổ sung các trường thông tin về NUC
  + Thêm tính năng test Loa, LED
  + Cải tiến chức năng:
    - Test điểm
	- Test gương
  + Thêm tính năng thêm điểm đặc biệt

Update 5/8"
  + Thêm reset trạng thái bàn nâng khi chuyển chế độ
  + Thêm hiển thị nút xóa lỗi đã được nhấn
  
"""

import sys
from math import sin , cos , pi , atan2
import time
import threading
import signal

import os
import re  
import subprocess
import argparse
from datetime import datetime

import sqlite3
import json
import math
# -- 
from app_interface import *

class Program(threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.shutdown_flag = threading.Event()
		# --
		self.name_card = rospy.get_param("name_card", "wlo2")
		self.name_card = "wlo2"

		self.address_traffic = rospy.get_param("address_traffic", "172.21.15.224")
		#self.address_traffic = "192.168.1.92"
		self.pre_timePing = time.time()
		# --
		rospy.init_node('app_ros', anonymous=False)
		self.rate = rospy.Rate(20)

		self.app = QApplication(sys.argv)
		self.welcomeScreen = WelcomeScreen()

		screen = self.app.primaryScreen()

		size = screen.size()
		print('Size: %d x %d' % (size.width(), size.height()))

		self.widget = QtWidgets.QStackedWidget()
		self.widget.addWidget(self.welcomeScreen)
		self.widget.setFixedHeight(585)
		self.widget.setFixedWidth(1024)
		self.widget.setWindowFlag(Qt.FramelessWindowHint)
		# --
		self.valueLable = valueLable()
		self.statusColor = statusColor()
		# -- 
		self.is_exist = 1
		# -----------------------------------------------------------
		# -- Break
		rospy.Subscriber("/enable_brake", Bool, self.callback_brakeControl) 
		self.status_brake = Bool()

		# -- Driver1
		rospy.Subscriber("/driver1_respond", Driver_respond, self.callback_driver1) 
		self.driver1_respond = Driver_respond()

		# -- Driver2
		rospy.Subscriber("/driver2_respond", Driver_respond, self.callback_driver2) 
		self.driver2_respond = Driver_respond()

		# -- HC
		rospy.Subscriber("/HC_info", HC_info, self.callback_HC) 
		self.HC_info = HC_info()

		# -- Main
		rospy.Subscriber("/POWER_info", POWER_info, self.callback_Main) 
		self.main_info = POWER_info()

		# -- OC board
		rospy.Subscriber("/lift_status", Lift_status, self.callback_OC_board) # lay thong tin trang thai mach dieu khien ban nang.
		self.OC_status = Lift_status()
		
		# -- PSU board
		rospy.Subscriber("/PSU_info", PSU_info, self.callback_Psu) 
		self.psu_info = PSU_info()

		# -- MCU board
		rospy.Subscriber("/MCU_info", MCU_info, self.callback_mcu) 
		self.mcu_info = MCU_info()

		rospy.Subscriber("/HCU_info", HCU_info, self.callback_hcu) 
		self.hcu_info = HCU_info()

		# -- Status Port
		rospy.Subscriber("/status_port", Status_port, self.callback_statusPort) 
		self.status_port = Status_port()

		# -- NUC info
		rospy.Subscriber("/nuc_info", Nuc_info, self.callback_nucInfo) 
		self.nuc_info = Nuc_info()
		# ------------------------------
		# -- data nav
		rospy.Subscriber("/nav350_data", Nav350_data, self.callback_nav350) 
		self.nav350_data = Nav350_data()

		# -- data safety NAV
		rospy.Subscriber("/safety_NAV", Int8, self.callback_safetyNAV) 
		self.safety_NAV = Int8()

		# -- Pose robot
		rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback_robotPose) 
		self.robotPose_nav = PoseStamped()

		# -- Traffic cmd
		rospy.Subscriber("/server_cmdRequest", Server_cmdRequest, self.callback_server_cmdRequest)
		self.server_cmdRequest = Server_cmdRequest()

		# -- Traffic cmd
		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.NN_cmdRequest_callback) 
		self.NN_cmdRequest = NN_cmdRequest()

		# -- Pose robot
		rospy.Subscriber("/NN_infoRequest", NN_infoRequest, self.callback_NN_infoRequest) 
		self.NN_infoRequest = NN_infoRequest()

		# -- info AGV
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_callback) 
		self.NN_infoRespond = NN_infoRespond()

		# -- info move
		rospy.Subscriber("/status_goal_control", Status_goal_control, self.goalControl_callback)
		self.status_goalControl = Status_goal_control() # sub from move_base

		# -- Launch
		rospy.Subscriber("/status_launch", Status_launch, self.callback_statusLaunch)
		self.status_launch = Status_launch()

		# -- Info Move
		# rospy.Subscriber("/navigation_respond", Navigation_respond, self.callback_navigationRespond)
		# self.navigation_respond = Navigation_respond()

		# -----------------------------------------------------------
		rospy.Subscriber("/cancelMission_status", Int16, self.callBack_cancelMission)
		self.cancelMission_status = Int16()
		# --
		self.pub_cancelMission = rospy.Publisher("/cancelMission_control", Int16, queue_size = 4)
		self.cancelMission_control = Int16()
		# --
		self.pub_button = rospy.Publisher("/app_button", App_button, queue_size = 4)
		self.app_button = App_button()
		self.pre_app_setColor = App_color()

		# --
		rospy.Subscriber("/nav350_reflectors", Reflector_array, self.callback_nav350Reflectors) 
		self.nav350_reflectors = Reflector_array()
		self.arrReflector = []

		rospy.Subscriber("/Keyboard_cmd", String, self.callback_Keyboard) # 
		self.keyboard_cmd = String()

		rospy.Subscriber("/raw_vel", TwistWithCovarianceStamped, self.callback_Rawvel) # 
		self.data_rawvel = TwistWithCovarianceStamped()
		self.VEL_MAX = 0.8

		rospy.Subscriber("/cmd_vel", Twist, self.callback_Cmdvel) # 
		self.data_cmdvel = Twist()
  
		rospy.Subscriber("wifi_respond", Wifi_info, self.callback_Wifidata) #
		self.wifi_info = Wifi_info()
		self.list_wifi = CustomDataWifi()
		self.is_recv_wifi_respond = False

		# --
		self.name_agv = ""
		self.ip_agv = ""
		# --
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_byhand = 1
		self.modeRun_auto = 2
		self.modeRun_byhand_tryTarget = 3
		self.modeRun_cancelMission = 5

		# self.app_button.bt_speaker = 1

		self.pathData_SpecialPoint = '/home/stivietnam/catkin_ws/src/app_ros/config/special_point.json'
		self.list_SpecialPoint = []
		self.dataDict_SpecialPoint = {}
		self.isUpdate_SpecialPoint = 0
		self.savetime_SpecialPoint = rospy.Time.now()
		self.savetime_Delete_SpecialPoint = rospy.Time.now()
		self.savetime_Add_SpecialPoint = rospy.Time.now()
		self.list_SpecialPoint_detail = []

		self.loadData_SpecialPoint()

		self.welcomeScreen.tb_SpecialPoint.setColumnCount(2)
		self.welcomeScreen.tb_SpecialPoint.setColumnWidth(0, 70)
		self.welcomeScreen.tb_SpecialPoint.setColumnWidth(1, 351)

		self.welcomeScreen.tb_SpecialPoint.setHorizontalHeaderLabels(['ID', 'Tọa độ'])
		self.welcomeScreen.tb_SpecialPoint.setRowCount(len(self.valueLable.list_SpecialPoint))
		self.welcomeScreen.tb_SpecialPoint.verticalHeader().setVisible(False)

		for e in range(0, len(self.valueLable.list_SpecialPoint)):
			item = QTableWidgetItem(str(self.valueLable.list_SpecialPoint[e]))
			item.setTextAlignment(Qt.AlignCenter)
			self.welcomeScreen.tb_SpecialPoint.setItem(e, 0, item)

			item1 = QTableWidgetItem(str(self.list_SpecialPoint_detail[e]))
			item1.setTextAlignment(Qt.AlignCenter)
			self.welcomeScreen.tb_SpecialPoint.setItem(e, 1, item1)

		# -- display wifi
		self.pre_listwifi = []
		self.flag_update_wifi = 0
		self.list_wifi_ssid = []
		self.list_wifi_mac = []
		self.list_wifi_signal = []

		self.welcomeScreen.tb_listWifi.setColumnCount(3)
		self.welcomeScreen.tb_listWifi.setColumnWidth(0, 200)
		self.welcomeScreen.tb_listWifi.setColumnWidth(1, 200)
		self.welcomeScreen.tb_listWifi.setColumnWidth(2, 121)

		self.welcomeScreen.tb_listWifi.setHorizontalHeaderLabels(['SSID', 'MAC', 'Cường độ'])
		self.welcomeScreen.tb_listWifi.verticalHeader().setVisible(False)

	def anlis_ref2(self):
		self.arrReflector = []

		# max_x = -1000
		# max_y = -1000

		# length = self.nav350_reflectors.num_reflector
		# for i in range(length):
		# 	dis = self.nav350_reflectors.reflectors[i].Polar_Dist/1000.
		# 	ang0 = radians(self.nav350_reflectors.reflectors[i].Polar_Phi/1000.)
		# 	ang = self.limitAngle(ang0)

		# 	p_x, p_y = self.convert_position(dis, ang)

		# if max_x < abs(p_x):
		# 	max_x = abs(p_x)

		# if max_y < abs(p_y):
		# 	max_y = abs(p_y)

		# rate_show = 0.0
		# rate_xy = float(max_x/max_y)
		# if rate_xy > 0.5:
		# 	rate_show = 1.0 # (max_x*1000)/431.
		# else:
		# 	rate_show = 1.0 # (max_y*1000)/811.
		# ----
		
		rate_show = self.welcomeScreen.valueLable.rate_reflector_map
		# print ("----------")
		for i in range(self.nav350_reflectors.num_reflector):
			dis = self.nav350_reflectors.reflectors[i].Polar_Dist/1000.
			ang0 = radians(self.nav350_reflectors.reflectors[i].Polar_Phi/1000. + self.welcomeScreen.valueLable.angleCompare)
			ang = self.limitAngle(ang0)

			p_x, p_y = self.convert_position(dis, ang)
			rp_x = int(p_x/rate_show)
			rp_y = int(p_y/rate_show)
			sh_x = rp_y + 400
			sh_y = rp_x + 200
			
			# print ( str(self.nav350_reflectors.reflectors[i].LocalID) + " | " + str(self.nav350_reflectors.reflectors[i].GlobalID) + " | " + str(round(sh_x, 3)) + " | " + str(round(sh_y, 3)) )
			# --
			reflector = Reflector()
			reflector.x = sh_x
			reflector.y = sh_y
			reflector.localID  = str(self.nav350_reflectors.reflectors[i].LocalID)
			reflector.globalID = str(self.nav350_reflectors.reflectors[i].GlobalID)

			self.arrReflector.append(reflector)

	def callback_nav350Reflectors(self, data):
		self.nav350_reflectors = data
		# print ("NAV NAV")
		# self.anlis_ref1()
		self.anlis_ref2()
		# --

			
	def callback_brakeControl(self, data):
		self.status_brake = data
		
	def callback_driver1(self, data):
		self.driver1_respond = data

	def callback_driver2(self, data):
		self.driver2_respond = data

	def callback_HC(self, data):
		self.HC_info = data

	def callback_Main(self, data):
		self.main_info = data

	def callback_Psu(self, data):
		self.psu_info = data

	def callback_mcu(self, data):
		self.mcu_info = data

	def callback_hcu(self, data):
		self.hcu_info = data

	def callback_OC_board(self, data):
		self.OC_status = data

	def callback_statusPort(self, data):
		self.status_port = data

	def callback_nucInfo(self, data):
		self.nuc_info = data

	def goalControl_callback(self, data):
		self.status_goalControl = data
		
	def callBack_cancelMission(self, data):
		self.cancelMission_status = data

	def callback_nav350(self, data):
		self.nav350_data = data

	def callback_safetyNAV(self, data):
		self.safety_NAV = data

	def callback_robotPose(self, data):
		self.robotPose_nav = data

	def callback_server_cmdRequest(self, data):
		self.server_cmdRequest = data

	def NN_cmdRequest_callback(self, data):
		self.NN_cmdRequest = data	

	def callback_NN_infoRequest(self, data):
		self.NN_infoRequest = data

	def infoAGV_callback(self, data):
		self.NN_infoRespond = data	

	def callback_statusLaunch(self, data):
		self.status_launch = data

	def callback_navigationRespond(self, data):
		self.navigation_respond = data

	def callBack_cancelMission(self, data):
		self.cancelMission_status = data

	def callback_Keyboard(self, data):
		self.keyboard_cmd = data
		if self.keyboard_cmd.data == "":
			# self.flag_reset = 0
			# self.valueLable.ctrl_value = 0
			pass

		elif self.keyboard_cmd.data == "run":
			self.valueLable.ctrl_value = 1
			self.valueLable.num_val += 1

		elif self.keyboard_cmd.data == "reset":
			self.valueLable.ctrl_value = 2
		
		elif self.keyboard_cmd.data == "auto":
			self.valueLable.ctrl_value = 3

	def callback_Rawvel(self, data):
		self.data_rawvel = data
		self.valueLable.raw_vel = str(round(self.data_rawvel.twist.twist.linear.x, 1)) + ' m/s'
		v = int(self.data_rawvel.twist.twist.linear.x * 10 / self.VEL_MAX)
		if 0 < v < 2:   # v < 0.16
			self.valueLable.ratio_v = 5
		elif 2 <= v < 4:  # v < 0.32
			self.valueLable.ratio_v = 4
		elif 4 <= v < 6:  # v < 0.48
			self.valueLable.ratio_v = 3
		elif 6 <= v < 8:  # v < 0.64 
			self.valueLable.ratio_v = 2
		elif 8 <= v: 
			self.valueLable.ratio_v = 1
		else:
			self.valueLable.ratio_v = 0

	def callback_Cmdvel(self, data):
		self.data_cmdvel = data
		self.valueLable.raw_vel = str(round(self.data_cmdvel.linear.x, 1)) + ' m/s'
		v = int(self.data_cmdvel.linear.x * 10 / self.VEL_MAX)
		if 0 < v < 2:   # v < 0.16
			self.valueLable.ratio_v = 5
		elif 2 <= v < 4:  # v < 0.32
			self.valueLable.ratio_v = 4
		elif 4 <= v < 6:  # v < 0.48
			self.valueLable.ratio_v = 3
		elif 6 <= v < 8:  # v < 0.64 
			self.valueLable.ratio_v = 2
		elif 8 <= v: 
			self.valueLable.ratio_v = 1
		else:
			self.valueLable.ratio_v = 0
 
	def callback_Wifidata(self,data):
		self.wifi_info = data
		self.list_wifi = self.wifi_info.datawifi
		self.is_recv_wifi_respond = True

	def getBit_fromInt16(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(16):
			bit_out = value_now%2
			value_now = int(value_now/2)
			if i == pos:
				return bit_out

			if value_now < 1:
				return 0		
		return 0

	def convert_position(self, distance, angle):
		x = 0
		y = 0
		x = distance*cos(angle)
		y = distance*sin(angle)
		# y = distance*cos(angle)
		# x = distance*sin(angle)
		return x, y

	def ping_traffic(self, address):
		try:
			ping = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
			# print(ping)
			vitri = str(ping).find("time")
			time_ping = str(ping)[(vitri+5):(vitri+9)]
			# print (time_ping)
			return str(float(time_ping))
		except Exception:
			return '-1'

	def run_screen(self):
		self.widget.show()
		try:
			# print ("run 1")
			sys.exit(self.app.exec_())
			# print ("run 2")
		except:
			pass
			# print("Exiting 1")
		self.is_exist = 0

	def kill_app(self):
		self.welcomeScreen.out()
		self.is_exist = 0

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def limitAngle(self, angle_in): # - rad
		qua_in = self.euler_to_quaternion(angle_in)
		angle_out = self.quaternion_to_euler(qua_in)
		return angle_out

	def convert_errorAll(self, val):
		switcher={
			0:'AGV Hoạt Động Bình Thường',
			2:'Đã Khởi Tạo Lại Quy Trình Hiện Tại',
			3:'Nút Xóa Lỗi Đã Được Nhấn',
			311:'Mất kết nối với Mạch STI-RTC',			
			361:'Mất Kết Nối Với Mạch STI-CPD', # 

			351:'Mất Kết Nối Với Mạch STI-HC', # 
			352:'Không Giao Tiếp CAN Với Mạch STI-HC', # 

			341:'Mất Kết Nối Với Mạch STI-OC', #
			342:'Mất Cổng USB của USB của Mạch STI-OC', # 
			343:'Không Giao Tiếp CAN Với Mạch STI-OC', # 
			344:'Không Giao Tiếp Với Mạch STI-OC1', # 
			345:'Không Giao Tiếp Với Mạch STI-OC2', # 
			346:'Không Giao Tiếp Với Mạch STI-OC3', # 

			323:'Mạng CAN Không Gửi Được', # 
			321:'Mất Kết Nối Với Mạch STI-Main', # 
			322:'Mất Cổng USB của USB của Mạch STI-Main', # 
			251:'Mất Kết Nối Với Driver1', # 
			252:'Lỗi Động Cơ Số 1', # 
			261:'Mất Kết Nối Với Driver2', # 
			262:'Lỗi Động Cơ Số 2', # 
			231:'Mất Kết Nối Với Cảm Biến Góc', # 
			232:'Mất Cổng USB của Cảm Biến IMU', # 
			221:'Mất Kết Nối Với Cảm Biến NAV350', # 
			181:'LoadCell-Ket Noi', # 
			182:'LoadCell-Dau noi', # 
			183:'LoadCell-USB', # 
			184:'Quá Tải 700kg', # 
			222:'Mất Tọa Độ NAV350', # 
			141:'Lỗi Không Chạm Được Cảm Biến Bàn Nâng', # 
			121:'Trạng Thái Dừng Khẩn - EMG', # 
			122:'AGV Bị Chạm Blsock', #
			272:'Không Phát Hiện Được Đủ Gương', #
			281:'Mất TF Parking', #
			282:'Mất Gói GoalControl', #
			441:'AGV Đã Di Chuyển Hết Điểm', #
			442:'AGV Đang Dừng Để Nhường Đường Cho AGV Khác', # 
			443:'vị Trí Chờ Lệnh Mới của AGV Bị Thay Đổi',
			477:'Không Có Kệ Tại Vị Trí', # 
			411:'Vướng Vật Cản - Di Chuyển Giữa Các Điểm', #
			412:'Vướng Vật Cản - Di Chuyển Vào Vị Trí Kệ', # 
			431:'AGV Không Giao Tiếp Với Phần Mềm Traffic', #
			451:'Điện Áp Của AGV Đang Rất Thấp', # 
			452:'AGV Không Sạc Được Pin', #
			453:'Không Phát Hiện Được Đủ Gương', #
      333:'Không Phát hiện Wifi',     #

		}
		return switcher.get(val, 'UNK')

	def show_job(self, val):
		job_now = 'Không\nXác Định'
		switcher={
			0:'...', #
			1:'Kiểm Tra Lại Nhiệm Vụ', # 
			2:'Thực Hiện Nhiệm Vụ Trước', # 
			3:'Kiểm Tra Trạng Thái Kệ',
			4:'Di Chuyển Ra Khởi Vị Trí', # 
			5:'Di Chuyển Giữa Các Điểm', #
			6:'Di Chuyển Vào Vị Trí Thao Tác', # 
			7:'Thực Hiện Nhiệm Vụ Sau', # 
			8:'Đợi Lệnh Mới', # 
			9:'Đợi Hoàn Thành Lệnh Cũ', # 
			20:'Chế Độ Bằng Tay', # 
			30:'Chế Độ Tự Động', # 
			50:'Kiểm Tra Vị Trí Trả Hàng', # 
		}
		return switcher.get(val, job_now)

	def show_misson(self, val):
		job_now = 'Không\nXác Định'
		switcher={
			0:'...', #
			65:'Nâng Kệ', # 
			1:'Nâng Kệ', # 
			66:'Hạ Kệ', #
			2:'Hạ Kệ', #
			6:'Sạc Pin', # 
			10:'Hạ Kệ\nSạc Pin' # 
		}
		return switcher.get(val, job_now)

	def controlColor(self):
		self.statusColor.lbc_safety_up = self.safety_NAV.data
		# -- HC_info
		self.statusColor.lbc_safety_ahead = self.HC_info.zone_sick_ahead
		self.statusColor.lbc_safety_behind = self.HC_info.zone_sick_behind
		# --
		self.statusColor.lbc_button_clearError = self.main_info.stsButton_reset
		self.statusColor.lbc_button_power = self.main_info.stsButton_power

		self.statusColor.lbc_emg1 = self.mcu_info.status_EMG1
		self.statusColor.lbc_blsock1 = self.mcu_info.status_CS1

		self.statusColor.lbc_emg2 = self.mcu_info.status_EMG2
		self.statusColor.lbc_blsock2 = self.mcu_info.status_CS2

		self.statusColor.lbc_safety_relay = self.mcu_info.status_safetyRelay

		# -- Port
		self.statusColor.lbc_port_rtc    = self.status_port.rtc
		self.statusColor.lbc_port_rs485  = self.status_port.driverall
		self.statusColor.lbc_port_nav350 = self.status_port.nav350

		# -- CAN 
		self.statusColor.lbc_can_psu     = self.psu_info.stsRev_CAN
		self.statusColor.lbc_can_mcu     = self.mcu_info.status_recCAN
		self.statusColor.lbc_can_hcu     = self.hcu_info.status_recCAN
		if self.OC_status.status.data == -2:
			self.statusColor.lbc_can_oc  = 0
		else:
			self.statusColor.lbc_can_oc  = 1

		# --
		self.statusColor.lbc_limitAbove = self.OC_status.sensorUp.data
		self.statusColor.lbc_limitBelow = self.OC_status.sensorDown.data
		self.statusColor.lbc_checkTray = self.OC_status.sensorLift.data

		# -- 
		if self.OC_status.status.data >= 3:
			# print("AGV đã nâng/ hạ kệ xong")
			self.statusColor.lbc_liftUp = 1
			self.statusColor.lbc_liftDown = 1
		elif self.OC_status.status.data == 1:    # Dang ha ke
			# print("AGV đang hạ kệ")
			self.statusColor.lbc_liftDown = 1
			self.statusColor.lbc_liftUp = 0
		elif self.OC_status.status.data == 2:
			# print("AGV đang nâng kệ")
			self.statusColor.lbc_liftDown = 0
			self.statusColor.lbc_liftUp = 1
		elif self.OC_status.status.data == 0:
			# print("Ko có yêu cầu")
			self.statusColor.lbc_liftDown = 0
			self.statusColor.lbc_liftUp = 0					

	def loadData_SpecialPoint(self):
		list_SpecialPoint = []
		# Opening JSON file
		try:
			with open(self.pathData_SpecialPoint, 'r') as openfile:
				# Reading from json file
				json_object = json.load(openfile)

			for p in json_object:
				id = p[6:]
				list_SpecialPoint.append(id)
			
			self.valueLable.id_number = len(list_SpecialPoint)

			if self.valueLable.list_SpecialPoint != list_SpecialPoint:
				self.dataDict_SpecialPoint = json_object

				self.valueLable.isUpdateList_SpecialPoint = True
				self.valueLable.list_SpecialPoint = list_SpecialPoint
				self.isUpdate_SpecialPoint = 1
				self.app_button.list_special_x = []
				self.app_button.list_special_y = []
				self.app_button.list_special_z = []
				print(self.dataDict_SpecialPoint)

			print("List ban dau", self.valueLable.list_SpecialPoint)

			pos_str = ''
			for value in self.dataDict_SpecialPoint.values():
				pos_str = "({x}, {y}, {z})".format(x = value['x'], y = value['y'], z = value['z'])
				self.list_SpecialPoint_detail.append(pos_str)

			print("List chi tiet ban dau", self.list_SpecialPoint_detail)
		except:
			print("Open file json error")

	def updateData_SpecialPoint(self):
		if self.welcomeScreen.flag_delete_SpecialPoint == 1:
			self.welcomeScreen.flag_delete_SpecialPoint = 0
			selecId = self.welcomeScreen.cb_listSpecialPoint.currentText()
			# print("Hang duoc chon la:", selecId)
			if selecId == '':
				self.statusColor.lbc_deletePose = -1
				self.savetimeDeletepose = rospy.Time.now()
				self.welcomeScreen.bt_DeleteSpecialPoint.setStyleSheet("background-color: red;")
				return
			
			else:
				self.welcomeScreen.bt_DeleteSpecialPoint.setStyleSheet("background-color: white;")
				del self.dataDict_SpecialPoint['point_' + str(selecId)]
				self.valueLable.list_SpecialPoint.remove(selecId)
				# self.valueLable.id_number = self.valueLable.id_number - 1
				row_id = 0
				for row in range(self.welcomeScreen.tb_SpecialPoint.rowCount()):
					item = self.welcomeScreen.tb_SpecialPoint.item(row, 0)
					if item and item.text() == str(selecId):
						print("Hang duoc chon la:", row)
						row_id = row
			
				self.welcomeScreen.tb_SpecialPoint.removeRow(row_id)

				self.valueLable.isUpdateList_SpecialPoint = True
				self.isUpdate_SpecialPoint = 1
				self.app_button.list_special_x = []
				self.app_button.list_special_y = []
				self.app_button.list_special_z = []
				# print(self.dataDicSetPose)

				try:
					with open(self.pathData_SpecialPoint, "w") as outfile: 
						json.dump(self.dataDict_SpecialPoint, outfile)
				except:
					print("Convert dict to json Error")

		if self.welcomeScreen.flag_add_SpecialPoint == 1:
			self.welcomeScreen.flag_add_SpecialPoint = 0

			addID = self.welcomeScreen.valueLable.lbv_tryTarget_id
			if addID == 0:
				print("ID bang 0")
				self.welcomeScreen.bt_SaveSpecialPoint.setStyleSheet("background-color: red;")
				return
			
			else:
				# check xem id được thêm có trùng với các id khác ko ?
				for id in self.valueLable.list_SpecialPoint:
					if addID == int(id):
						print("ID bij trungf list")
						self.welcomeScreen.bt_SaveSpecialPoint.setStyleSheet("background-color: red;")
						return
				# self.valueLable.id_number += 1

				self.welcomeScreen.bt_SaveSpecialPoint.setStyleSheet("background-color: white;")	
				self.valueLable.list_SpecialPoint.append(str(addID))
				dx = round(self.welcomeScreen.valueLable.lbv_tryTarget_x, 3)
				dy = round(self.welcomeScreen.valueLable.lbv_tryTarget_y, 3)
				dz = round(self.welcomeScreen.valueLable.lbv_tryTarget_r, 2)

				self.dataDict_SpecialPoint['point_'+ str(addID)] = {'x': dx, 'y': dy, 'z': dz}
				# print(self.dataDicSetPose)

				rowCount = self.welcomeScreen.tb_SpecialPoint.rowCount()
				pos_str = "({x}, {y}, {z})".format(x = dx, y = dy, z = dz)
				self.welcomeScreen.tb_SpecialPoint.insertRow(rowCount)

				item = QTableWidgetItem(str(addID))
				item.setTextAlignment(Qt.AlignCenter)
				self.welcomeScreen.tb_SpecialPoint.setItem(rowCount, 0, item)

				item1 = QTableWidgetItem(pos_str)
				item1.setTextAlignment(Qt.AlignCenter)
				self.welcomeScreen.tb_SpecialPoint.setItem(rowCount, 1, item1)

				try:
					with open(self.pathData_SpecialPoint, "w") as outfile: 
						json.dump(self.dataDict_SpecialPoint, outfile)
				except:
					print("Convert dict to json Error")

				self.valueLable.isUpdateList_SpecialPoint = True
				self.isUpdate_SpecialPoint = 1
				self.app_button.list_special_x = []
				self.app_button.list_special_y = []
				self.app_button.list_special_z = []

	def control_SpecialPoint(self):
		if self.isUpdate_SpecialPoint == 1:
			self.isUpdate_SpecialPoint = 0

			if not self.dataDict_SpecialPoint:
				self.app_button.list_special_x = []
				self.app_button.list_special_y = []
				self.app_button.list_special_z = []
			else:
				# pos_str = ''
				for value in self.dataDict_SpecialPoint.values():
					self.app_button.list_special_x.append(value['x'])
					self.app_button.list_special_y.append(value['y'])
					
					c = value['z']
					if c > 180:
						c = 360 - c
						z = (c/180)*math.pi*(-1)
					else:
						z = (c/180)*math.pi
					
					self.app_button.list_special_z.append(round(z, 3))

	def updateData_Wifi(self):
		if self.is_recv_wifi_respond == 1:
			self.valueLable.lbv_ap_mac = self.wifi_info.bssid
			self.valueLable.lbv_ap_signal = self.wifi_info.ap_signal
			
			if self.pre_listwifi != self.list_wifi:
				self.pre_listwifi = self.list_wifi

				# -- update number row
				self.welcomeScreen.tb_listWifi.setRowCount(len(self.list_wifi))

				# -- Split parent list to 3 child list
				for i in range(0, len(self.list_wifi)):
					self.list_wifi_ssid.append(self.list_wifi[i].ssid)
					self.list_wifi_mac.append(self.list_wifi[i].mac_address)
					self.list_wifi_signal.append(self.list_wifi[i].signal_strength)


				# - update on table wifi
				for e in range(0, len(self.list_wifi)):
					item = QTableWidgetItem(str(self.list_wifi_ssid[e]))
					item.setTextAlignment(Qt.AlignCenter)
					self.welcomeScreen.tb_listWifi.setItem(e, 0, item)

					item1 = QTableWidgetItem(str(self.list_wifi_mac[e]))
					item1.setTextAlignment(Qt.AlignCenter)
					self.welcomeScreen.tb_listWifi.setItem(e, 1, item1)

					item2 = QTableWidgetItem(str(self.list_wifi_signal[e]))
					item2.setTextAlignment(Qt.AlignCenter)
					self.welcomeScreen.tb_listWifi.setItem(e, 2, item2)

				# -- reset child list
				self.list_wifi_mac = []
				self.list_wifi_ssid = []
				self.list_wifi_signal = []

	def controlAll(self):

		self.valueLable.lbv_ipWifi = self.nuc_info.nuc_ipWifi
		self.valueLable.lbv_ipEthernet = self.nuc_info.nuc_ipEthernet
		self.valueLable.lbv_mac = self.nuc_info.nuc_mac
		self.valueLable.lbv_namePc = self.nuc_info.nuc_name

		self.valueLable.lbv_cpu_usage = str(self.nuc_info.cpu_usage) + ' %'
		self.valueLable.lbv_cpu_temp = str(self.nuc_info.cpu_temp) +" "+ str(chr(176))+"C"
		self.valueLable.lbv_ram = str(self.nuc_info.ram_usage) + "/ " + str(self.nuc_info.ram_total)

		self.valueLable.lbv_pingServer = self.nuc_info.ping_server

		self.valueLable.lbv_wifiQuality = str(self.nuc_info.wifi_quality) + " %"
		# -
		if self.nuc_info.wifi_quality == 0 or self.nuc_info.wifi_signal == 0:
			self.valueLable.lbv_wifiSignal = "Mất kết nối"
		else:
			if self.nuc_info.wifi_signal >= -60:
				self.valueLable.lbv_wifiSignal = "Tốt"
			elif -60 > self.nuc_info.wifi_signal >= -70:
				self.valueLable.lbv_wifiSignal = "Trung Bình"
			elif self.nuc_info.wifi_signal < -70:
				self.valueLable.lbv_wifiSignal = "Kém"

		self.valueLable.lbv_runtime = self.nuc_info.uptime

		# -- Mode show
		if (self.NN_infoRespond.mode == 0):   # - launch
			self.valueLable.modeRuning = self.modeRun_launch

		elif (self.NN_infoRespond.mode == 1): # -- md_by_hand
			self.valueLable.modeRuning = self.modeRun_byhand

		elif (self.NN_infoRespond.mode == 2): # -- md_auto
			self.valueLable.modeRuning = self.modeRun_auto

		# -- Battery
		if (self.main_info.charge_current > 0.1):
			self.statusColor.lbc_battery = 4
		else:
			if (self.main_info.voltages < 23.5):
				self.statusColor.lbc_battery = 3
			elif (self.main_info.voltages >= 23.5 and self.main_info.voltages < 24.5):
				self.statusColor.lbc_battery = 2
			else:
				self.statusColor.lbc_battery = 1

		bat = round(self.main_info.voltages, 1)
		if bat > 25.5:
			bat = 25.5
			
		#self.valueLable.lbv_battery = "  " + str(self.NN_infoRespond.battery/10) + " V"
		self.valueLable.lbv_battery = "  " + str(bat) + " V"
		# -- status AGV
		self.statusColor.cb_status = self.NN_infoRespond.status
		lg_err = len(self.NN_infoRespond.listError)
		self.valueLable.listError = []
		
		if (lg_err == 0):
			self.valueLable.listError.append( self.convert_errorAll(0) )
		else:
			length = len(self.valueLable.list_logError)
			if length > 15:
				self.valueLable.list_logError = []
			# -
			for i in range(lg_err):
				self.valueLable.listError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )
				# -
				if self.NN_infoRespond.listError[i] < 400:
					self.valueLable.list_logError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )
		# -
		self.valueLable.lbv_name_agv = self.NN_infoRequest.name_agv                   # Archie
		self.valueLable.lbv_numbeReflector = str(self.nav350_data.number_reflectors)
		self.valueLable.lbv_reflectorDetect = str(self.nav350_reflectors.num_reflector)
		ang0 = self.nav350_data.phi - radians(self.welcomeScreen.valueLable.angleCompare)
		ang = self.limitAngle(ang0)
		self.valueLable.nav_angle = degrees(ang)

		# -- Ping
		# deltaTime_ping = (time.time() - self.pre_timePing)%60
		# if (deltaTime_ping > 1.0):
		# 	self.pre_timePing = time.time()
		# 	self.valueLable.lbv_pingServer = self.nuc_info.ping_server
		# 	# -
		# 	self.valueLable.lbv_qualityWifi = self.nuc_info.
		# 	self.valueLable.lbv_QualityWifi

		# -- 
		self.valueLable.lbv_coordinates_x = str(round(self.robotPose_nav.pose.position.x, 3))
		self.valueLable.lbv_coordinates_y = str(round(self.robotPose_nav.pose.position.y, 3))
		angle = self.quaternion_to_euler(self.robotPose_nav.pose.orientation)
		
		if angle < 0:
			angle_robot = 2*pi + angle
		else:
			angle_robot = angle
		self.valueLable.lbv_coordinates_r = str( round( degrees(angle_robot), 3) )
		# --
		self.valueLable.lbv_route_target = str(self.NN_cmdRequest.target_id) + "\n" + str(round(self.NN_cmdRequest.target_x, 3)) + "\n" + str(round(self.NN_cmdRequest.target_y, 3)) + "\n" + str(round(degrees(self.NN_cmdRequest.target_z), 2)) + "\n" + str(round(self.NN_cmdRequest.offset, 3))
		# # -
		if len(self.NN_cmdRequest.list_id) >= 5:
			self.valueLable.lbv_route_point0 = str(self.NN_cmdRequest.list_id[0]) + "\n" + str(round(self.NN_cmdRequest.list_x[0], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[0], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[0])
			self.valueLable.lbv_route_point1 = str(self.NN_cmdRequest.list_id[1]) + "\n" + str(round(self.NN_cmdRequest.list_x[1], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[1], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[1])
			self.valueLable.lbv_route_point2 = str(self.NN_cmdRequest.list_id[2]) + "\n" + str(round(self.NN_cmdRequest.list_x[2], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[2], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[2])
			self.valueLable.lbv_route_point3 = str(self.NN_cmdRequest.list_id[3]) + "\n" + str(round(self.NN_cmdRequest.list_x[3], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[3], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[3])
			self.valueLable.lbv_route_point4 = str(self.NN_cmdRequest.list_id[4]) + "\n" + str(round(self.NN_cmdRequest.list_x[4], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[4], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[4])
		
		self.valueLable.lbv_route_job1 = str(self.NN_cmdRequest.before_mission)
		self.valueLable.lbv_route_job2 = str(self.NN_cmdRequest.after_mission)

		self.valueLable.lbv_route_job1_mean = self.show_misson(self.NN_cmdRequest.before_mission)
		self.valueLable.lbv_route_job2_mean = self.show_misson(self.NN_cmdRequest.after_mission)

		self.valueLable.lbv_route_message = self.NN_cmdRequest.command
		self.valueLable.lbv_jobRuning = self.show_job(self.NN_infoRespond.process)

		if self.NN_infoRespond.process == 10:
			self.statusColor.lbc_tryTarget_status = 0
		elif self.NN_infoRespond.process == 11:
			self.statusColor.lbc_tryTarget_status = 1
		elif self.NN_infoRespond.process == 12:
			self.statusColor.lbc_tryTarget_status = 2
		elif self.NN_infoRespond.process == 13:
			self.statusColor.lbc_tryTarget_status = 3
		elif self.NN_infoRespond.process == 14:
			self.statusColor.lbc_tryTarget_status = 4
		elif self.NN_infoRespond.process == 15:
			self.welcomeScreen.bt_resetFrameWork.setStyleSheet("background-color: white;")
		# -- 
		self.valueLable.lbv_goalFollow_id = str(self.status_goalControl.ID_follow)

		# -- Launch
		self.valueLable.percentLaunch = self.status_launch.persent
		self.valueLable.lbv_launhing = self.status_launch.notification
		self.valueLable.lbv_numberLaunch = self.status_launch.position

		# -- driver
		self.valueLable.lbv_velLeft = str(self.driver1_respond.speed)
		self.valueLable.lbv_velRight = str(self.driver2_respond.speed)

		self.valueLable.lbv_notification_driver1 = self.driver1_respond.message_error
		self.valueLable.lbv_notification_driver2 = self.driver2_respond.message_error

		# -- Temp
		self.valueLable.lbv_temp = str(self.psu_info.temperture)

		# - Voltage raw
		self.valueLable.lbv_voltage = str(round(self.psu_info.voltages, 1))

	def readButton(self):
		# -- 
		self.app_button.bt_cancelMission = self.welcomeScreen.statusButton.bt_cancelMission
		self.app_button.bt_passAuto 	 = self.welcomeScreen.statusButton.bt_passAuto
		self.app_button.bt_passHand 	 = self.welcomeScreen.statusButton.bt_passHand
		self.app_button.bt_setting 		 = self.welcomeScreen.statusButton.bt_setting
		self.app_button.bt_clearError 	 = self.welcomeScreen.statusButton.bt_clearError
		# --
		self.app_button.bt_forwards 	  = self.welcomeScreen.statusButton.bt_forwards
		self.app_button.bt_backwards	  = self.welcomeScreen.statusButton.bt_backwards
		self.app_button.bt_rotation_left  = self.welcomeScreen.statusButton.bt_rotation_left
		self.app_button.bt_rotation_right = self.welcomeScreen.statusButton.bt_rotation_right
		self.app_button.bt_stop 		  = self.welcomeScreen.statusButton.bt_stop
		# --
		self.app_button.bt_chg_on	= self.welcomeScreen.statusButton.bt_chg_on
		self.app_button.bt_chg_off	= self.welcomeScreen.statusButton.bt_chg_off

		self.app_button.bt_spk_on  = self.welcomeScreen.statusButton.bt_spk_on
		self.app_button.bt_spk_off  = self.welcomeScreen.statusButton.bt_spk_off

		self.app_button.bt_disableBrake	= self.welcomeScreen.statusButton.bt_disableBrake

		# -- 
		self.app_button.bt_lift	 = self.welcomeScreen.statusButton.bt_lift
		# self.app_button.bt_lift_down = self.welcomeScreen.statusButton.bt_lift_down
		# self.app_button.bt_lift_reset = self.welcomeScreen.statusButton.bt_lift_reset
		# -
		self.app_button.vs_speed = self.welcomeScreen.statusButton.vs_speed
		self.app_button.bt_resetFrameWork = self.welcomeScreen.statusButton.bt_resetFrameWork

		# -
		self.app_button.bt_tryTarget_start = self.welcomeScreen.statusButton.bt_tryTarget_start
		self.app_button.bt_tryTarget_stop = self.welcomeScreen.statusButton.bt_tryTarget_stop
		self.app_button.bt_tryTarget_reset = self.welcomeScreen.statusButton.bt_tryTarget_reset
		self.app_button.ck_tryTarget_safety = self.welcomeScreen.statusButton.ck_tryTarget_safety
		# -
		self.app_button.tryTarget_x = self.welcomeScreen.valueLable.lbv_tryTarget_x
		self.app_button.tryTarget_y = self.welcomeScreen.valueLable.lbv_tryTarget_y
		self.app_button.tryTarget_r = self.welcomeScreen.valueLable.lbv_tryTarget_r
		self.app_button.tryTarget_d = self.welcomeScreen.valueLable.lbv_tryTarget_d
		
		# -
		self.app_button.bt_remote = self.welcomeScreen.statusButton.bt_remote

		# - 
		self.app_button.soundtype = self.welcomeScreen.statusButton.soundtype
		self.app_button.ledtype = self.welcomeScreen.statusButton.ledtype

		# -
		self.app_button.bt_wifi = self.welcomeScreen.statusButton.bt_wifi


	def run(self):
		# -- 
		# self.valueLable.lbv_ip = self.get_ipAuto(self.name_card)
		# self.valueLable.lbv_mac = self.get_MAC(self.name_card)
		# self.valueLable.lbv_namePc = self.get_hostname()

		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
			# -
			self.controlAll()
			self.controlColor()
			# -
			self.updateData_SpecialPoint()
			self.control_SpecialPoint()
			# - 
			self.updateData_Wifi()
			# --
			self.readButton()
			self.pub_button.publish(self.app_button)

			if self.cancelMission_status.data == 1:
				self.welcomeScreen.statusButton.bt_cancelMission = 0
				self.cancelMission_control.data = 0

			if self.welcomeScreen.statusButton.bt_cancelMission == 1:
				self.cancelMission_control.data = 1

			self.pub_cancelMission.publish(self.cancelMission_control)

			# ----------------------
			self.welcomeScreen.valueLable = self.valueLable
			self.welcomeScreen.statusColor = self.statusColor
			# -- 
			self.welcomeScreen.robotPoseNow = self.robotPose_nav.pose
			# -
			self.welcomeScreen.valueLable.arrReflector = self.arrReflector
			self.rate.sleep()

		self.is_exist = 0
		self.kill_app()

		print('Thread #%s stopped' % self.threadID)


class ServiceExit(Exception):
	"""
	Custom exception which is used to trigger the clean exit
	of all running threads and the main program.
	"""
	pass
 
def service_shutdown(signum, frame):
	print('Caught signal %d' % signum)
	raise ServiceExit

def main():
	# Register the signal handlers
	signal.signal(signal.SIGTERM, service_shutdown)
	signal.signal(signal.SIGINT, service_shutdown)

	print('Starting main program')

	# Start the job threads
	try:
		thread1 = Program(1)
		thread1.start()

		# Keep the main thread running, otherwise signals are ignored.
		thread1.run_screen()
		thread1.is_exist = 0

	except ServiceExit:
		thread1.shutdown_flag.set()
		thread1.join()
		print('Exiting main program')
 
if __name__ == '__main__':
	main()
