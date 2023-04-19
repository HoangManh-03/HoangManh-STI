#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Author: Hoang Van Quang - BEE
#Date: 31/10/2020
import sys
import serial
import struct
import time
from decimal import *
import math
import rospy
import roslib
import threading
import signal
# get ip
import os                                                                                                                                                           
import re                                                                                                                                                           

from datetime import datetime

from sti_msgs.msg import *

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from message_pkg.msg import *

# How to cancel mission 
# This node to process entry password.
# If password right -> pub topic to Node Control and exit Password pape.
# If Node Control received -> pub confirm to Node Systhtic and clear flag cancel mission.

class synthetic():
	def __init__(self):
		rospy.init_node('M341_stiSynthetic', anonymous=False)
		self.rate = rospy.Rate(40)
		# launch
# -- add new
	# -- cancel mission
		self.pub_cancelMission = rospy.Publisher("/cancelMission_control", Int16 , queue_size=1)
		self.cancelMission_control = Int16()

		rospy.Subscriber("/cancelMission_status", Int16, self.sts_cancelMissionCallback)
		self.cancelMission_status = Int16()

		rospy.Subscriber("/setpose_status", Setpose_status, self.setposeCallback)
		self.setpose_status = Setpose_status()

		rospy.Subscriber('/status_launch', Status_launch, self.callback_statusLaunch)
		self.status_launch = Status_launch()

		rospy.Subscriber("/safety_zone", Zone_lidar_2head, self.callback_safetyZone)
		self.safety_zone = Zone_lidar_2head()

		rospy.Subscriber('/status_reconnect', Status_reconnect, self.callBack_reconnect)
		self.status_reconnect = Status_reconnect()

		rospy.Subscriber("/status_port", Status_port, self.callback_statusPort)
		self.status_port = Status_port()

	  # for info general
		# status + number error
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.NN_infoCallback)
		self.NN_infoRespond = NN_infoRespond()

		self.pub_infoGeneral = rospy.Publisher("/HMI_infoGeneral", HMI_infoGeneral , queue_size=1)
		self.infoGeneral = HMI_infoGeneral()

		self.pub_papeLaunch = rospy.Publisher("/HMI_papeLaunch", HMI_papeLaunch , queue_size=1)
		self.HMI_papeLaunch = HMI_papeLaunch()

		self.pub_papeNN = rospy.Publisher("/HMI_papeDetailNN", HMI_papeDetailNN , queue_size=1)
		self.HMI_papeDetailNN = HMI_papeDetailNN()

		self.pub_papeFL = rospy.Publisher("/HMI_papeDetailFL", HMI_papeDetailFL , queue_size=1)
		self.HMI_papeDetailFL = HMI_papeDetailFL()

		self.pub_papeMessage = rospy.Publisher("/HMI_papeMessage", HMI_papeMessage , queue_size=1)
		self.HMI_papeMessage = HMI_papeMessage()

		self.pub_papeByHand = rospy.Publisher("/HMI_papeByHand", HMI_papeByHand , queue_size=1)
		self.HMI_papeByHand = HMI_papeByHand()

		self.pub_papeAuto = rospy.Publisher("/HMI_papeAuto", HMI_papeAuto , queue_size=1)
		self.HMI_papeAuto = HMI_papeAuto()
# -- add new
		self.pub_papePassword = rospy.Publisher("/HMI_papePassword", HMI_papePassword , queue_size=1)
		self.HMI_papePassword = HMI_papePassword()
		# # card magnetic
		rospy.Subscriber("/POWER_info", POWER_info, self.mainInfoCallback)
		self.mainInfo = POWER_info()		

		# rospy.Subscriber("/status_base", Status_base, self.mc_Callback)
		# self.mc_status = Status_base()
	# 	# Goal
		rospy.Subscriber("/goal_control", Goal_control, self.goal_callback)
		self.goal_control = Goal_control()
	  	# cmd_vel:
		rospy.Subscriber("/cmd_vel", Twist, self.cmd_velCallback)
		self.cmd_vel = Twist()

		# Task + comunicate Sys
		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.NN_cmdCallback)
		self.NN_cmdRequest = NN_cmdRequest()

		rospy.Subscriber("/HMI_allButton", HMI_allButton, self.HMI_allButton_callback)
		self.HMI_allButton = HMI_allButton()
# -- add new
		rospy.Subscriber("/HMI_buttonPassword", HMI_buttonPassword, self.HMI_buttonPassword_callback)
		self.HMI_buttonPassword = HMI_buttonPassword()
		self.old_HMI_buttonPassword = HMI_buttonPassword()


		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoNN_callback)
		self.NN_infoRespond = NN_infoRespond()

		self.nameAgv = "AMR-L300-M34"
		self.ip = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen('ip addr show wlp3s0').read()).groups()[0]

		self.max_pin = 25.5 # Voltages
		self.min_pin = 24.0   # Voltages
		self.rate_vol = (self.max_pin - self.min_pin)/100. # 1%/vol	

		self.currentPape = 0
		self.pape_launch = 0
		self.pape_auto = 1
		self.pape_detailFL = 2
		self.pape_detailNN = 3
		self.pape_byHand = 4
		self.pape_message = 5
		self.pape_password = 6
# -- add new
		self.is_displayDetail = 0  # 1: yes
		self.is_displayMessage = 0
		self.is_displayPassword = 0 

		self.total_launch = 9 + 11 + 3 + 4 + 3
		
		self.is_main82 = 0
		self.is_newDataMain = 0
		self.pre_timeMain = int(round(time.time() * 1000))
		self.timeMain = 0

		self.is_newDataMC = 0
		self.pre_timeMC = int(round(time.time() * 1000))
		self.timeMC = 0
		
# -- add new
		self.password = [2,0,2,1] # [1,5,0,4]
		self.passwordEntry = [10,10,10,10]
		self.passwordShow = "----"
		self.flag_cancelMission = 0

	def HMI_buttonPassword_callback(self, dat):
		self.HMI_buttonPassword = dat

	def sts_cancelMissionCallback(self, dat):
		self.cancelMission_status = dat

	def setposeCallback(self, dat):
		self.setpose_status = dat

	def callback_safetyZone(self, dat):
		self.safety_zone = dat

	def callback_statusPort(self, dat):
		self.status_port = dat

	def HMI_allButton_callback(self, dat):
		self.HMI_allButton = dat

	def callBack_reconnect(self, dat):
		self.status_reconnect = dat

	def callback_statusLaunch(self, dat):
		self.status_launch = dat

	def NN_infoCallback(self, dat):
			self.NN_infoRespond = dat

	def cmd_velCallback(self, dat):
			self.cmd_vel = dat

	def NN_cmdCallback(self, dat):
		self.NN_cmdRequest = dat

	def goal_callback(self, dat):
		self.goal_control = dat

	def mainInfoCallback(self, dat):
		self.mainInfo = dat
		self.is_main82 = 1
		self.is_newDataMain = 1

	# def mc_Callback(self, dat):
	# 	self.mc_status = dat
	# 	self.is_newDataMC = 1

	def infoNN_callback(self, dat):
		self.NN_infoRespond = dat

	def authenticationPassword(self): # authentication: xac thuc
		count = 0
		data = ""
		for i in range(0,4):
			if self.passwordEntry[i] < 10:
				count += 1
				data += "*"
			else:
				data += "-"

		self.HMI_papePassword.t_password = data

		# -- 0
		if self.old_HMI_buttonPassword.b_n0 != self.HMI_buttonPassword.b_n0:
			self.old_HMI_buttonPassword.b_n0 = self.HMI_buttonPassword.b_n0
			if self.HMI_buttonPassword.b_n0 == 1:
				if count < 4:
					self.passwordEntry[count] = 0
					count += 1
		# # -- 1
		if self.old_HMI_buttonPassword.b_n1 != self.HMI_buttonPassword.b_n1:
			self.old_HMI_buttonPassword.b_n1 = self.HMI_buttonPassword.b_n1		
			if self.HMI_buttonPassword.b_n1 == 1:
				if count < 4:
					self.passwordEntry[count] = 1
					count += 1
		# -- 2			
		if self.old_HMI_buttonPassword.b_n2 != self.HMI_buttonPassword.b_n2:
			self.old_HMI_buttonPassword.b_n2 = self.HMI_buttonPassword.b_n2			
			if self.HMI_buttonPassword.b_n2 == 1:
				if count < 4:
					self.passwordEntry[count] = 2
					count += 1
		# -- 3				
		if self.old_HMI_buttonPassword.b_n3 != self.HMI_buttonPassword.b_n3:
			self.old_HMI_buttonPassword.b_n3 = self.HMI_buttonPassword.b_n3		
			if self.HMI_buttonPassword.b_n3 == 1:
				if count < 4:
					self.passwordEntry[count] = 3
					count += 1
		# -- 4				
		if self.old_HMI_buttonPassword.b_n4 != self.HMI_buttonPassword.b_n4:
			self.old_HMI_buttonPassword.b_n4 = self.HMI_buttonPassword.b_n4		
			if self.HMI_buttonPassword.b_n4 == 1:
				if count < 4:
					self.passwordEntry[count] = 4
					count += 1
		# -- 5				
		if self.old_HMI_buttonPassword.b_n5 != self.HMI_buttonPassword.b_n5:
			self.old_HMI_buttonPassword.b_n5 = self.HMI_buttonPassword.b_n5		
			if self.HMI_buttonPassword.b_n5 == 1:
				if count < 4:
					self.passwordEntry[count] = 5
					count += 1
		# -- 6				
		if self.old_HMI_buttonPassword.b_n6 != self.HMI_buttonPassword.b_n6:
			self.old_HMI_buttonPassword.b_n6 = self.HMI_buttonPassword.b_n6		
			if self.HMI_buttonPassword.b_n6 == 1:
				if count < 4:
					self.passwordEntry[count] = 6
					count += 1
		# -- 7				
		if self.old_HMI_buttonPassword.b_n7 != self.HMI_buttonPassword.b_n7:
			self.old_HMI_buttonPassword.b_n7 = self.HMI_buttonPassword.b_n7		
			if self.HMI_buttonPassword.b_n7 == 1:
				if count < 4:
					self.passwordEntry[count] = 7
					count += 1
		# -- 8				
		if self.old_HMI_buttonPassword.b_n8 != self.HMI_buttonPassword.b_n8:
			self.old_HMI_buttonPassword.b_n8 = self.HMI_buttonPassword.b_n8		
			if self.HMI_buttonPassword.b_n8 == 1:
				if count < 4:
					self.passwordEntry[count] = 8
					count += 1
		# -- 9				
		if self.old_HMI_buttonPassword.b_n9 != self.HMI_buttonPassword.b_n9:
			self.old_HMI_buttonPassword.b_n9 = self.HMI_buttonPassword.b_n9		
			if self.HMI_buttonPassword.b_n9 == 1:
				if count < 4:
					self.passwordEntry[count] = 9
					count += 1

		# -- delete single.
		if self.old_HMI_buttonPassword.b_delete != self.HMI_buttonPassword.b_delete:
			self.old_HMI_buttonPassword.b_delete = self.HMI_buttonPassword.b_delete		
			if self.HMI_buttonPassword.b_delete == 1:
				if count > 0:
					count -= 1
					self.passwordEntry[count] = 10
				
		# -- delete all.
		if self.HMI_buttonPassword.b_clear == 1:
			self.passwordEntry = [10,10,10,10]

		xx = 0
		for i in range(0, 4):
			if self.passwordEntry[i] == self.password[i]:
				xx += 1

		if xx == 4:
			self.HMI_papePassword.t_color = 1
			if self.HMI_buttonPassword.b_confirm == 1:
				self.flag_cancelMission = 1	
				self.is_displayPassword = 0	
				self.passwordEntry = [10, 10, 10, 10]		
		else:
			self.HMI_papePassword.t_color = 2

		if self.HMI_buttonPassword.b_cancel == 1:
			self.is_displayPassword = 0
			self.passwordEntry = [10, 10, 10, 10]

		self.cancelMission_control.data = self.flag_cancelMission

		if self.flag_cancelMission == 1: # xac nhan node control da nhan lenh huy nhiem vu.
			if self.cancelMission_status.data == 1:
				self.flag_cancelMission = 0

		self.pub_cancelMission.publish(self.cancelMission_control)
		self.pub_papePassword.publish(self.HMI_papePassword)

	def picture_error(self, x):
		switcher={
		  # NN
			0:27, # ALL RIGHT
			111:9, # Va vào Blsock.
			121:13, # Ấn EMG.		231	121
			131:23, # Ra khỏi đường từ.		239	131
			141:17, # Bàn nâng.		225	141
			211:10, # Camera:	Mất kết nối vật lý.		211
			212:10, #	Camera: Không giao tiếp truyền thông.		212
			221:16, # Lidar.	Phía trước.		221
			222:16, #	Lidar. Phía sau.		222
			223:16, #	Lidar. Cả 2.		223
			231:15, # IMU	Không giao tiếp truyền thông.		231
			241:24, # PS2	Không giao tiếp truyền thông.		241

			251:12, # DRIVER 1 (trái)	NN – Natual Navigatiroson		251
			261:12, # DRIVER 2 (phải)	NN – Natual Navigation.		261
			271:18, #	Mangnetic line:	NN - RS232 – PC lỗi (phía trước).
			272:18, #	Mangnetic line:	NN - RS232 – PC lỗi (phía sau).
			311:21, # Mạch MC - NN.	Mất kết nối vật lý Serial.		311
			312:21, #	Mạch MC - NN. Không giao tiếp truyền thông Serial.		312			
			321:19, # Mạch Main - NN:	Mất kết nối vật lý Serial.		321
			322:19, #	Mạch Main - NN: Không giao tiếp truyền thông Serial.		322
			331:26, # Mạch SC: Mất kết nối vật lý Serial.		331
			332:26, #	Mạch SC: Không giao tiếp truyền thông Serial.		332			
			341:22,	# Mạch OC: Mất kết nối vật lý Serial.
			342:22,	#	Mạch OC: Không giao tiếp truyền thông Serial.
			351:14,	# Mạch HC: Mất kết nối vật lý Serial.
			352:14,	#	Mạch HC: Không giao tiếp truyền thông Serial.
			411:30, # Di chuyển NN (không vạch từ) -  Không thể đến được đích.			411
			421:28, # Có vật cản			421
			431:29, # Mất kết nối server			431
			441:48, # Không thể thấy Tag.
			451:49, # Điện Áp Thấp	
			461:62, # Có vật cản khi vào kệ.
			471:57  # không có kệ hoặc lệch kệ khi nâng.
		}
		return switcher.get(x, 38)

	def picture_methor(self, x):
		switcher={
		  # NN
			0:0,   # ALL RIGHT
			111:1, # Va vào Blsock.
			121:3, # Ấn EMG.		231	121
			131:4, # Ra khỏi đường từ.		239	131
			141:2, # Bàn nâng.		225	141
			211:2, # Camera:	Mất kết nối vật lý.		211
			212:5, #	Camera: Không giao tiếp truyền thông.		212
			221:5, # Lidar.	Phía trước.		221
			222:5, #	Lidar. Phía sau.		222
			223:5, #	Lidar. Cả 2.		223
			231:5, # IMU	Không giao tiếp truyền thông.		231
			241:5, # PS2	Không giao tiếp truyền thông.		241
			251:5, # DRIVER 1 (trái)	NN – Natual Navigation		251
			261:5, # DRIVER 2 (phải)	NN – Natual Navigation.		261
			271:5, #	Mangnetic line:	NN - RS232 – PC lỗi (phía trước).
			272:5, #	Mangnetic line:	NN - RS232 – PC lỗi (phía sau).
			311:2, # Mạch MC - NN.	Mất kết nối vật lý Serial.		311
			312:5, #	Mạch MC - NN. Không giao tiếp truyền thông Serial.		312			
			321:2, # Mạch Main - NN:	Mất kết nối vật lý Serial.		321
			322:5, #	Mạch Main - NN: Không giao tiếp truyền thông Serial.		322
			331:2, # Mạch SC: Mất kết nối vật lý Serial.		331
			332:5, #	Mạch SC: Không giao tiếp truyền thông Serial.		332			
			341:2, # Mạch OC: Mất kết nối vật lý Serial.
			342:5, #	Mạch OC: Không giao tiếp truyền thông Serial.
			351:2, # Mạch HC: Mất kết nối vật lý Serial.
			352:5, #	Mạch HC: Không giao tiếp truyền thông Serial.
			411:59, # Di chuyển NN (không vạch từ) -  Không thể đến được đích.			411
			421:7, # Có vật cản			421
			431:8, # Mất kết nối server			431		
			441:50,# Không thể thấy Tag.
			451:51,# Điện Áp Thấp
			461:63, # vat can vao ke.
			471:58  # không có kệ hoặc lệch kệ khi nâng.
		}
		return switcher.get(x, 11)	


	def synthetic_launch(self):
		# sua cuoi cung.
		self.HMI_papeLaunch.j_waitLaunch = int(self.status_launch.persent)
		self.HMI_papeLaunch.n_launch = self.status_launch.position
		self.HMI_papeLaunch.t_message = self.status_launch.notification

		self.HMI_papeLaunch.t_colorLD1 = self.status_port.lms100
		self.HMI_papeLaunch.t_colorLD2 = self.status_port.tim551
		self.HMI_papeLaunch.t_colorCAM = self.status_port.camera
		self.HMI_papeLaunch.t_colorMC = self.status_port.hc
		self.HMI_papeLaunch.t_colorMain = self.status_port.main
		self.HMI_papeLaunch.t_colorSC = self.status_port.sc
		self.HMI_papeLaunch.t_colorOC = self.status_port.oc
		self.HMI_papeLaunch.t_colorRFID = self.status_port.magLine

		self.pub_papeLaunch.publish(self.HMI_papeLaunch)

	def synthetic_byHand(self):
		self.HMI_papeByHand.t_tag = str(self.setpose_status.find_tag)

		if self.setpose_status.status == -1:
			self.HMI_papeByHand.t_colorSetpose = 0
		elif self.setpose_status.status > 0 and self.setpose_status.status < 7:
			self.HMI_papeByHand.t_colorSetpose = 1
		elif self.setpose_status.status < -1:
			self.HMI_papeByHand.t_colorSetpose = 2
		self.pub_papeByHand.publish(self.HMI_papeByHand)

	def syntheticInfo_detailNN(self):
		self.HMI_papeDetailNN.t_locateX = str(round(self.NN_infoRespond.x, 1))
		self.HMI_papeDetailNN.t_locateY = str(round(self.NN_infoRespond.y, 1))
		self.HMI_papeDetailNN.t_locateZ = str(round(self.NN_infoRespond.z, 1))

		self.HMI_papeDetailNN.t_goalX = str(round(self.goal_control.send_goal.pose.position.x, 1))
		self.HMI_papeDetailNN.t_goalY = str(round(self.goal_control.send_goal.pose.position.y, 1))
		self.HMI_papeDetailNN.t_goalZ = str(0)

		self.HMI_papeDetailNN.t_taskX = str(round(self.NN_cmdRequest.target_x, 1))
		self.HMI_papeDetailNN.t_taskY = str(round(self.NN_cmdRequest.target_y, 1))
		self.HMI_papeDetailNN.t_taskZ = str(round(self.NN_cmdRequest.target_z, 1))

		self.HMI_papeDetailNN.t_taskBefore = str(self.NN_cmdRequest.before_mission)
		self.HMI_papeDetailNN.t_taskAfter = str(self.NN_cmdRequest.after_mission)

		self.HMI_papeDetailNN.t_tag = str(self.NN_cmdRequest.tag)
		self.HMI_papeDetailNN.t_distance = str(self.NN_cmdRequest.offset)

		self.HMI_papeDetailNN.t_requirSpeedX = str(round(self.cmd_vel.linear.x, 2))
		self.HMI_papeDetailNN.t_requirSpeedZ = str(round(self.cmd_vel.angular.z, 2))

		self.HMI_papeDetailNN.t_setpose = str(0)
		self.HMI_papeDetailNN.t_reboot = str(0)
		self.HMI_papeDetailNN.t_process = str(0)
		self.HMI_papeDetailNN.t_funtion =str(0)

		self.pub_papeNN.publish(self.HMI_papeDetailNN)

	def synthetic_infoGeneral(self):
		if (self.NN_infoRespond.mode == 0):
			self.infoGeneral.pape = self.pape_launch
		else:
			# button
			if self.HMI_allButton.b_back == 1:
				self.is_displayDetail = 0
				self.is_displayMessage = 0
				self.is_displayPassword = 0
				
			if self.HMI_allButton.b_message == 1:
				self.is_displayMessage = 1
				self.is_displayDetail = 0
				self.is_displayPassword = 0

			if self.HMI_allButton.b_detail == 1:
				self.is_displayDetail = 1
				self.is_displayMessage = 0
				self.is_displayPassword = 0
		# -- add new
			if self.HMI_allButton.b_cancel == 1:
				self.is_displayDetail = 0
				self.is_displayMessage = 0
				self.is_displayPassword = 1

			if self.is_displayDetail == 0 and self.is_displayMessage == 0 and self.is_displayPassword == 0:
				if self.NN_infoRespond.mode == 2:
					self.infoGeneral.pape = self.pape_auto
				elif (self.NN_infoRespond.mode == 1):
					self.infoGeneral.pape = self.pape_byHand

			elif self.is_displayDetail == 1 and self.is_displayMessage == 0 and self.is_displayPassword == 0:
				if self.NN_infoRespond.mode == 2:
					self.infoGeneral.pape = self.pape_detailNN

			elif self.is_displayDetail == 0 and self.is_displayMessage == 1 and self.is_displayPassword == 0:
				self.infoGeneral.pape = self.pape_message

			elif self.is_displayDetail == 0 and self.is_displayMessage == 0 and self.is_displayPassword == 1:
				self.infoGeneral.pape = self.pape_password

		self.infoGeneral.t_nameAgv = self.nameAgv

		# ipv4 = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen('ip addr show wlp3s0').read()).groups()[0]                                                   
		# self.infoGeneral.t_ip = ipv4
		self.infoGeneral.t_ip = self.ip

		# status		
		# if self.NN_infoRespond.status == 0:
		err_ = self.NN_infoRespond.error_device

		# -- status		
		if self.NN_infoRespond.status == 5: # cancel mission
			self.infoGeneral.t_status = 3		
		if self.NN_infoRespond.status == 1: # warning
			self.infoGeneral.t_status = 1
		elif self.NN_infoRespond.status == 2: # error
			self.infoGeneral.t_status = 2
		else:
			self.infoGeneral.t_status = 0

		# error
		self.infoGeneral.n_error = err_
		# battery:
		if self.is_main82 == 1:		
			raw_bat = self.NN_infoRespond.battery/10.
			val_bat = (raw_bat - self.min_pin)/self.rate_vol
			if abs(val_bat) > 100:
				val_bat = 100
			elif val_bat < 0:
				val_bat = 0
				
			if self.mainInfo.charge_current >= 1.:
				val_bat = val_bat*(-1)

			self.infoGeneral.j_battery = int(val_bat)
			# print "BAT:", (val_bat)
			self.infoGeneral.t_voltage = str(round(raw_bat, 2))

		# -- add new
		self.infoGeneral.t_mission = self.NN_cmdRequest.command

		self.pub_infoGeneral.publish(self.infoGeneral)

	def synthetic_message(self):

		self.HMI_papeMessage.t_phone = "0988109936"

		id_er = self.NN_infoRespond.error_device
		self.HMI_papeMessage.p_typeError = self.picture_error(id_er)

		self.HMI_papeMessage.n_typeError = id_er
		self.HMI_papeMessage.p_methor = self.picture_methor(id_er)
		self.pub_papeMessage.publish(self.HMI_papeMessage)

	def syntheticInfo_Auto(self):
		# ----- mode run	
		self.HMI_papeAuto.p_modeRun = 47 # NN

		# ----- color safety zone
		if self.safety_zone.zone_ahead == 0 or self.safety_zone.zone_ahead == 3:
			self.HMI_papeAuto.t_safetyHeader = 0
		elif self.safety_zone.zone_ahead == 1:
			self.HMI_papeAuto.t_safetyHeader = 2
		elif self.safety_zone.zone_ahead == 2:
			self.HMI_papeAuto.t_safetyHeader = 1

		if self.safety_zone.zone_behind == 0 or self.safety_zone.zone_behind == 3:
			self.HMI_papeAuto.t_safetyBehind = 0
		elif self.safety_zone.zone_behind == 1:
			self.HMI_papeAuto.t_safetyBehind = 2
		elif self.safety_zone.zone_behind == 2:
			self.HMI_papeAuto.t_safetyBehind = 1

		# ----- color lidar 1, 2
		if self.status_port.lms100 == 1 and self.status_port.tim551 == 1:
			if self.status_reconnect.lidar.sts == 0:
				self.HMI_papeAuto.t_lidar1 = 0				
				self.HMI_papeAuto.t_lidar2 = 0				
			else:
				self.HMI_papeAuto.t_lidar1 = 1				
				self.HMI_papeAuto.t_lidar2 = 1

		elif self.status_port.lms100 == 0 and self.status_port.tim551 == 1:
			self.HMI_papeAuto.t_lidar1 = 2
			if self.status_reconnect.lidar.sts == 0:
				self.HMI_papeAuto.t_lidar2 = 0
			else:
				self.HMI_papeAuto.t_lidar2 = 1

		elif self.status_port.lms100 == 1 and self.status_port.tim551 == 0:
			self.HMI_papeAuto.t_lidar2 = 2
			if self.status_reconnect.lidar.sts == 0:
				self.HMI_papeAuto.t_lidar1 = 0
			else:
				self.HMI_papeAuto.t_lidar1 = 1

		elif self.status_port.lms100 == 0 and self.status_port.tim551 == 0:
			self.HMI_papeAuto.t_lidar1 = 2
			self.HMI_papeAuto.t_lidar2 = 2

		# --- Camera 3
		if self.status_port.camera == 1:
			if self.status_reconnect.camera.sts == 0:
				self.HMI_papeAuto.t_camera = 0
			else:
				self.HMI_papeAuto.t_camera = 1
		else:
			self.HMI_papeAuto.t_camera = 2

		# --- HC 4
		if self.status_port.hc == 1:
			if self.status_reconnect.hc.sts == 0:
				self.HMI_papeAuto.t_MC = 0
			else:
				self.HMI_papeAuto.t_MC = 1
		else:
			self.HMI_papeAuto.t_MC = 2

		# --- Main 5
		if self.status_port.main == 1:
			if self.status_reconnect.main.sts == 0:
				self.HMI_papeAuto.t_MN = 0
			else:
				self.HMI_papeAuto.t_MN = 1
		else:
			self.HMI_papeAuto.t_MN = 2

		# --- SC 6
		if self.status_port.sc == 1:
			if self.status_reconnect.sc.sts == 0:
				self.HMI_papeAuto.t_SC = 0
			else:
				self.HMI_papeAuto.t_SC = 1
		else:
			self.HMI_papeAuto.t_SC = 2

		# --- OC 7
		if self.status_port.oc == 1:
			if self.status_reconnect.oc.sts == 0:
				self.HMI_papeAuto.t_sys = 0
			else:
				self.HMI_papeAuto.t_sys = 1
		else:
			self.HMI_papeAuto.t_sys = 2

		# -- magLine 8
		if self.status_port.magLine == 1:
			if self.status_reconnect.magLine.sts == 0:
				self.HMI_papeAuto.t_setpose = 0
			else:
				self.HMI_papeAuto.t_setpose = 1
		else:
			self.HMI_papeAuto.t_setpose = 2

		# -- driver1 - 9
		if self.status_port.new1 == 1:
			if self.status_reconnect.driver1.sts == 0:
				self.HMI_papeAuto.t_driver1 = 0
			else:
				self.HMI_papeAuto.t_driver1 = 1
		else:
			self.HMI_papeAuto.t_driver1 = 2

		# -- driver2 - 10
		if self.status_port.new2 == 1:
			if self.status_reconnect.driver2.sts == 0:
				self.HMI_papeAuto.t_driver2 = 0
			else:
				self.HMI_papeAuto.t_driver2 = 1
		else:
			self.HMI_papeAuto.t_driver2 = 2

		self.pub_papeAuto.publish(self.HMI_papeAuto)

	def run(self):
		# if (self.is_newDataMain == 1):
		# 	self.is_newDataMain = 0
		# 	millis = int(round(time.time() * 1000))
		# 	self.timeMain = (millis - self.pre_timeMain)
		# 	self.pre_timeMain = millis

		# if (self.is_newDatamc == 1):
		# 	self.is_newDataMC = 0
		# 	millis = int(round(time.time() * 1000))
		# 	self.timeMC = (millis - self.pre_timeMC)
		# 	self.pre_timeMC = millis

		self.synthetic_infoGeneral()
		self.syntheticInfo_detailNN()
		self.synthetic_launch()
		self.synthetic_message()
		self.synthetic_byHand()
		self.syntheticInfo_Auto()
		self.authenticationPassword()

		self.rate.sleep()

def power():
	# Start the job threads
	class_1 = synthetic()

	# Keep the power thread running, otherwise signals are ignored.
	while not rospy.is_shutdown():
		class_1.run()

	class_1.exit()

if __name__ == '__main__':
    power()			
