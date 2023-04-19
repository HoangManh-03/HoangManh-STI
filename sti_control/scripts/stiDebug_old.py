#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 30/09/2020

"""
import roslib
from uptime import uptime
import sys
import time
from decimal import *
import math
import rospy
from datetime import datetime
# ip
import os
import re  
import subprocess
import argparse

from sti_msgs.msg import *
from std_msgs.msg import Float64
from message_pkg.msg import *
import psutil
#--------------------------------------------------------------------------------- ROS
class debug():
	def __init__(self):
		rospy.init_node('stiDebug', anonymous=False)
		self.rate = rospy.Rate(200)
	  # SUB - PUB
		# distance
		rospy.Subscriber("/quang_duong", Float64, self.distance_callback)
		self.distanceRuned = Float64()
		# -- MAIN - POWER
		rospy.Subscriber("/POWER_info", POWER_info, self.POWER_info_callback) 
		self.power_info = POWER_info()
		self.is_readMain = 0

		rospy.Subscriber("/POWER_request", POWER_request, self.POWER_request_callback) 
		self.power_request = POWER_request()
		self.flag_checkCharger = 0

		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.NN_cmdCallback)
		self.NN_cmdRequest = NN_cmdRequest()
		self.pre_NN_cmdRequest = NN_cmdRequest()		
		self.is_client = 1
		self.pre_timeCharger = 0
		self.count_requirCharger = 0
		self.count_ChagerOk = 0

		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.NN_infoCallback)
		self.NN_infoRespond = NN_infoRespond()

		rospy.Subscriber("/status_goal_control", Status_goal_control, self.goalControl_callback)
		self.status_goalControl = Status_goal_control()
		self.pre_status_goalControl = Status_goal_control()
		self.is_goalControl = 0

		rospy.Subscriber("/parking_status", Parking_status, self.parkingStatus_callback)
		self.parking_status = Parking_status()
		self.pre_parking_status = Parking_status()
		self.is_parkingStatus = 0

		rospy.Subscriber("/status_reconnect", Status_reconnect, self.reconnectStatus_callback)
		self.reconnect_status = Status_reconnect()
		self.pre_reconnect_status = Status_reconnect()
		self.is_reconnectStatus = 0

		self.name_card = "wlo2"
		# self.address = "192.168.1.41"
		self.address = "192.168.1.69"
		self.ip_lidar = "192.168.10.100"

		self.ip_server = "192.168.1.2"

		self.path_historyDelete = "/home/stivietnam/catkin_ws/debug/historyDelete.txt"
		self.path_log_NN = "/home/stivietnam/catkin_ws/debug/NAV_log_NN.txt"
		self.path_log_standard = "/home/stivietnam/catkin_ws/debug/NAV_logStandard.txt"
		self.path_log_charger = "/home/stivietnam/catkin_ws/debug/NAV_logCharger.txt"
		self.path_log_error = "/home/stivietnam/catkin_ws/debug/errorLog.txt"

		self.process = 0
		self.present_error = 0
		
		self.twoWeeks = 1209600 # s
		self.sevenDay = 604800 # s
		self.threeDay = 259200
		self.aHour = 3600
		self.fiveMinutes = 300
		self.timeDelete = self.sevenDay # self.sevenDay
		self.timeCheckEnbDelete = 40
		self.pre_timeCheckDelete = time.time()

		self.check_main = 0
		self.last_time_main = 0
		self.is_print_lostConnect_main = 0
		self.is_print_Reconnect_main = 0
		
		self.is_infoNN = 1

		self.is_print_lostConnect_mc = 0
		self.is_print_Reconnect_mc = 0

		self.battery = 0.0
		self.pre_battery = 0.0

		self.last_time_reconnect = 0
		self.preTime_infoNN = 0

		self.pre_mess = ""
		self.preTime_uptime = 0
		self.preTime_standard = 0

		self.preTime_uptime = 0
		self.preTime_standard = 0

		# -- add new
		self.serverMission_liftDown_charger = 6
		self.serverMission_charger = 5

	def distance_callback(self, dat):
		self.distanceRuned = dat

	def POWER_info_callback(self, dat):
		self.power_info = dat
		self.is_readMain = 1

	def POWER_request_callback(self, dat):
		self.power_request = dat

	def NN_cmdCallback(self, dat):
		self.NN_cmdRequest = dat
		self.is_client = 1

	def NN_infoCallback(self, dat):
		self.NN_infoRespond = dat
		self.is_infoNN = 1

	def goalControl_callback(self, dat):
		self.status_goalControl = dat
		self.is_goalControl = 1

	def parkingStatus_callback(self, dat):
		self.parking_status = dat
		self.is_parkingStatus = 1

	def setposeStatus_callback(self, dat):
		self.setpose_status = dat
		self.is_setposeStatus = 1

	def reconnectStatus_callback(self, dat):
		self.reconnect_status = dat
		self.is_reconnectStatus = 1

	def FL_infoCallback(self, dat):
		self.FL_infoRespond = dat
		self.is_main82 = 1
		self.check_main = 1
		self.is_FL_info = 1 

	def infoGeneral_Callback(self, dat):
		self.HMI_infoGeneral = dat

	def log_checkCharger(self): # sac dc hay ko.
		if self.power_request.charge == 1:
			if self.flag_checkCharger == 1:
				t = (time.time() - self.pre_timeCharger)%60
				if (t > 10):
					print "Log Charger: ", t
					self.pre_timeCharger = time.time()
					self.flag_checkCharger = 0
					self.count_requirCharger += 1
					cur = self.power_info.charge_current
					now = datetime.now()
					current_time = now.strftime("%B/%d|%H:%M:%S")
					if cur > 0.4:
						self.count_ChagerOk += 1

					self.file_log = open(self.path_log_charger, "a+")
					self.file_log.write("\n" + str(self.count_requirCharger) + ", "+ "OK: " + str(self.count_ChagerOk) + " I: " + str(cur) + "| T: " + str(current_time))
					self.file_log.close()
		else:
			self.pre_timeCharger = time.time()
			self.flag_checkCharger = 1

	def main_checkConnection(self): # not
		if self.check_main == 1:
			self.last_time_main = time.time()
			self.check_main = 0
		t = (time.time() - self.last_time_main)%60

		if (t > 20): # 20 s
			if self.is_print_lostConnect_main == 0:
				now = datetime.now()
				current_time = now.strftime("%B/%d|%H:%M:%S")

				self.file_log = open(self.path_log, "a+")
				self.file_log.write("\nLost connect Main| " + str(current_time))
				self.file_log.write("\n----------------------------------------")
				self.file_log.close()

				print("Lost connect Main|" , current_time)
				print "----------------------------------------"
				self.is_print_lostConnect_main = 1
				self.is_print_Reconnect_main = 0

		elif (t < 2): # 2 s
			if self.is_print_Reconnect_main == 0:
				now = datetime.now()
				current_time = now.strftime("%B/%d|%H:%M:%S")

				self.file_log = open(self.path_log, "a+")
				self.file_log.write("\nConnecting Main| "+ str(current_time))
				self.file_log.write("\n----------------------------------------")
				self.file_log.close()

				print("Connecting Main|", current_time)
				print "----------------------------------------"
				self.is_print_Reconnect_main = 1
				self.is_print_lostConnect_main = 0

	def logPing_nav_server():
		self.file_log = open(self.path_log_error, "a+")
		try:
			# -- wifi
			ping_wifi = str(subprocess.check_output("ping -{} 1 {}".format('c', self.ip_server), shell=True) )
			vitri_wifi = ping_wifi.find("time=")
			time_pingWifi = ping_wifi[(vitri_wifi + 5):(vitri_wifi + 9)]
			print ("time_pingWifi: ", time_pingWifi)
			self.file_log.write("\nP_wifi: " + str(time_pingWifi) )
		except Exception, e:
			self.file_log.write("\nP_wifi: " + "-1" )

		try:
			# -- ping nav
			ping_nav = str(subprocess.check_output("ping -{} 1 {}".format('c', self.ip_lidar), shell=True) )
			vitri_nav = ping_nav.find("time=")
			time_pingNav = ping_wifi[(vitri_nav + 5):(vitri_wifi + 9)]
			print ("time_pingRos: ", time_pingNav)
			self.file_log.write(" | P_nav: " + str(time_pingNav) )
			
		except Exception, e:
			self.file_log.write(" | P_nav: " + "-1" )
		self.file_log.close()

	def log_standard(self):
		t = (time.time() - self.preTime_standard)%60
		if (t > 10): # 10 s	
			self.preTime_standard = time.time()
			now = datetime.now()
				
			self.file_log = open(self.path_log_standard, "a+")
			# date, time
			current_time = now.strftime("%B/%d|%H:%M:%S")	
			self.file_log.write("\n" + str(current_time) )
			# battery
			bat = float(self.NN_infoRespond.battery)/10
			self.file_log.write(" " + str(bat) )
			# uptime
			times = uptime()
			hours = int(times/3600)
			minutes = int((times - hours*3600)/60)
			seconds = int(times - hours*3600 - minutes*60)
			self.file_log.write(" " + str(hours) + ":" + str(minutes) + ":" + str(seconds))
			# distance runed
			self.file_log.write(" " + str(round(self.distanceRuned.data, 2)))
			# db wifi
			try:
				db = subprocess.check_output("iwconfig {}".format(self.name_card), shell=True)
				# print(ping)
				vitri = db.find("Signal level")
				db_ = db[(vitri+13):(vitri+16)]
				self.file_log.write(" " + str(float(db_)) )
				# print("db= {}".format(float(db_)))

			except Exception, e:
				self.file_log.write(" " + "-1")
				# print("no db")

			# ping server
			try:
				ping = subprocess.check_output("ping -{} 1 {}".format('c',self.address), shell=True)
				# print(ping)
				vitri = ping.find("time")
				time_ping = ping[(vitri+5):(vitri+9)]
				self.file_log.write(" " + str(float(time_ping)) )
				# print("time= {}".format(float(time_ping)))

			except Exception, e:
				# print("no ping")			
				self.file_log.write(" " + "-1")

			# temperature
			try:
				sensors = subprocess.check_output("sensors", shell=True)
				vitri = sensors.find("Package id")
				temperature = sensors[(vitri+16):(vitri+20)]
				# print("temperature= {}".format(float(temperature)))
				self.file_log.write(" " + str(float(temperature)))

			except Exception, e:
				# print("no temperature")
				self.file_log.write(" " + "-1")
			
			try:
				cpu = psutil.cpu_percent() # cpu 
				# print("cpu= {}".format(float(cpu)))
				self.file_log.write(" " + str(float(cpu)))
			except Exception, e:
				# print("no cpu")
				self.file_log.write(" " + "-1")

			# RAM
			try:
				ram = psutil.virtual_memory().available * 100 / psutil.virtual_memory().total # ram
				# print("ram= {}".format(float(ram)))
				self.file_log.write(" " + str(float(ram)))
			except Exception, e:
				# print("no ram")
				self.file_log.write(" " + "-1")

			# Charger - request from server
			try:
				req = '0'
				if (self.NN_cmdRequest.after_mission == self.serverMission_charger or self.NN_cmdRequest.after_mission == self.serverMission_liftDown_charger):
					req = '1'
				else:
					req = '0'
				self.file_log.write(" " + req)
			except Exception, e:
				self.file_log.write(" " + "-1")

			# Charger - request to main
			try:
				req = '0'
				if (self.power_request.charge == 1):
					req = '1'
				else:
					req = '0'
				self.file_log.write(" " + req)
			except Exception, e:
				self.file_log.write(" " + "-1")

			# Charger CURRENT
			try:
				charge_current = self.power_info.charge_current
				self.file_log.write(" " + str(float(charge_current)))
			except Exception, e:
				# print("no ram")
				self.file_log.write(" " + "-1")

			self.file_log.close()

	def log_reconnect_v0(self): # ok
		t = (time.time() - self.last_time_reconnect)%60
		if (t > 1): # 1 s
			if self.is_reconnectStatus == 1:
				self.last_time_reconnect = time.time()
				self.is_reconnectStatus = 0

				now = datetime.now()
				current_time = now.strftime("%B/%d|%H:%M:%S")

				self.file_log = open(self.path_log_NN, "a+")
				self.file_log.write("\rReconnect_status| " + str(current_time))
				self.file_log.write(str(self.reconnect_status))
				self.file_log.write("\n----------------------------------------")
				self.file_log.close()

	def log_NN_info(self): # ok
		t = (time.time() - self.preTime_infoNN)%60
		if (t > 2): # 1 s
			self.preTime_infoNN = time.time()

			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_NN, "a+")
			self.file_log.write("\rNN info| " + str(current_time) + "\n")
			self.file_log.write(str(self.NN_infoRespond))
			self.file_log.write("\n----------------------------------------")
			self.file_log.close()

	def log_reconnect_v1(self): # ok
		if self.pre_reconnect_status != self.reconnect_status:
			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_NN, "a+")
			self.file_log.write("\rReconnect_status| " + str(current_time) +"\n")
			self.file_log.write(str(self.reconnect_status))
			self.file_log.write("\n----------------------------------------")
			self.file_log.close()
			self.pre_reconnect_status = self.reconnect_status

	def detailError(self, x):
		switcher={
			0: "ALL RIGHT",
			111:"Va vao Blsock",
			121:"An EMG",
			131:"Ra khoi duong tu",
			141:"Ban nang",
			211:"Camera: Mat ket noi vat ly",
			212:"Camera: Khong giao tiep truyen thong",
			221:"Lidar Phia truoc",
			222:"Lidar Phia sau",
			223:"Lidar Ca 2",
			231:"IMU Khong giao tiep truyen thong",
			241:"PS2 Khong giao tiep truyen thong",
			251:"DRIVER 1 (trai) NN – Natual Navigatiroson",
			261:"DRIVER 2 (phai) NN – Natual Navigation",
			271:"Mangnetic line: NN - RS232 – PC loi (phia truoc)",
			272:"Mangnetic line: NN - RS232 – PC loi (phia sau)",
			311:"Mach MC - NN.	Mat ket noi vat ly Serial",
			312:"Mach MC - NN. Khong giao tiep truyen thong Serial",
			321:"Mach Main - NN: Mat ket noi vat ly Serial",
			322:"Mach Main - NN: Khong giao tiep truyen thong Serial",
			331:"Mach SC: Mat ket noi vat ly Serial",
			332:"Mach SC: Khong giao tiep truyen thong Serial",
			341:"Mach OC: Mat ket noi vat ly Serial",
			342:"Mach OC: Khong giao tiep truyen thong Serial",
			351:"Mach HC: Mat ket noi vat ly Serial",
			352:"Mach HC: Khong giao tiep truyen thong Serial",
			411:"Di chuyen NN (khong vach tu) - Khong the den duoc dich",
			421:"Co vat can",
			431:"Mat ket noi server",
			441:"Khong the thay Tag",
			451:"Dien ap Thap",
			461:"Co vat can khi vao ke",
			471:"khong co ke hoac lech ke khi nang"
		}
		return switcher.get(x, "Don't know!")		

	def log_status(self):
		mess = ''
		if (self.old_HMI_infoGeneral.n_error != self.HMI_infoGeneral.n_error): # ok
			self.present_error = self.HMI_infoGeneral.n_error
			mess = self.detailError(self.present_error)

			self.old_HMI_infoGeneral = self.HMI_infoGeneral

			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_error, "a+")
			self.file_log.write("\nrespond Move| " + mess + "\n")
			self.file_log.write(str(self.HMI_infoGeneral))
			self.file_log.write("\nE: " + str(current_time) + "\n")
			self.file_log.write("----------------------------------------")
			self.file_log.close()

	def log_mess(self, typ, mess, val):
		if self.pre_mess != mess:
			if typ == "info":
				rospy.loginfo (mess + ": %s", val)
			elif typ == "warn":
				rospy.logwarn (mess + ": %s", val)
			else:
				rospy.logerr (mess + ": %s", val)	
		self.pre_mess = mess

	def enb_checkDelete(self):
		t_now = time.time()
		t = (t_now - self.pre_timeCheckDelete)%60
		if (t > self.timeCheckEnbDelete): # s
			self.pre_timeCheckDelete = t_now
			print "check enb delete"
			return 1
		else:
			return 0

	def enb_delete(self):
		is_ok = 0
	  # 1, Lay dong cuoi cung.
		file_log = open(self.path_historyDelete, "a+")
		rowNow = sum(1 for _ in file_log) - 1
		# print "rowNow:", rowNow
		file_log.close()
  	  # 2, Lay thoi gian cuoi cung.
		file_log = open(self.path_historyDelete, "a+")
		raw_text = file_log.readlines()
		try:
			textAll = raw_text[rowNow]
			# print "Read:", textAll
			
			textFinal = textAll.split('|')
			timePre = int(textFinal[0])
			# print "TIME PRE:", timePre
			is_ok = 1
		except:
			is_ok = -1
			print "ERROR READ DATA"	
		file_log.close()

	  # 3, So sanh thoi gian truoc va sau -> Ghi log + xoa.
		if is_ok == 1:
			if time.time() - timePre > self.timeDelete: #sevenDay
				return 1
			else:
				return 0
		else:
			return -1

	def deleteFIleLog(self):
		# -- Log history
		now = datetime.now()
		current_time = now.strftime("%B/%d|%H:%M:%S")

		file_log = open(self.path_historyDelete, "a+")
		file_log.write(str(int(time.time())) + "|" + str(current_time) +'\n')
		file_log.close()
		# -- delete file
		try:
			os.remove(self.path_log_NN)
		except:
			print "ERROR DELETE FILE log_NN"

		try:
			os.remove(self.path_log_standard)
		except:
			print "ERROR DELETE FILE log_standard"

		try:
			os.remove(self.path_log_charger)
		except:
			print "ERROR DELETE FILE log_charger"

		try:
			os.remove(self.path_log_error)
		except:
			print "ERROR DELETE FILE log_error"

		try:
			os.system("yes | rosclean purge") # xoa log ROS
		except:
			print "ERROR DELETE LOG ROS"

		print "DELETE FILE COMPLETED"

			
	def createFIleHhistoryDelete(self):
		# -- Log history
		now = datetime.now()
		current_time = now.strftime("%B/%d|%H:%M:%S")

		file_log = open(self.path_historyDelete, "a+")
		file_log.write(str(int(time.time())) + "|" + str(current_time) +'\n')
		file_log.close()
		print "CREATE FILE HISTORY NEW"

	def run(self):
		if self.process == 0:	# chờ cac node khoi dong xong.
			self.process = 1

		if self.process == 1:	# chờ cac node khoi dong xong.
			ct = 7

			if (ct == 7):
				self.log_mess("info", "All Right", 0)
				self.file_log = open(self.path_log_charger, "a+")
				self.file_log.write("\n-------------------------------------")
				self.file_log.close()
				self.process = 2

		elif self.process == 2:
			if self.enb_checkDelete() == 1:
				dlt = self.enb_delete()
				if dlt == 0:
					self.process = 3
				elif dlt == 1:
					self.deleteFIleLog()
				elif dlt == -1:
					self.createFIleHhistoryDelete()
			else:
				self.process = 3

		elif self.process == 3:
			if (self.pre_NN_cmdRequest != self.NN_cmdRequest): # ok
				now = datetime.now()
				current_time = now.strftime("%B/%d|%H:%M:%S")

				self.file_log = open(self.path_log_NN, "a+")
				self.file_log.write("\nNN cmd request| ")
				self.file_log.write(str(current_time) + "\n")
				self.file_log.write(str(self.NN_cmdRequest))
				self.file_log.write("\n----------------------------------------")
				self.file_log.close()
				
				self.pre_NN_cmdRequest = self.NN_cmdRequest

			if (self.pre_status_goalControl != self.status_goalControl): # ok
				now = datetime.now()
				current_time = now.strftime("%B/%d|%H:%M:%S")

				self.file_log = open(self.path_log_NN, "a+")
				self.file_log.write("\nGoal control status| " + str(current_time) + "\n")
				self.file_log.write(str(self.status_goalControl))
				self.file_log.write("\n----------------------------------------")
				self.file_log.close()

				self.pre_status_goalControl = self.status_goalControl

			if (self.pre_parking_status != self.parking_status): # ok
				now = datetime.now()
				current_time = now.strftime("%B/%d|%H:%M:%S")

				self.file_log = open(self.path_log_NN, "a+")
				self.file_log.write("\nParking status| " + str(current_time) + "\n")
				self.file_log.write(str(self.parking_status))
				self.file_log.write("\n----------------------------------------")
				self.file_log.close()

				self.pre_parking_status = self.parking_status

			if (self.pre_setpose_status != self.setpose_status): # ok
				now = datetime.now()
				current_time = now.strftime("%B/%d|%H:%M:%S")

				self.file_log = open(self.path_log_NN, "a+")
				self.file_log.write("\nSetpose status| " + str(current_time) + "\n")
				self.file_log.write(str(self.setpose_status))
				self.file_log.write("\n----------------------------------------")
				self.file_log.close()

				self.pre_setpose_status = self.setpose_status				

			# if (self.pre_respond_move != self.respond_move): # ok
			# 	now = datetime.now()
			# 	current_time = now.strftime("%B/%d|%H:%M:%S")

			# 	self.file_log = open(self.path_log_NN, "a+")
			# 	self.file_log.write("\nrespond Move| " + str(current_time) + "\n")
			# 	self.file_log.write(str(self.respond_move))
			# 	self.file_log.write("\n----------------------------------------")
			# 	self.file_log.close()

			# 	self.pre_respond_move = self.respond_move	

			# self.log_reconnect_v1()
			self.log_standard()
			self.log_NN_info()
			# self.log_checkCharger()

			self.process = 2
		self.rate.sleep()

def main():
	# Start the job threads
	class_1 = debug()		
	# Keep the main thread running, otherwise signals are ignored.
	while not rospy.is_shutdown():
		class_1.run()
	# file_log.close() 

if __name__ == '__main__':
	main()
