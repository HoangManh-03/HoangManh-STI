#!/usr/bin/env python
#-*- coding: utf-8 -*-
#Author: Hoang Van Quang - BEE
#Date: 26/12/2020
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
from datetime import datetime
from sti_msgs.msg import HMI_allButton
from sti_msgs.msg import HMI_papeLaunch
from sti_msgs.msg import HMI_papeAuto
from sti_msgs.msg import HMI_papeByHand
from sti_msgs.msg import HMI_papeDetailFL
from sti_msgs.msg import HMI_papeDetailNN
from sti_msgs.msg import HMI_papeMessage
from sti_msgs.msg import HMI_infoGeneral
from sti_msgs.msg import HMI_buttonPassword
from sti_msgs.msg import HMI_papePassword

class communicate_nextion(threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.shutdown_flag = threading.Event()

		rospy.init_node('stiNextion', anonymous=False)
		self.rate = rospy.Rate(50)		

		self.pub_button = rospy.Publisher("/HMI_allButton", HMI_allButton , queue_size=1)
		self.allButton = HMI_allButton()
		self.allButton_old = HMI_allButton()

		self.pub_buttonPassword = rospy.Publisher("/HMI_buttonPassword", HMI_buttonPassword , queue_size=1)
		self.buttonPassword = HMI_buttonPassword()

		rospy.Subscriber("/HMI_papeAuto", HMI_papeAuto, self.papeAuto_callback)
		self.papeAuto = HMI_papeAuto()

		rospy.Subscriber("/HMI_papeByHand", HMI_papeByHand, self.papeByHand_callback)		
		self.papeByHand = HMI_papeByHand()

		rospy.Subscriber("/HMI_papeLaunch", HMI_papeLaunch, self.papeLaunch_callback)
		self.papeLaunch = HMI_papeLaunch()

		rospy.Subscriber("/HMI_papeDetailFL", HMI_papeDetailFL, self.papeDetailFL_callback)
		self.papeDetailFL = HMI_papeDetailFL()

		rospy.Subscriber("/HMI_papeDetailNN", HMI_papeDetailNN, self.papeDetailNN_callback)
		self.papeDetailNN = HMI_papeDetailNN()

		rospy.Subscriber("/HMI_papeMessage", HMI_papeMessage, self.papeMessage_callback)
		self.papeMessage = HMI_papeMessage()

		rospy.Subscriber("/HMI_infoGeneral", HMI_infoGeneral, self.infoGeneral_callback)
		self.infoGeneral = HMI_infoGeneral()

		rospy.Subscriber("/HMI_papePassword", HMI_papePassword, self.papePassword_callback)
		self.papePassword = HMI_papePassword()		
		# data
	# -- Serrial
		self.ser = serial.Serial()
		# self.ser.baudrate = 115200
		self.ser.baudrate = 115200 # 115200
		self.port_hmi = rospy.get_param("port_hmi", 'stibase_hmi')
		self.ser.port = '/dev/' + self.port_hmi
		
		# self.ser.port = '/dev/ttyUSB0' 
		self.ser.timeout = 0.15
		self.ser.parity = serial.PARITY_NONE
		self.ser.stopbits = serial.STOPBITS_ONE
		self.ser.bytesize = serial.EIGHTBITS

	# -- Connect to serial
		try :
			print "Connect with Port:", self.ser.port 
			print "Baudrate:", self.ser.baudrate
			self.ser.open()
			self.ser.flushInput()
			self.ser.flushOutput()				
		except:
			print "Failed to Open Port:", self.ser.port 
			print "Baudrate:", self.ser.baudrate
			self.ser.close()
			print('Serial stopped')			
			sys.exit()

	# -- Declare ID:
		self.frequenceUpdate = 4 # Hz 
		self.pre_time = 10 # 
		self.endFrame = [255, 255, 255]
		self.current_mode = 0
		self.hmi_colorRed = 63488
		self.hmi_colorGreen = 1606
		self.hmi_colorGray = 50712
		self.hmi_colorYellow = 62400
		self.hmi_colorYellow_c = 65504
		self.hmi_colorOrange = 64520
		#-- Pape
		self.currentPape = 0
		self.pape_launch = 0
		self.pape_auto = 1
		self.pape_detailFL = 2
		self.pape_detailNN = 3
		self.pape_byHand = 4
		self.pape_message = 5
		self.pape_password = 6
      # DISPLAY
	   # General information	
		#-- Text display:
		self.nextPape(0)
	  # Image ID error
	
		self.byte_stop = 255
		self.old_data = [] # du lieu chua dc xu ly

	def papePassword_callback(self, dat):
		self.papePassword = dat

	def papeByHand_callback(self, dat):
		self.papeByHand = dat

	def papeAuto_callback(self, dat):
		self.papeAuto = dat

	def infoGeneral_callback(self, dat):
		self.infoGeneral = dat

	def papeLaunch_callback(self, dat):
		self.papeLaunch = dat

	def papeDetailFL_callback(self, dat):
		self.papeDetailFL = dat

	def papeDetailNN_callback(self, dat):
		self.papeDetailNN = dat

	def papeMessage_callback(self, dat):
		self.papeMessage = dat

	def nextPape(self, val):
		self.ser.write("dp=" + str(val))
		self.ser.write(self.endFrame)
		time.sleep(0.01)

	def sent_number(self, name_number, val):
		self.ser.write(name_number + ".val=" + str(val))
		self.ser.write(self.endFrame)
		time.sleep(0.01)

	def sent_text(self, name_text, txt):
		self.ser.write(name_text + ".txt=\"" + txt + "\\r\"")
		self.ser.write(self.endFrame)
		time.sleep(0.01)

	def sent_pic(self, name_text, pic):
		self.ser.write(name_text + ".pic=" + str(pic))
		self.ser.write(self.endFrame)
		time.sleep(0.01)

	def sent_color(self, name_obj, color): # for text, number, ...
		self.ser.write(name_obj + ".bco=" + str(color))
		self.ser.write(self.endFrame)
		time.sleep(0.01)

	def sent_color2(self, name_obj, color, typ): # for procsesBar | typ = 1(.bco) or 2(.pco)
		if typ == 1:
			self.ser.write(name_obj + ".bco=" + str(color))
			self.ser.write(self.endFrame)
			time.sleep(0.01)			
		elif typ == 2:
			self.ser.write(name_obj + ".pco=" + str(color))
			self.ser.write(self.endFrame)
			time.sleep(0.01)

	def stop_serial(self):
		self.ser.close()
		print "Close serial!"

	def exit(self):
		self.stop_serial()
		print('Exiting main program!')

	def moveAll_element(self, arr):
		l = len(arr)
		for i in range(l):
			arr.remove(arr[0])

	def analysis_frame(self, data): # rec <=>  bytes
		p_start = -1
		p_stop = -1
		l = len(data)
		# for i in range(l):
		# 	print "dd:", ord(self.data[i])
		# print self.data
		for i in range(l):
			if p_stop == -1: # locate position Byte stop
				lc = l - i - 1 # 
				if data[lc] == self.byte_stop:
					if lc > 5: # 255, 255, 255,| 1, 255, 255, 255
						if data[lc - 1] == self.byte_stop:
							if data[lc - 2] == self.byte_stop:
								p_stop = lc - 3
					else:
						break
			else:
				if lc > 2:
					p_start = p_stop - 3
				# if data[lc] == self.byte_stop: # locate position Byte stop
				# 	if lc > 2:
				# 		if data[lc - 1] == self.byte_stop:
				# 			if data[lc - 2] == self.byte_stop:
				# 				p_start = lc - 3
				# 				break
				# 	else:
				# 		break

		# print "start- stop", p_start, p_stop
		if p_stop != -1 and p_start != -1: 
			ll = p_stop - p_start + 1
			if (data[p_start] == 101):
				bt = [data[p_start + 1], data[p_start + 2], data[p_start + 3]]
				self.moveAll_element(self.old_data)
				return bt
			else:
				bt = []
				return bt
		else:
			bt = []
			return bt

	def classify_button(self, dat):
# -- add new		
	  # -- cancel mission
		if (dat[0] == 1) and (dat[1] == 24):
			self.allButton.b_cancel = 1
		if (dat[0] == 2) and (dat[1] == 20):
			self.allButton.b_cancel = 1
		if (dat[0] == 3) and (dat[1] == 23):
			self.allButton.b_cancel = 1
		if (dat[0] == 4) and (dat[1] == 19):
			self.allButton.b_cancel = 1
		if (dat[0] == 5) and (dat[1] == 10):
			self.allButton.b_cancel = 1

	  # clear error
		if (dat[0] == 1) and (dat[1] == 6) and (dat[2] == 1):
			self.allButton.b_clearError = 1
		if (dat[0] == 1) and (dat[1] == 6) and (dat[2] == 0):
			self.allButton.b_clearError = 0

		if (dat[0] == 4) and (dat[1] == 1) and (dat[2] == 1):
			self.allButton.b_clearError = 1
		if (dat[0] == 4) and (dat[1] == 1) and (dat[2] == 0):
			self.allButton.b_clearError = 0			
			#print "clearError"
	  # message
		if (dat[0] == 1) and (dat[1] == 22):
			self.allButton.b_message = 1
		if (dat[0] == 4) and (dat[1] == 18):
			self.allButton.b_message = 1

	  # detail parammeter
		if (dat[0] == 1) and (dat[1] == 20):
			self.allButton.b_detail = 1
	  # back
		if (dat[0] == 2) and (dat[1] == 1):
			self.allButton.b_back = 1
		if (dat[0] == 3) and (dat[1] == 1):
			self.allButton.b_back = 1
		if (dat[0] == 5) and (dat[1] == 1):
			self.allButton.b_back = 1						

	  # nextMode
		# pape auto
		if (dat[0] == 1) and (dat[1] == 7):
			self.allButton.b_modeBH = 1
			#print "nextMode"
		# pape by hand
		if (dat[0] == 4) and (dat[1] == 2):
			self.allButton.b_modeAuto = 1
			#print "nextMode"

	  # control device - pape byhand
		if (dat[0] == 4) and (dat[1] == 8):
			self.allButton.b_lifting = 1
			#print "Lifting"

		if (dat[0] == 4) and (dat[1] == 9):
			self.allButton.b_lower = 1
			#print "Lower"

		if (dat[0] == 4) and (dat[1] == 10):
			self.allButton.b_speakerOn = 1
			#print "Turn on speaker"

		if (dat[0] == 4) and (dat[1] == 11):
			self.allButton.b_speakerOff = 1
			#print "Turn off speaker"

		if (dat[0] == 4) and (dat[1] == 12):
			self.allButton.b_chargerOn = 1
			#print "Turn on charger"

		if (dat[0] == 4) and (dat[1] == 13):
			self.allButton.b_chargerOff = 1
			#print "Turn off charger"

		if (dat[0] == 4) and (dat[1] == 14):
			self.allButton.b_setpose = 1
			#print "Turn on funtion"
# -- add new
	def classify_buttonPassword(self, dat):
		# -- 0
		if (dat[0] == 6) and (dat[1] == 12):
			self.buttonPassword.b_n0 = 1		
		# -- 1
		if (dat[0] == 6) and (dat[1] == 2):
			self.buttonPassword.b_n1 = 1
		# -- 2
		if (dat[0] == 6) and (dat[1] == 3):
			self.buttonPassword.b_n2 = 1
		# -- 3
		if (dat[0] == 6) and (dat[1] == 4):
			self.buttonPassword.b_n3 = 1		
		# -- 4
		if (dat[0] == 6) and (dat[1] == 5):
			self.buttonPassword.b_n4 = 1
		# -- 5
		if (dat[0] == 6) and (dat[1] == 6):
			self.buttonPassword.b_n5 = 1
		# -- 6
		if (dat[0] == 6) and (dat[1] == 7):
			self.buttonPassword.b_n6 = 1		
		# -- 7
		if (dat[0] == 6) and (dat[1] == 8):
			self.buttonPassword.b_n7 = 1
		# -- 8
		if (dat[0] == 6) and (dat[1] == 9):
			self.buttonPassword.b_n8 = 1
		# -- 9
		if (dat[0] == 6) and (dat[1] == 10):
			self.buttonPassword.b_n9 = 1		
		# -- delete
		if (dat[0] == 6) and (dat[1] == 11):
			self.buttonPassword.b_delete = 1
		# -- clear
		if (dat[0] == 6) and (dat[1] == 13):
			self.buttonPassword.b_clear = 1
		# -- confirm
		if (dat[0] == 6) and (dat[1] == 14):
			self.buttonPassword.b_confirm = 1
		# -- cancel
		if (dat[0] == 6) and (dat[1] == 15):
			self.buttonPassword.b_cancel = 1

	def read_nextion(self):
		# -- Read and Analysis data.
		data_ = self.ser.readline()
		lenght = len(data_)

		if lenght > 0:
			for i in range(lenght):
				self.old_data.append(ord(data_[i]))
			print self.old_data
			if len(self.old_data) > 6:
				dat_bt = self.analysis_frame(self.old_data) # bt[0,1] -> 0:Pape | 1: id_button.
				if len(dat_bt) > 0:
					self.classify_button(dat_bt)
					self.classify_buttonPassword(dat_bt)
# -- add new					
					self.allButton_old = self.allButton
				else:
					print "Error analysis_frame!"
				
		else:
			# giu lai trang thai xoa loi. con lai reset toan bo nut nhan.
			self.allButton = HMI_allButton()
			self.allButton.b_clearError = self.allButton_old.b_clearError
			# -- clear status
			self.buttonPassword = HMI_buttonPassword()
# -- add new
		# -- PUB
		self.pub_button.publish(self.allButton)
		self.pub_buttonPassword.publish(self.buttonPassword)

	def draw_battery(self):
		if (self.infoGeneral.j_battery < 0): # charging battery
			self.sent_color2("j_battery", self.hmi_colorYellow_c, 2)
			self.sent_number("j_battery", abs(self.infoGeneral.j_battery))
			# self.sent_number("j_battery", 0)

		elif (self.infoGeneral.j_battery >= 0) and (self.infoGeneral.j_battery <= 30):
			self.sent_color2("j_battery", self.hmi_colorRed, 2)
			self.sent_number("j_battery", self.infoGeneral.j_battery)
		elif (self.infoGeneral.j_battery > 30) and (self.infoGeneral.j_battery <= 50):
			self.sent_color2("j_battery", self.hmi_colorYellow, 2)
			self.sent_number("j_battery", self.infoGeneral.j_battery)
		elif (self.infoGeneral.j_battery > 50) and (self.infoGeneral.j_battery <= 100):
			self.sent_color2("j_battery", self.hmi_colorGreen, 2)
			self.sent_number("j_battery", self.infoGeneral.j_battery)

	def sent_infoGeneral(self):
# -- add new		
		if self.currentPape != self.pape_launch: 
			self.sent_text("t_mission", self.infoGeneral.t_mission)

		self.sent_text("t_nameAgv", self.infoGeneral.t_nameAgv)
		self.sent_text("t_ip", self.infoGeneral.t_ip)	
		
		self.draw_battery()

		self.sent_text("t_voltage", self.infoGeneral.t_voltage)
		# nb_err = self.processImage(self.infoGeneral.n_error)
		# self.sent_number("n_error", nb_err)
		self.sent_number("n_error", self.infoGeneral.n_error)
# -- add new
		if (self.infoGeneral.t_status == 0):   # all right
			self.sent_color("t_status", self.hmi_colorGreen)
		elif (self.infoGeneral.t_status == 1): # warning
			self.sent_color("t_status", self.hmi_colorOrange)
		elif (self.infoGeneral.t_status == 2): # error
			self.sent_color("t_status", self.hmi_colorRed)
		elif (self.infoGeneral.t_status == 3): # cancel mission
			self.sent_color("t_status", self.hmi_colorYellow_c)

	def sent_pape(self, pape):
		if (pape == self.pape_launch): # ok
			self.draw_battery()

			self.sent_text("t_voltage", self.infoGeneral.t_voltage)

			self.sent_number("j_waitLaunch", self.papeLaunch.j_waitLaunch)
			self.sent_number("n_launch", self.papeLaunch.n_launch)
			self.sent_text("t_message", self.papeLaunch.t_message)
			self.sent_text("t_nameAgv", self.infoGeneral.t_nameAgv)
			self.sent_text("t_ip", self.infoGeneral.t_ip)
			# 1
			ld1 = self.papeLaunch.t_colorLD1
			ld1_cl = self.hmi_colorGreen
			if ld1 == 1:
				ld1_cl = self.hmi_colorGreen
			elif ld1 == 0:
				ld1_cl = self.hmi_colorRed			
			self.sent_color("t_colorLD1", ld1_cl)
			# 2
			ld2 = self.papeLaunch.t_colorLD2
			ld2_cl = self.hmi_colorGreen
			if ld2 == 1:
				ld2_cl = self.hmi_colorGreen
			elif ld2 == 0:
				ld2_cl = self.hmi_colorRed			
			self.sent_color("t_colorLD2", ld2_cl)
			# 3 - camera
			ca = self.papeLaunch.t_colorCAM
			ca_cl = self.hmi_colorGreen
			if ca == 1:
				ca_cl = self.hmi_colorGreen
			elif ca == 0:
				ca_cl = self.hmi_colorRed			
			self.sent_color("t_colorCAM", ca_cl)
			# 4
			mc = self.papeLaunch.t_colorMC
			mc_cl = self.hmi_colorGreen
			if mc == 1:
				mc_cl = self.hmi_colorGreen
			elif mc == 0:
				mc_cl = self.hmi_colorRed			
			self.sent_color("t_colorMC", mc_cl)
			# 5
			mai = self.papeLaunch.t_colorMain
			mai_cl = self.hmi_colorGreen
			if mai == 1:
				mai_cl = self.hmi_colorGreen
			elif mai == 0:
				mai_cl = self.hmi_colorRed			
			self.sent_color("t_colorMain", mai_cl)
			# 6
			sc = self.papeLaunch.t_colorSC
			sc_cl = self.hmi_colorGreen
			if sc == 1:
				sc_cl = self.hmi_colorGreen
			elif sc == 0:
				sc_cl = self.hmi_colorRed			
			self.sent_color("t_colorSC", sc_cl)
			# 7			
			oc = self.papeLaunch.t_colorOC
			oc_cl = self.hmi_colorGreen
			if oc == 1:
				oc_cl = self.hmi_colorGreen
			elif oc == 0:
				oc_cl = self.hmi_colorRed			
			self.sent_color("t_colorOC", oc_cl)			
			# 8
			rf = self.papeLaunch.t_colorRFID
			rf_cl = self.hmi_colorGreen
			if rf == 1:
				rf_cl = self.hmi_colorGreen
			elif rf == 0:
				rf_cl = self.hmi_colorRed			
			self.sent_color("t_colorRFID", rf_cl)

		elif (pape == self.pape_auto):
			self.sent_infoGeneral()
			self.sent_pic("p_modeRun", self.papeAuto.p_modeRun)

			sh = self.papeAuto.t_safetyHeader
			cl_sh = self.hmi_colorGreen
			if sh == 0:
				cl_sh = self.hmi_colorGreen
			elif sh == 1:
				cl_sh = self.hmi_colorOrange
			elif sh == 2:
				cl_sh = self.hmi_colorRed
			self.sent_color("t_safetyHeader", cl_sh)

			sb = self.papeAuto.t_safetyBehind
			cl_sb = self.hmi_colorGreen
			if sb == 0:
				cl_sb = self.hmi_colorGreen
			elif sb == 1:
				cl_sb = self.hmi_colorOrange
			elif sb == 2:
				cl_sb = self.hmi_colorRed
			self.sent_color("t_safetyBeHind", cl_sb)

			sl1 = self.papeAuto.t_lidar1
			cl_sl1 = self.hmi_colorGreen
			if sl1 == 0:
				cl_sl1 = self.hmi_colorGreen
			elif sl1 == 1:
				cl_sl1 = self.hmi_colorOrange
			elif sl1 == 2:
				cl_sl1 = self.hmi_colorRed
			self.sent_color("t_lidar1", cl_sl1)

			sl2 = self.papeAuto.t_lidar2
			cl_sl2 = self.hmi_colorGreen
			if sl2 == 0:
				cl_sl2 = self.hmi_colorGreen
			elif sl2 == 1:
				cl_sl2 = self.hmi_colorOrange
			elif sl2 == 2:
				cl_sl2 = self.hmi_colorRed
			self.sent_color("t_lidar2", cl_sl2)

			sd1 = self.papeAuto.t_driver1
			cl_sd1 = self.hmi_colorGreen
			if sd1 == 0:
				cl_sd1 = self.hmi_colorGreen
			elif sd1 == 1:
				cl_sd1 = self.hmi_colorOrange
			elif sd1 == 2:
				cl_sd1 = self.hmi_colorRed
			self.sent_color("t_driver1", cl_sd1)

			sd2 = self.papeAuto.t_driver2
			cl_sd2 = self.hmi_colorGreen
			if sd2 == 0:
				cl_sd2 = self.hmi_colorGreen
			elif sd2 == 1:
				cl_sd2 = self.hmi_colorOrange
			elif sd2 == 2:
				cl_sd2 = self.hmi_colorRed
			self.sent_color("t_driver2", cl_sd2)

			sp = self.papeAuto.t_setpose
			cl_sp = self.hmi_colorGreen
			if sp == 0:
				cl_sp = self.hmi_colorGreen
			elif sp == 1:
				cl_sp = self.hmi_colorOrange
			elif sp == 2:
				cl_sp = self.hmi_colorRed
			self.sent_color("t_setpose", cl_sp)

			ss = self.papeAuto.t_sys
			cl_ss = self.hmi_colorGreen
			if ss == 0:
				cl_ss = self.hmi_colorGreen
			elif ss == 1:
				cl_ss = self.hmi_colorOrange
			elif ss == 2:
				cl_ss = self.hmi_colorRed
			self.sent_color("t_sys", cl_ss)

			sn = self.papeAuto.t_MN
			cl_sn = self.hmi_colorGreen
			if sn == 0:
				cl_sn = self.hmi_colorGreen
			elif sn == 1:
				cl_sn = self.hmi_colorOrange
			elif sn == 2:
				cl_sn = self.hmi_colorRed
			self.sent_color("t_MN", cl_sn)

			sm = self.papeAuto.t_MC
			cl_sm = self.hmi_colorGreen
			if sm == 0:
				cl_sm = self.hmi_colorGreen
			elif sm == 1:
				cl_sm = self.hmi_colorOrange
			elif sm == 2:
				cl_sm = self.hmi_colorRed
			self.sent_color("t_MC", cl_sm)

			sc = self.papeAuto.t_SC
			cl_sc = self.hmi_colorGreen
			if sc == 0:
				cl_sc = self.hmi_colorGreen
			elif sc == 1:
				cl_sc = self.hmi_colorOrange
			elif sc == 2:
				cl_sc = self.hmi_colorRed
			self.sent_color("t_SC", cl_sc)

			ca = self.papeAuto.t_camera
			cl_ca = self.hmi_colorGreen
			if ca == 0:
				cl_ca = self.hmi_colorGreen
			elif ca == 1:
				cl_ca = self.hmi_colorOrange
			elif ca == 2:
				cl_ca = self.hmi_colorRed
			self.sent_color("t_camera", cl_ca)

		elif (pape == self.pape_detailFL):
			self.draw_battery()
			self.sent_text("t_voltage", self.infoGeneral.t_voltage)
			# vi tri:
			self.sent_number("n_locatePoint", self.papeDetailFL.n_locatePoint)
			self.sent_number("n_locateDir", self.papeDetailFL.n_locateDir)

			# # nhiem vu:
			self.sent_number("n_taskPoint", self.papeDetailFL.n_taskPoint)
			self.sent_number("n_taskDir", self.papeDetailFL.n_taskDir)			
			self.sent_number("n_taskBefor", self.papeDetailFL.n_taskBefor)
			self.sent_number("n_taskAfter", self.papeDetailFL.n_taskAfter)
			self.sent_number("n_lineTr", self.papeDetailFL.n_lineTr)
			self.sent_number("n_lineSa", self.papeDetailFL.n_lineSa)
			self.sent_number("n_aBit", self.papeDetailFL.n_aBit)
			self.sent_number("n_ssLf", self.papeDetailFL.n_ssLf)

			# # toc do yeu cau:
			self.sent_number("n_requirSpeed", self.papeDetailFL.n_requirSpeed)

			# # the tu:
			self.sent_number("n_cardByte0", self.papeDetailFL.n_cardByte0)
			self.sent_number("n_cardByte1", self.papeDetailFL.n_cardByte1)
			self.sent_number("n_cardByte2", self.papeDetailFL.n_cardByte2)
			self.sent_number("n_cardByte3", self.papeDetailFL.n_cardByte3)
			self.sent_number("n_cardByte4", self.papeDetailFL.n_cardByte4)

		elif (pape == self.pape_detailNN):
			self.draw_battery()
			self.sent_text("t_voltage", self.infoGeneral.t_voltage)
			# vi tri:
			self.sent_text("t_locateX", self.papeDetailNN.t_locateX)
			self.sent_text("t_locateY", self.papeDetailNN.t_locateY)
			self.sent_text("t_locateZ", self.papeDetailNN.t_locateZ)

			# dich(goal)
			self.sent_text("t_goalX", self.papeDetailNN.t_goalX)
			self.sent_text("t_goalY", self.papeDetailNN.t_goalY)
			self.sent_text("t_goalZ", self.papeDetailNN.t_goalZ)
			
			# nhiem vu:
			self.sent_text("t_taskX", self.papeDetailNN.t_taskX)
			self.sent_text("t_taskY", self.papeDetailNN.t_taskY)
			self.sent_text("t_taskZ", self.papeDetailNN.t_taskZ)
			self.sent_text("t_taskBefore", self.papeDetailNN.t_taskBefore)
			self.sent_text("t_taskAfter", self.papeDetailNN.t_taskAfter)

			# parking:
			self.sent_text("t_tag", self.papeDetailNN.t_tag)
			self.sent_text("t_distance", self.papeDetailNN.t_distance)

			# toc do yeu cau:
			self.sent_text("t_requirSpeedX", self.papeDetailNN.t_requirSpeedX)
			self.sent_text("t_requirSpeedZ", self.papeDetailNN.t_requirSpeedZ)

			# ....
			self.sent_text("t_setpose", self.papeDetailNN.t_setpose)
			self.sent_text("t_reboot", self.papeDetailNN.t_reboot)
			self.sent_text("t_process", self.papeDetailNN.t_process)
			self.sent_text("t_funtion", self.papeDetailNN.t_funtion)

		elif (pape == self.pape_byHand):
			self.sent_infoGeneral()
			self.sent_text("t_tag", self.papeByHand.t_tag)

			spo = self.papeByHand.t_colorSetpose
			spo_cl = self.hmi_colorGreen
			if spo == 0:
				spo_cl = self.hmi_colorGray # dang cho
			elif spo == 1:
				spo_cl = self.hmi_colorYellow # dang setpose
			elif spo == 2:
				spo_cl = self.hmi_colorRed # loi
			self.sent_color("t_colorSetpose", spo_cl)

		elif (pape == self.pape_message):
			# pass
			self.draw_battery()
			self.sent_text("t_voltage", self.infoGeneral.t_voltage)

			self.sent_pic("p_typeError", self.papeMessage.p_typeError)
			self.sent_pic("p_methor", self.papeMessage.p_methor)
			self.sent_number("n_typeError", self.papeMessage.n_typeError)
			self.sent_text("t_phone", self.papeMessage.t_phone)

			self.sent_number("n_sw_y", self.papeMessage.n_sw_y)
			self.sent_number("n_sw_p", self.papeMessage.n_sw_p)
		# -- add new
		elif (pape == self.pape_password): # ok
			self.sent_text("t_password", self.papePassword.t_password)
			
			spo = self.papePassword.t_color
			spo_cl = self.hmi_colorGray
			if spo == 0:
				spo_cl = self.hmi_colorGray  # free
			elif spo == 1:
				spo_cl = self.hmi_colorGreen # right
			elif spo == 2:
				spo_cl = self.hmi_colorRed	 # correct
			self.sent_color("t_color", spo_cl)


	def run(self):
		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()):
			millis = int(round(time.time() * 1000))
			t = (millis - self.pre_time)
			if (t >= 1000/self.frequenceUpdate):
				self.pre_time = millis
				if (self.infoGeneral.pape != self.currentPape):
					print "sent next pape"
					self.currentPape = self.infoGeneral.pape
					self.nextPape(self.currentPape)

				self.sent_pape(self.currentPape)

			self.rate.sleep()
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
		nextion_thread = communicate_nextion(1)
		nextion_thread.start()

		# Keep the main thread running, otherwise signals are ignored.
		while not rospy.is_shutdown():
			nextion_thread.read_nextion()
			time.sleep(0.1)

	except ServiceExit:
		nextion_thread.shutdown_flag.set()
		nextion_thread.join()
	print('Exiting main program')
 
if __name__ == '__main__':
	main()
