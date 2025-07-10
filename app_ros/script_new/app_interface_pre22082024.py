#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Phùng Quý Dương
Company: STI Viet Nam
Date: 19/08/2024
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

sys.path.append('/home/stivietnam/catkin_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

import roslib
import rospy

from ros_canBus.msg import *
from sti_msgs.msg import *
from message_pkg.msg import *
from std_msgs.msg import Int16, Bool, Int8
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import * # QDialog, QApplication, QWidget
from PyQt5.QtGui import * # QPixmap
from PyQt5.QtCore import * # QTimer, QDateTime, Qt

import sqlite3

from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class statusButton:
	def __init__(self):
		self.bt_passAuto = 0
		self.bt_passHand = 0
		self.bt_cancelMission = 0
		self.bt_tryTarget_hide = 0
		self.bt_info = 0
		self.bt_setting = 0
		self.bt_clearError = 0

		self.bt_forwards = 0
		self.bt_backwards = 0
		self.bt_rotation_left = 0
		self.bt_rotation_right = 0
		self.bt_stop = 0

		self.bt_coorAverage = 0
		self.bt_disPointA = 0
		self.bt_disPointB = 0

		self.bt_chg_on = 0
		self.bt_chg_off = 0

		self.bt_spk_on = 0
		self.bt_spk_off = 0

		self.bt_disableBrake = 0

		self.bt_lift_up = 0
		self.bt_lift_down = 0
		self.bt_lift_reset = 0

		self.bt_hideSetting = 0
		# -
		self.bt_resetFrameWork = 0
		self.vs_speed = 50

		self.bt_tryTarget_start = 0
		self.bt_tryTarget_stop = 0
		self.bt_tryTarget_reset = 1
		self.ck_tryTarget_safety = 0
		
		self.bt_remote = 0
		self.soundtype = 0
		self.ledtype = 0

class statusColor:
	def __init__(self):
	# --
		self.lbc_safety_up = 0
		self.lbc_safety_ahead = 0
		self.lbc_safety_behind = 0
		# --
		self.cb_status = 0
		self.lbc_battery = 0

		# --
		self.lbc_button_clearError = 0
		self.lbc_button_power = 0

		self.lbc_blsock1 = 0
		self.lbc_emg1 = 0
		self.lbc_blsock2 = 0
		self.lbc_emg2 = 0
		self.lbc_safety_relay = 0

		# self.lbc_limit_up = 0
		# self.lbc_limit_down = 0
		# self.lbc_detect_lifter = 0

		self.lbc_limitAbove = 0
		self.lbc_limitBelow = 0
		self.lbc_checkTray = 0

		self.lbc_liftUp = 0
		self.lbc_liftDown = 0

		self.lbc_port_rtc = 0
		self.lbc_port_rs485 = 0
		self.lbc_port_nav350 = 0

		self.lbc_can_rtc = 0
		self.lbc_can_psu = 0
		self.lbc_can_mcu = 0
		self.lbc_can_hcu = 0
		self.lbc_can_oc = 0

		self.lbc_tryTarget_status = 0

		self.lbc_deletePose = 0
		self.lbc_addPose = 0
	
		# self. = 0
		# self. = 0
		# self. = 0
		# self. = 0
		# self. = 0

class valueLable:
	def __init__(self):
		self.modeRuning = 1

		self.lbv_name_agv = ''

		# - nuc info 
		self.lbv_ipWifi = ''
		self.lbv_mac = ''
		self.lbv_namePc = ''
		self.lbv_ipEthernet = ''
		self.lbv_cpu_usage = ''
		self.lbv_cpu_temp = ''
		self.lbv_ram = ''
		self.lbv_pingServer = ''
		self.lbv_wifiQuality = ''
		self.lbv_wifiSignal = ''
		self.lbv_runtime = ''

		self.lbv_battery = ''
		self.lbv_voltage = ''
		self.lbv_date = ''

		self.lbv_coordinates_x = ''
		self.lbv_coordinates_y = ''
		self.lbv_coordinates_r = ''

		self.lbv_numbeReflector = ''
		self.lbv_jobRuning = ''
		self.lbv_goalFollow_id = ''
		
		self.lbv_route_target = ''
		self.lbv_route_point0 = ''
		self.lbv_route_point1 = ''
		self.lbv_route_point2 = ''
		self.lbv_route_point3 = ''
		self.lbv_route_point4 = ''
		self.lbv_route_job1 = ''
		self.lbv_route_job2 = ''
		self.lbv_route_job1_mean = ''
		self.lbv_route_job2_mean = ''
		self.lbv_route_message = ''

		self.lbv_conveyorA = ''
		self.lbv_conveyorB = ''

		self.lbv_coorAverage_x = ''
		self.lbv_coorAverage_y = ''
		self.lbv_coorAverage_r = ''
		self.lbv_coorAverage_times = ''
		self.lbv_deltaDistance = ''

		self.lbv_velLeft = ''
		self.lbv_velRight = ''

		self.lbv_launhing = ''
		self.lbv_numberLaunch = ''
		self.percentLaunch = 0
		self.listError = ['A', 'B', 'C']
		self.listError_pre = []

		self.lbv_notification_driver1 = ''
		self.lbv_notification_driver2 = ''

		self.lbv_qualityWifi = 0
		# -
		self.list_logError = []
		self.list_logError_pre = []
		# --
		self.lbv_navi_job = ''
		self.lbv_navi_type = ''
		self.lbv_debug1 = ''
		self.lbv_debug2 = ''
		# -
		self.lbv_reflectorDetect = ''
		# -
		self.arrReflector = []
		self.angleCompare = 0
		
		self.lbv_tryTarget_x = 0
		self.lbv_tryTarget_y = 0
		self.lbv_tryTarget_r = 0
		self.lbv_tryTarget_d = 0
		self.lbv_tryTarget_id = 0

		# - 
		self.lbv_temp = ''

		# -
		self.list_SpecialPoint = []
		self.id_number = 0
		self.isUpdateList_SpecialPoint = True

class Reflector:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.localID = 0
		self.globalID = 0

class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		loadUi("/home/stivietnam/catkin_ws/src/app_ros/interface/app_ver2.ui", self)
		# --
		self.statusButton = statusButton()
		self.statusColor  = statusColor()
		self.valueLable   = valueLable()
		# -
		self.bt_controlConveyor_show.released.connect(self.released_controlConveyor_show)
		self.bt_controlConveyor_hide.released.connect(self.released_controlConveyor_hide)
		# -- -- -- 
		self.bt_exit.pressed.connect(self.out)
		# -
		self.bt_cancelMission.pressed.connect(self.pressed_cancelMission)
		self.bt_cancelMission.released.connect(self.released_cancelMission)
		# -
		self.bt_passHand.pressed.connect(self.pressed_passHand)
		self.bt_passHand.released.connect(self.released_passHand)
		# -
		self.bt_passAuto.pressed.connect(self.pressed_passAuto)
		self.bt_passAuto.released.connect(self.released_passAuto)
		# -
		self.bt_clearError.pressed.connect(self.pressed_clearError)
		self.bt_clearError.released.connect(self.released_clearError)
		# -- --
		self.bt_speaker_on.clicked.connect(self.clicked_speaker_on)
		self.bt_speaker_off.clicked.connect(self.clicked_speaker_off)
		# -
		self.bt_charger_on.clicked.connect(self.clicked_charger_on)
		self.bt_charger_off.clicked.connect(self.clicked_charger_off)
		# -- 
		self.bt_disableBrake_on.clicked.connect(self.clicked_brakeOn)
		self.bt_disableBrake_off.clicked.connect(self.clicked_brakeOff)

		# -- manual navigate agv
		self.bt_forwards.clicked.connect(self.clicked_forwards)
		self.bt_backwards.clicked.connect(self.clicked_backwards)
		# -
		self.bt_rotation_left.clicked.connect(self.clicked_rotation_left)
		self.bt_rotation_right.clicked.connect(self.clicked_rotation_right)
		# -
		self.bt_stop.clicked.connect(self.clicked_stop)

		self.bt_tryTarget_goUp.clicked.connect(self.clicked_tryTarget_goUp)
		self.bt_tryTarget_goDown.clicked.connect(self.clicked_tryTarget_goDown)
		# -
		self.bt_tryTarget_goLeft.clicked.connect(self.clicked_tryTarget_goLeft)
		self.bt_tryTarget_goRight.clicked.connect(self.clicked_tryTarget_goRight)
		# -
		self.bt_tryTarget_goStop.clicked.connect(self.clicked_tryTarget_goStop)

		self.bt_tryTarget_goUp2.clicked.connect(self.clicked_tryTarget_goUp)
		self.bt_tryTarget_goDown2.clicked.connect(self.clicked_tryTarget_goDown)
		# -
		self.bt_tryTarget_goLeft2.clicked.connect(self.clicked_tryTarget_goLeft)
		self.bt_tryTarget_goRight2.clicked.connect(self.clicked_tryTarget_goRight)
		# -
		self.bt_tryTarget_goStop2.clicked.connect(self.clicked_tryTarget_goStop2)

		self.bt_disableBrake1.clicked.connect(self.clicked_bt_disableBrake)
		self.bt_disableBrake2.clicked.connect(self.clicked_bt_disableBrake)

		# -- Setting devices
		self.bt_setting.pressed.connect(self.pressed_setting)
		self.bt_setting.released.connect(self.released_setting)
		self.bt_hideSetting1.clicked.connect(self.clicked_hideSetting)
		self.bt_hideSetting2.clicked.connect(self.clicked_hideSetting)
		self.bt_hideSetting3.clicked.connect(self.clicked_hideSetting)

		self.setting_status = 0
		self.timeSave_setting = rospy.Time.now()
		self.isShow_setting = 0
		self.status_show_setting = 0

		# self.bt_checkDevice.clicked.connect(self.clicked_bt_checkDevice)
		# self.status_checkDevice = 0

		self.bt_page_reflectorCheck.clicked.connect(self.clicked_page_reflectorCheck)
		self.bt_page_getPoint.clicked.connect(self.clicked_page_getPoint)
		self.bt_page_editSpecialPoint.clicked.connect(self.clicked_page_editSpecialPoint)

		self.bt_page_reflectorCheck2.clicked.connect(self.clicked_page_reflectorCheck)
		self.bt_page_getPoint2.clicked.connect(self.clicked_page_getPoint)
		self.bt_page_editSpecialPoint2.clicked.connect(self.clicked_page_editSpecialPoint)

		# self.bt_EditSpecialPoint.clicked.connect(self.clicked_bt_EditSpecialPoint)
		# self.bt_getWarehouse.clicked.connect(self.clicked_bt_getWarehouse)
		# self.bt_getWarehouse.setStyleSheet("background-color: blue;")
		self.fr_function_no = 0

		# self.bt_refresh_showRelector.pressed.connect(self.pressed_refresh_showRelector)

		# - Info
		self.bt_info.pressed.connect(self.pressed_info)
		self.bt_info.released.connect(self.released_info)
		self.info_status = 0
		self.timeSave_info = rospy.Time.now()
		self.isShow_info = 0

		self.bt_hideInfo.clicked.connect(self.clicked_hideInfo)
		# -
		self.bt_pw_cancel.clicked.connect(self.clicked_password_cancel)
		self.bt_pw_agree.clicked.connect(self.clicked_password_agree)
		self.password_data = ""
		self.password_right = "0111"
		self.bt_pw_0.clicked.connect(self.clicked_password_n0)
		self.bt_pw_1.clicked.connect(self.clicked_password_n1)
		self.bt_pw_2.clicked.connect(self.clicked_password_n2)
		self.bt_pw_3.clicked.connect(self.clicked_password_n3)
		self.bt_pw_4.clicked.connect(self.clicked_password_n4)
		self.bt_pw_5.clicked.connect(self.clicked_password_n5)
		self.bt_pw_6.clicked.connect(self.clicked_password_n6)
		self.bt_pw_7.clicked.connect(self.clicked_password_n7)
		self.bt_pw_8.clicked.connect(self.clicked_password_n8)
		self.bt_pw_9.clicked.connect(self.clicked_password_n9)
		self.bt_pw_clear.clicked.connect(self.clicked_password_clear)
		self.bt_pw_delete.clicked.connect(self.clicked_password_delete)

		# --
		self.bt_lift_up.pressed.connect(self.pressed_liftUp)
		self.bt_lift_down.pressed.connect(self.pressed_liftDown)
		self.bt_lift_reset.pressed.connect(self.pressed_liftReset)

		self.bt_lift_up.released.connect(self.released_liftUp)
		self.bt_lift_down.released.connect(self.released_liftDown)
		self.bt_lift_reset.released.connect(self.released_liftReset)

		# --
		self.bt_coorAverage.pressed.connect(self.pressed_bt_coorAverage)
		self.bt_coorAverage.released.connect(self.released_bt_coorAverage)

		self.bt_disPointA.clicked.connect(self.clicked_pointA)
		self.bt_disPointB.clicked.connect(self.clicked_pointB)

		# -- 
        # -- combo Box of speaker
		self.cb_listSpeak.addItem(" Tắt loa")
		self.cb_listSpeak.addItem(" Âm 1 - Bình Thường")
		self.cb_listSpeak.addItem(" Âm 2 - Cảnh báo")
		self.cb_listSpeak.addItem(" Âm 3 - Lỗi")

		self.cb_listLed.addItem(" Tắt led")
		self.cb_listLed.addItem(" Led 1 - Lỗi")
		self.cb_listLed.addItem(" Led 2 - Di Chuyển thường")
		self.cb_listLed.addItem(" Led 3 - Di Chuyển đặc biệt")
		self.cb_listLed.addItem(" Led 4 - Thao tác kệ")
		self.cb_listLed.addItem(" Led 5 - Hoàn thành")
		self.cb_listLed.addItem(" Led 6 - Lỗi vật cản")

		# -- Set speed manual
		self.bt_upSpeed.pressed.connect(self.pressed_upSpeed)
		self.bt_reduceSpeed.pressed.connect(self.pressed_reduceSpeed)

		self.bt_upSpeed1.pressed.connect(self.pressed_upSpeed)
		self.bt_reduceSpeed1.pressed.connect(self.pressed_reduceSpeed)

		self.bt_upSpeed2.pressed.connect(self.pressed_upSpeed)
		self.bt_reduceSpeed2.pressed.connect(self.pressed_reduceSpeed)

		# -- Reset FrameWork
		self.bt_resetFrameWork.pressed.connect(self.pressed_resetFrameWork)
		self.bt_resetFrameWork.released.connect(self.released_resetFrameWork)
		# -
		
		# ------------------------
		# self.bt_tryTarget_show.pressed.connect(self.pressed_tryTarget)
		# self.bt_tryTarget_show.released.connect(self.released_tryTarget)
		# -
		# self.bt_tryTarget_hide.pressed.connect(self.pressed_tryTarget_hide)
		# self.bt_tryTarget_hide.released.connect(self.released_tryTarget_hide)
		# -
		self.bt_tryTarget_up.pressed.connect(self.pressed_tryTarget_up)
		self.bt_tryTarget_up.released.connect(self.released_tryTarget_up)
		# # -
		self.bt_tryTarget_down.pressed.connect(self.pressed_tryTarget_down)
		self.bt_tryTarget_down.released.connect(self.released_tryTarget_down)
		# # -
		self.bt_tryTarget_reset.pressed.connect(self.pressed_tryTarget_reset)
		self.bt_tryTarget_reset.released.connect(self.released_tryTarget_reset)
		self.bt_tryTarget_reset.setStyleSheet("background-color: blue;")
		# # -
		self.bt_tryTarget_start.pressed.connect(self.pressed_tryTarget_start)
		self.bt_tryTarget_start.released.connect(self.released_tryTarget_start)
		# # -
		self.bt_tryTarget_stop.pressed.connect(self.pressed_tryTarget_stop)
		self.bt_tryTarget_stop.released.connect(self.released_tryTarget_stop)
		# -
		self.bt_tryTarget_x.pressed.connect(self.pressed_tryTarget_x)
		# -
		self.bt_tryTarget_y.pressed.connect(self.pressed_tryTarget_y)
		# -
		self.bt_tryTarget_r.pressed.connect(self.pressed_tryTarget_r)
		# -
		self.bt_tryTarget_d.pressed.connect(self.pressed_tryTarget_d)
		# -
		self.bt_tryTarget_id.pressed.connect(self.pressed_tryTarget_id)

		# -- Allow remote AGV from keyboard
		self.cb_remote.stateChanged.connect(self.checkbox_stateChanged)

		# - 
		self.chb_showDetail.stateChanged.connect(self.checkbox_showDetail_stateChanged)
		self.is_showDetail = 1

		self.bt_SaveSpecialPoint.clicked.connect(self.clicked_add_SpecialPoint)
		self.bt_DeleteSpecialPoint.clicked.connect(self.clicked_delete_SpecialPoint)

		self.flag_add_SpecialPoint = 0
		self.flag_delete_SpecialPoint = 0
				
		# -- -- -- Timer updata data
		# -- Fast
		timer_fast = QTimer(self)
		timer_fast.timeout.connect(self.process_fast)
		timer_fast.start(50)
		# -- normal
		timer_normal = QTimer(self)
		timer_normal.timeout.connect(self.process_normal)
		timer_normal.start(996)
		# -- Slow
		timer_slow = QTimer(self)
		timer_slow.timeout.connect(self.process_slow)
		timer_slow.start(3000)
		# --
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_byhand = 1
		self.modeRun_auto = 2
		self.modeRun_cancelMission = 5

		self.modeRuning = self.modeRun_launch
		# --

		self.password_data = ""
		# --
		self.timeSave_cancelMisson = rospy.Time.now()
		self.cancelMission_status = 0
		# -- 
		self.isShow_tryTarget = 0

		# -
		self.robotPoseNow = Pose()
		self.pointA = Point()
		self.pointB = Point()
		# -
		self.bt_coorAverage_status = 0
		# -
		self.countTime_coorAverage = 0
		self.total_x = 0.0
		self.total_y = 0.0
		self.total_angle = 0.0
		# - 
		self.isShow_moveHand = 1
		# - 
		self.flag_updateShowReflector = 0
		# self.show_reflector()

		self.changeNow = 0
		self.lbv_tryTarget_x.setText(str(self.valueLable.lbv_tryTarget_x))
		self.lbv_tryTarget_y.setText(str(self.valueLable.lbv_tryTarget_y))
		self.lbv_tryTarget_r.setText(str(self.valueLable.lbv_tryTarget_r))
		self.lbv_tryTarget_d.setText(str(self.valueLable.lbv_tryTarget_d))

		self.bt_lift_up.setStyleSheet("background-color: white;")
		self.bt_lift_down.setStyleSheet("background-color: white;")
		self.bt_lift_reset.setStyleSheet("background-color: blue;")

		self.fr_control.show()
		self.fr_setting.hide()
		self.fr_listTask.hide()
		self.fr_nav350.hide()
		self.fr_handMode_conveyor.hide()
		self.fr_handMode_move.show()
		self.fr_AGVamimation.hide()
		self.fr_listPoint.show()
		
	def process_fast(self):
		self.pb_speed.setValue(self.statusButton.vs_speed)
		self.pb_speed1.setValue(self.statusButton.vs_speed)
		self.pb_speed2.setValue(self.statusButton.vs_speed)

		# - combo box
		if (self.valueLable.listError != self.valueLable.listError_pre):
			self.valueLable.listError_pre = self.valueLable.listError
			self.cb_status.clear()
			lg = len(self.valueLable.listError) 
			for i in range(lg):
				self.cb_status.addItem(self.valueLable.listError[i])
		# -
		if (self.valueLable.list_logError != self.valueLable.list_logError_pre):
			self.valueLable.list_logError_pre = self.valueLable.list_logError
			#self.cb_logError.clear()
			lg = len(self.valueLable.list_logError) 
			#for i in range(lg):
				#self.cb_logError.addItem(self.valueLable.list_logError[i])
		# -
		if self.valueLable.isUpdateList_SpecialPoint == True:
			# print("Update combo box")
			self.valueLable.isUpdateList_SpecialPoint = False
			self.cb_listSpecialPoint.clear()
			self.cb_listSpecialPoint.addItems(self.valueLable.list_SpecialPoint)

		# - 
		if self.isShow_setting == 1 and self.status_show_setting == 1:
			self.statusButton.bt_setting = 1
		else:
			self.statusButton.bt_setting = 0

		# -- 
		self.coorAverage_run()
		# # --
		self.set_labelValue()
		# # --
		self.set_labelColor()
		# # --
		self.controlShow_followMode()
		# --
		self.statusButton.ck_tryTarget_safety = self.ck_tryTarget_safety.isChecked()

		# -- show check devices
		if (self.setting_status == 1):
			delta_t = rospy.Time.now() - self.timeSave_setting
			# -- Chi kich hoat khi dang o che do Bang Tay.
			if (delta_t.to_sec() > 1) and self.valueLable.modeRuning == 1:
				self.isShow_setting = 1
				self.password_data = ""
		else:
			self.timeSave_setting = rospy.Time.now()
		
		# -- show info page
		if self.info_status == 1:
			delta_t = rospy.Time.now() - self.timeSave_info

			if delta_t.to_sec() > 1.5 and self.valueLable.modeRuning == 1:
				self.isShow_info = 1
				self.password_data = ""
		else: 
			self.timeSave_info = rospy.Time.now()

		# -- add 21/01/2022 - show cancelMission
		if (self.cancelMission_status == 1):
			delta_c = rospy.Time.now() - self.timeSave_cancelMisson
			if (delta_c.to_sec() > 0.5):
				self.fr_agv.hide()
				self.fr_password.show()
				self.clicked_stop()
		else:
			self.timeSave_cancelMisson = rospy.Time.now()

		# --
		self.show_password()
		# --

		if self.flag_updateShowReflector == 1:
			self.flag_updateShowReflector = 0
			length = len(self.valueLable.arrReflector)
			# print ("length: ", length)

			self.show_reflector()

		# -- Reset lift Up/ Down button
		if self.statusColor.lbc_liftUp == 1 and self.statusColor.lbc_liftDown == 0:
			self.bt_lift_up.setStyleSheet("background-color: blue;")
			self.bt_lift_down.setStyleSheet("background-color: white;")
			self.bt_lift_reset.setStyleSheet("background-color: white;")

		elif self.statusColor.lbc_liftUp == 0 and self.statusColor.lbc_liftDown == 1:
			self.bt_lift_up.setStyleSheet("background-color: white;")
			self.bt_lift_down.setStyleSheet("background-color: blue;")
			self.bt_lift_reset.setStyleSheet("background-color: white;")

		elif self.statusColor.lbc_liftUp == 0 and self.statusColor.lbc_liftDown == 0:
			self.bt_lift_up.setStyleSheet("background-color: white;")
			self.bt_lift_down.setStyleSheet("background-color: white;")
			self.bt_lift_reset.setStyleSheet("background-color: blue;")

		elif self.statusColor.lbc_liftUp == 1 and self.statusColor.lbc_liftDown == 1:
			self.bt_lift_up.setStyleSheet("background-color: white;")
			self.bt_lift_down.setStyleSheet("background-color: white;")
			self.bt_lift_reset.setStyleSheet("background-color: blue;")

		# - cb of list speak
		self.statusButton.soundtype = self.cb_listSpeak.currentIndex()

		# - cb of list led
		self.statusButton.ledtype = self.cb_listLed.currentIndex()

		# -
		if self.statusButton.bt_disableBrake == 1:
			self.bt_disableBrake1.setStyleSheet("background-color: blue;")
			self.bt_disableBrake2.setStyleSheet("background-color: blue;")

		else:
			self.bt_disableBrake1.setStyleSheet("background-color: white;")
			self.bt_disableBrake2.setStyleSheet("background-color: white;")
	# -
	def show_reflector(self):
		# --
		# self.lb_nav350.setText("NAV")
		# self.lb_nav350.move(400, 250)
		# self.lb_nav350.setStyleSheet("border: 1px solid blue; ")
		# self.lb_nav350.setAlignment(Qt.AlignCenter)
		# self.lb_nav350.setFont(QFont('Arial', 8))
		# self.lb_nav350.setParent(self.fr_showReflector)

		self.valueLable.angleCompare = self.dial_angleCompare.value()
		# --
		self.lb_rf_0.move(0, 0)
		self.lb_rf_1.move(0, 20)
		self.lb_rf_2.move(0, 40)
		self.lb_rf_3.move(0, 60)
		self.lb_rf_4.move(0, 80)
		self.lb_rf_5.move(0, 100)
		self.lb_rf_6.move(0, 120)
		self.lb_rf_7.move(0, 140)
		self.lb_rf_8.move(0, 160)
		self.lb_rf_9.move(0, 180)
		self.lb_rf_10.move(0, 200)
		self.lb_rf_11.move(0, 220)
		# --

		length = len(self.valueLable.arrReflector)
		if length > 0:
			self.lb_rf_0.setText(str(self.valueLable.arrReflector[0].globalID)) # localID
			self.lb_rf_0.move(self.valueLable.arrReflector[0].x, self.valueLable.arrReflector[0].y)
			if self.valueLable.arrReflector[0].globalID == '-1':
				self.lb_rf_0.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_0.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 1:
			self.lb_rf_1.setText(str(self.valueLable.arrReflector[1].globalID))
			self.lb_rf_1.move(self.valueLable.arrReflector[1].x, self.valueLable.arrReflector[1].y)
			if self.valueLable.arrReflector[1].globalID == '-1':
				self.lb_rf_1.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_1.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 2:
			self.lb_rf_2.setText(str(self.valueLable.arrReflector[2].globalID))
			self.lb_rf_2.move(self.valueLable.arrReflector[2].x, self.valueLable.arrReflector[2].y)
			if self.valueLable.arrReflector[2].globalID == '-1':
				self.lb_rf_2.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_2.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 3:
			self.lb_rf_3.setText(str(self.valueLable.arrReflector[3].globalID))
			self.lb_rf_3.move(self.valueLable.arrReflector[3].x, self.valueLable.arrReflector[3].y)
			if self.valueLable.arrReflector[3].globalID == '-1':
				self.lb_rf_3.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_3.setStyleSheet("border: 1px solid green; border-radius: 10px")


		if length > 4:
			self.lb_rf_4.setText(str(self.valueLable.arrReflector[4].globalID))
			self.lb_rf_4.move(self.valueLable.arrReflector[4].x, self.valueLable.arrReflector[4].y)
			if self.valueLable.arrReflector[4].globalID == '-1':
				self.lb_rf_4.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_4.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 5:
			self.lb_rf_5.setText(str(self.valueLable.arrReflector[5].globalID))
			self.lb_rf_5.move(self.valueLable.arrReflector[5].x, self.valueLable.arrReflector[5].y)
			if self.valueLable.arrReflector[5].globalID == '-1':
				self.lb_rf_5.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_5.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 6:
			self.lb_rf_6.setText(str(self.valueLable.arrReflector[6].globalID))
			self.lb_rf_6.move(self.valueLable.arrReflector[6].x, self.valueLable.arrReflector[6].y)
			if self.valueLable.arrReflector[6].globalID == '-1':
				self.lb_rf_6.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_6.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 7:
			self.lb_rf_7.setText(str(self.valueLable.arrReflector[7].globalID))
			self.lb_rf_7.move(self.valueLable.arrReflector[7].x, self.valueLable.arrReflector[7].y)
			if self.valueLable.arrReflector[7].globalID == '-1':
				self.lb_rf_7.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_7.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 8:
			self.lb_rf_8.setText(str(self.valueLable.arrReflector[8].globalID))
			self.lb_rf_8.move(self.valueLable.arrReflector[8].x, self.valueLable.arrReflector[8].y)
			if self.valueLable.arrReflector[8].globalID == '-1':
				self.lb_rf_8.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_8.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 9:
			self.lb_rf_9.setText(str(self.valueLable.arrReflector[9].globalID))
			self.lb_rf_9.move(self.valueLable.arrReflector[9].x, self.valueLable.arrReflector[9].y)
			if self.valueLable.arrReflector[9].globalID == '-1':
				self.lb_rf_9.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_9.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 10:
			self.lb_rf_10.setText(str(self.valueLable.arrReflector[10].globalID))
			self.lb_rf_10.move(self.valueLable.arrReflector[10].x, self.valueLable.arrReflector[10].y)
			if self.valueLable.arrReflector[10].globalID == '-1':
				self.lb_rf_10.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_10.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 11:
			self.lb_rf_11.setText(str(self.valueLable.arrReflector[11].globalID))
			self.lb_rf_11.move(self.valueLable.arrReflector[11].x, self.valueLable.arrReflector[11].y)
			if self.valueLable.arrReflector[11].globalID == '-1':
				self.lb_rf_11.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_11.setStyleSheet("border: 1px solid green; border-radius: 10px")

	# -
	def released_controlConveyor_show(self):
		self.isShow_moveHand = 0
		# -
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 1
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: blue;")

	def released_controlConveyor_hide(self):
		self.isShow_moveHand = 1

	# -
	def pressed_cancelMission(self):
		self.bt_cancelMission.setStyleSheet("background-color: blue;")
		self.cancelMission_status = 1

	def released_cancelMission(self):
		self.bt_cancelMission.setStyleSheet("background-color: white;")
		self.cancelMission_status = 0
		self.isShow_setting = 0
	# -
	def pressed_passHand(self):
		self.statusButton.bt_passHand = 1
		self.bt_passHand.setStyleSheet("background-color: blue;")
		
	def released_passHand(self):
		self.statusButton.bt_passHand = 0
		self.bt_passHand.setStyleSheet("background-color: white;")
		self.isShow_setting = 0
	# -
	def pressed_passAuto(self):
		self.statusButton.bt_passAuto = 1
		self.clicked_stop()
		# --
		self.statusButton.bt_disableBrake = 0
		self.bt_disableBrake_off.setStyleSheet("background-color: blue;")
		self.bt_disableBrake_on.setStyleSheet("background-color: white;")

	def released_passAuto(self):
		self.statusButton.bt_passAuto = 0
		self.isShow_setting = 0

	# -
	def pressed_clearError(self):
		self.statusButton.bt_clearError = 1
		self.bt_clearError.setStyleSheet("background-color: blue;")
		self.clicked_stop()

	def released_clearError(self):
		self.statusButton.bt_clearError = 0
		self.bt_clearError.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# -- 
	def clicked_speaker_on(self):
		self.statusButton.bt_spk_on = 1
		self.statusButton.bt_spk_off = 0
		self.bt_speaker_on.setStyleSheet("background-color: blue;")
		self.bt_speaker_off.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_speaker_off(self):
		self.statusButton.bt_spk_on = 0
		self.statusButton.bt_spk_off = 1
		self.bt_speaker_off.setStyleSheet("background-color: blue;")
		self.bt_speaker_on.setStyleSheet("background-color: white;")
		self.clicked_stop()

	# - 
	def clicked_charger_on(self):
		self.statusButton.bt_chg_on = 1
		self.statusButton.bt_chg_off = 0
		self.bt_charger_on.setStyleSheet("background-color: blue;")
		self.bt_charger_off.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_charger_off(self):
		self.statusButton.bt_chg_on = 0
		self.statusButton.bt_chg_off = 1
		self.bt_charger_off.setStyleSheet("background-color: blue;")
		self.bt_charger_on.setStyleSheet("background-color: white;")
		self.clicked_stop()

	# -- OK --
	def clicked_brakeOn(self):
		self.statusButton.bt_disableBrake = 1
		self.bt_disableBrake_on.setStyleSheet("background-color: blue;")
		self.bt_disableBrake_off.setStyleSheet("background-color: white;")
		self.clicked_stop()

	def clicked_brakeOff(self):
		self.statusButton.bt_disableBrake = 0
		self.bt_disableBrake_off.setStyleSheet("background-color: blue;")
		self.bt_disableBrake_on.setStyleSheet("background-color: white;")
		self.clicked_stop()

	def clicked_bt_disableBrake(self):
		self.statusButton.bt_disableBrake = not self.statusButton.bt_disableBrake
		self.clicked_stop()

	# --
	def clicked_forwards(self):
		self.statusButton.bt_forwards = 1
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 0
		self.bt_forwards.setStyleSheet("background-color: blue;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_backwards(self):
		self.statusButton.bt_backwards = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 0
		self.bt_backwards.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_rotation_left(self):
		self.statusButton.bt_rotation_left = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 0
		self.bt_rotation_left.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_rotation_right(self):
		self.statusButton.bt_rotation_right = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_stop = 0
		self.bt_rotation_right.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_stop(self):
		self.statusButton.bt_stop = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_backwards = 0
  
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: blue;")

	def clicked_tryTarget_goUp(self):
		self.statusButton.bt_forwards = 1
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 0
		self.bt_tryTarget_goUp.setStyleSheet("background-color: blue;")
		self.bt_tryTarget_goDown.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goLeft.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goRight.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goStop.setStyleSheet("background-color: white;")
	# - 
	def clicked_tryTarget_goDown(self):
		self.statusButton.bt_backwards = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 0
		self.bt_tryTarget_goUp.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goDown.setStyleSheet("background-color: blue;")
		self.bt_tryTarget_goLeft.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goRight.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goStop.setStyleSheet("background-color: white;")
	# - 
	def clicked_tryTarget_goLeft(self):
		self.statusButton.bt_rotation_left = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 0
		self.bt_tryTarget_goUp.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goDown.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goLeft.setStyleSheet("background-color: blue;")
		self.bt_tryTarget_goRight.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goStop.setStyleSheet("background-color: white;")
	# - 
	def clicked_tryTarget_goRight(self):
		self.statusButton.bt_rotation_right = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_stop = 0
		self.bt_tryTarget_goUp.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goDown.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goLeft.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goRight.setStyleSheet("background-color: blue;")
		self.bt_tryTarget_goStop.setStyleSheet("background-color: white;")
	# - 
	def clicked_tryTarget_goStop(self):
		self.statusButton.bt_stop = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_backwards = 0
  
		self.bt_tryTarget_goUp.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goDown.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goLeft.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goRight.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goStop.setStyleSheet("background-color: blue;")

	def clicked_tryTarget_goStop2(self):
		self.statusButton.bt_stop = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_backwards = 0
  
		self.bt_tryTarget_goUp.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goDown.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goLeft.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goRight.setStyleSheet("background-color: white;")
		self.bt_tryTarget_goStop.setStyleSheet("background-color: blue;")
		self.flag_updateShowReflector = 1

	def clicked_add_SpecialPoint(self):
		self.flag_add_SpecialPoint = 1
		self.bt_SaveSpecialPoint.setStyleSheet("background-color: blue;")

	def clicked_delete_SpecialPoint(self):
		self.flag_delete_SpecialPoint = 1
		self.bt_DeleteSpecialPoint.setStyleSheet("background-color: blue;")

	# --  --
	def clicked_password_agree(self):
		self.fr_agv.show()
		self.fr_password.hide()
		self.statusButton.bt_cancelMission = 1
		self.password_data = ""
		self.status_show_setting = 1
		self.isShow_info = 0

	def clicked_password_cancel(self):
		self.fr_agv.show()
		self.fr_password.hide()
		self.password_data = ""
		self.status_show_setting = 0
		self.isShow_setting = 0
		self.isShow_info = 0
	
	def process_normal(self):
		self.set_dateTime()
		# self.lbv_ip.setText(self.valueLable.lbv_ip)
		# self.lbv_name_agv.setText(self.valueLable.lbv_name_agv)
		# self.lbv_mac.setText(self.valueLable.lbv_mac)
		# self.lbv_namePc.setText(self.valueLable.lbv_namePc)

	def process_slow(self):
		self.set_valueBattery(self.valueLable.lbv_battery)

	def pressed_setting(self):
		self.bt_setting.setStyleSheet("background-color: blue;")	
		self.setting_status = 1
		# self.info_status = 0
		# self.isShow_info = 0

	def released_setting(self):
		self.bt_setting.setStyleSheet("background-color: white;")	
		self.setting_status = 0
		# self.info_status = 0
		# self.isShow_info = 0

	def clicked_hideSetting(self):
		self.isShow_setting = 0
		self.status_show_setting = 0
		self.fr_function_no = 0

	def clicked_page_getPoint(self):
		self.fr_function_no = 0

	def clicked_page_reflectorCheck(self):
		self.fr_function_no = 1

	def clicked_page_editSpecialPoint(self):
		self.fr_function_no = 2

	# def clicked_bt_checkDevice(self):
	# 	self.status_checkDevice = not self.status_checkDevice
	# 	if self.status_checkDevice == 1:
	# 		self.bt_checkDevice.setStyleSheet("background-color: blue;")
	# 	else:
	# 		self.bt_checkDevice.setStyleSheet("background-color: white;")

	def pressed_info(self):
		self.bt_info.setStyleSheet("background-color: blue;")
		self.info_status = 1

	def released_info(self):
		self.bt_info.setStyleSheet("background-color: white;")
		self.info_status = 0

	def clicked_hideInfo(self):
		self.isShow_info = 0

	# - 
	def clicked_password_n0(self):
		if (len(self.password_data) < 4):
			self.password_data += "0"

	def clicked_password_n1(self):
		if (len(self.password_data) < 4):
			self.password_data += "1"

	def clicked_password_n2(self):
		if (len(self.password_data) < 4):
			self.password_data += "2"

	def clicked_password_n3(self):
		if (len(self.password_data) < 4):
			self.password_data += "3"

	def clicked_password_n4(self):
		if (len(self.password_data) < 4):
			self.password_data += "4"

	def clicked_password_n5(self):
		if (len(self.password_data) < 4):
			self.password_data += "5"

	def clicked_password_n6(self):
		if (len(self.password_data) < 4):
			self.password_data += "6"

	def clicked_password_n7(self):
		if (len(self.password_data) < 4):
			self.password_data += "7"

	def clicked_password_n8(self):
		if (len(self.password_data) < 4):
			self.password_data += "8"

	def clicked_password_n9(self):
		if (len(self.password_data) < 4):
			self.password_data += "9"

	def clicked_password_clear(self):
		self.password_data = ""

	def clicked_password_delete(self):
		lenght = len(self.password_data)
		if (lenght > 0):
			self.password_data = self.password_data[:-1]

	def pressed_bt_coorAverage(self):
		self.bt_coorAverage.setStyleSheet("background-color: blue;")
		self.bt_coorAverage_status = 1

	def released_bt_coorAverage(self):
		self.bt_coorAverage.setStyleSheet("background-color: white;")
		self.bt_coorAverage_status = 0

	def clicked_pointA(self):
		self.pointA.x = self.robotPoseNow.position.x
		self.pointA.y = self.robotPoseNow.position.y
		self.lbv_deltaDistance.setText("---")

	def clicked_pointB(self):
		self.pointB.x = self.robotPoseNow.position.x
		self.pointB.y = self.robotPoseNow.position.y
		delta_distance = self.calculate_distance(self.pointA, self.pointB)
		self.lbv_deltaDistance.setText(str(round(delta_distance, 3)))
		if self.ck_linkOffset.isChecked() == 1:
			self.valueLable.lbv_tryTarget_d = delta_distance
			self.lbv_tryTarget_d.setText(str(round(self.valueLable.lbv_tryTarget_d, 3)))

	def pressed_resetFrameWork(self):
		self.bt_resetFrameWork.setStyleSheet("background-color: blue;")
		self.statusButton.bt_resetFrameWork = 1

	def released_resetFrameWork(self):
		# self.bt_resetFrameWork.setStyleSheet("background-color: white;")
		self.statusButton.bt_resetFrameWork = 0

	def pressed_upSpeed(self):
		self.statusButton.vs_speed += 10
		if self.statusButton.vs_speed >= 100:
			self.statusButton.vs_speed = 100

	def pressed_reduceSpeed(self):
		self.statusButton.vs_speed -= 10
		if self.statusButton.vs_speed < 5:
			self.statusButton.vs_speed = 5

	def pressed_liftUp(self):
		self.statusButton.bt_lift_up = 1
		self.statusButton.bt_lift_down = 0

	def pressed_liftDown(self):
		self.statusButton.bt_lift_up = 0
		self.statusButton.bt_lift_down = 1

	def pressed_liftReset(self):
		# self.bt_lift_up.setStyleSheet("background-color: white;")
		# self.bt_lift_down.setStyleSheet("background-color: white;")
		# self.bt_lift_reset.setStyleSheet("background-color: blue;")
		self.statusButton.bt_lift_reset = 1

	# -- update 29/05/2024
	def released_liftUp(self):
		# self.bt_lift_up.setStyleSheet("background-color: blue;")
		# self.bt_lift_down.setStyleSheet("background-color: white;")
		# self.bt_lift_reset.setStyleSheet("background-color: white;")
		self.statusButton.bt_lift_up = 0

	def released_liftDown(self):
		# self.bt_lift_up.setStyleSheet("background-color: white;")
		# self.bt_lift_down.setStyleSheet("background-color: blue;")
		# self.bt_lift_reset.setStyleSheet("background-color: white;")
		self.statusButton.bt_lift_down = 0

	def released_liftReset(self):
		self.statusButton.bt_lift_reset = 0

	# def pressed_refresh_showRelector(self):
	# 	self.flag_updateShowReflector = 1

	# --
	def pressed_tryTarget(self):
		# self.bt_tryTarget_show.setStyleSheet("background-color: blue;")
		self.isShow_tryTarget = 1
		self.clicked_stop()

	def released_tryTarget(self):
		# self.bt_tryTarget_show.setStyleSheet("background-color: white;")
		self.clicked_stop()

	# --
	# def pressed_tryTarget_hide(self):
	# 	# self.bt_tryTarget_show.setStyleSheet("background-color: blue;")
	# 	self.clicked_stop()

	# def released_tryTarget_hide(self):
	# 	# self.bt_tryTarget_show.setStyleSheet("background-color: white;")
	# 	self.isShow_tryTarget = 0
	# 	self.clicked_stop()
	# 	self.statusButton.bt_tryTarget_start = 0
	# 	self.statusButton.bt_tryTarget_stop = 0
	# 	self.statusButton.bt_tryTarget_reset = 1
	# 	self.bt_tryTarget_reset.setStyleSheet("background-color: blue;")
	# 	self.bt_tryTarget_stop.setStyleSheet("background-color: white;")
	# 	self.bt_tryTarget_start.setStyleSheet("background-color: white;")

	# --
	def pressed_tryTarget_up(self):
		self.bt_tryTarget_up.setStyleSheet("background-color: blue;")
		if self.cb_unit.currentText() != '':
			val_change = float(self.cb_unit.currentText())
			# - X
			if self.changeNow == 1:
				self.valueLable.lbv_tryTarget_x += val_change
			# - Y
			if self.changeNow == 2:
				self.valueLable.lbv_tryTarget_y += val_change
			# - R
			if self.changeNow == 3:
				self.valueLable.lbv_tryTarget_r += val_change
			# - D
			if self.changeNow == 4:
				self.valueLable.lbv_tryTarget_d += val_change
			# - ID
			if self.changeNow == 5:
				self.valueLable.lbv_tryTarget_id += val_change
				self.valueLable.lbv_tryTarget_id = int(self.valueLable.lbv_tryTarget_id)			

	def released_tryTarget_up(self):
		self.bt_tryTarget_up.setStyleSheet("background-color: white;")
		self.lbv_tryTarget_x.setText(str(round(self.valueLable.lbv_tryTarget_x, 3)))
		self.lbv_tryTarget_y.setText(str(round(self.valueLable.lbv_tryTarget_y, 3)))
		self.lbv_tryTarget_r.setText(str(round(self.valueLable.lbv_tryTarget_r, 3)))
		self.lbv_tryTarget_d.setText(str(round(self.valueLable.lbv_tryTarget_d, 2)))
		self.lbv_tryTarget_id.setText(str(self.valueLable.lbv_tryTarget_id))

	# --
	def pressed_tryTarget_down(self):
		self.bt_tryTarget_down.setStyleSheet("background-color: blue;")
		val_change = float(self.cb_unit.currentText())
		# - X
		if self.changeNow == 1:
			self.valueLable.lbv_tryTarget_x -= val_change
		# - Y
		if self.changeNow == 2:
			self.valueLable.lbv_tryTarget_y -= val_change
		# - R
		if self.changeNow == 3:
			self.valueLable.lbv_tryTarget_r -= val_change
		# - D
		if self.changeNow == 4:
			self.valueLable.lbv_tryTarget_d -= val_change
		# - ID
		if self.changeNow == 5:
			self.valueLable.lbv_tryTarget_id -= val_change
			if self.valueLable.lbv_tryTarget_id <= 0:
				self.valueLable.lbv_tryTarget_id = 0

			self.valueLable.lbv_tryTarget_id = int(self.valueLable.lbv_tryTarget_id)

	def released_tryTarget_down(self):
		self.bt_tryTarget_down.setStyleSheet("background-color: white;")
		self.lbv_tryTarget_x.setText(str(round(self.valueLable.lbv_tryTarget_x, 3)))
		self.lbv_tryTarget_y.setText(str(round(self.valueLable.lbv_tryTarget_y, 3)))
		self.lbv_tryTarget_r.setText(str(round(self.valueLable.lbv_tryTarget_r, 3)))
		self.lbv_tryTarget_d.setText(str(round(self.valueLable.lbv_tryTarget_d, 2)))
		self.lbv_tryTarget_id.setText(str(self.valueLable.lbv_tryTarget_id))

	# --
	def pressed_tryTarget_reset(self):
		self.bt_tryTarget_reset.setStyleSheet("background-color: blue;")
		self.statusButton.bt_tryTarget_reset = 1

	def released_tryTarget_reset(self):
		self.bt_tryTarget_start.setStyleSheet("background-color: white;")
		self.bt_tryTarget_stop.setStyleSheet("background-color: white;")
		self.statusButton.bt_tryTarget_start = 0
		self.statusButton.bt_tryTarget_stop = 0

		self.bt_tryTarget_goUp.setEnabled(True)
		self.bt_tryTarget_goDown.setEnabled(True)
		self.bt_tryTarget_goStop.setEnabled(True)
		self.bt_tryTarget_goLeft.setEnabled(True)
		self.bt_tryTarget_goRight.setEnabled(True)

	# --
	def pressed_tryTarget_start(self):
		self.bt_tryTarget_start.setStyleSheet("background-color: blue;")
		self.statusButton.bt_tryTarget_start = 1


	def released_tryTarget_start(self):
		self.bt_tryTarget_reset.setStyleSheet("background-color: white;")
		self.bt_tryTarget_stop.setStyleSheet("background-color: white;")
		self.statusButton.bt_tryTarget_reset = 0
		self.statusButton.bt_tryTarget_stop = 0

		self.bt_tryTarget_goUp.setEnabled(False)
		self.bt_tryTarget_goDown.setEnabled(False)
		self.bt_tryTarget_goStop.setEnabled(False)
		self.bt_tryTarget_goLeft.setEnabled(False)
		self.bt_tryTarget_goRight.setEnabled(False)

	# --
	def pressed_tryTarget_stop(self):
		self.bt_tryTarget_stop.setStyleSheet("background-color: blue;")
		self.statusButton.bt_tryTarget_stop = 1

	def released_tryTarget_stop(self):
		self.bt_tryTarget_reset.setStyleSheet("background-color: white;")
		self.bt_tryTarget_start.setStyleSheet("background-color: white;")
		self.statusButton.bt_tryTarget_reset = 0
		self.statusButton.bt_tryTarget_start = 0

		self.bt_tryTarget_goUp.setEnabled(False)
		self.bt_tryTarget_goDown.setEnabled(False)
		self.bt_tryTarget_goStop.setEnabled(False)
		self.bt_tryTarget_goLeft.setEnabled(False)
		self.bt_tryTarget_goRight.setEnabled(False)

	# --
	def pressed_tryTarget_x(self):
		self.bt_tryTarget_x.setStyleSheet("background-color: blue;")
		self.bt_tryTarget_y.setStyleSheet("background-color: white;")
		self.bt_tryTarget_r.setStyleSheet("background-color: white;")
		self.bt_tryTarget_d.setStyleSheet("background-color: white;")
		self.bt_tryTarget_id.setStyleSheet("background-color: white;")
		self.show_combox_unitMeter()
		self.changeNow = 1

	def pressed_tryTarget_y(self):
		self.bt_tryTarget_x.setStyleSheet("background-color: white;")
		self.bt_tryTarget_y.setStyleSheet("background-color: blue;")
		self.bt_tryTarget_r.setStyleSheet("background-color: white;")
		self.bt_tryTarget_d.setStyleSheet("background-color: white;")
		self.bt_tryTarget_id.setStyleSheet("background-color: white;")
		self.show_combox_unitMeter()
		self.changeNow = 2

	def pressed_tryTarget_r(self):
		self.bt_tryTarget_x.setStyleSheet("background-color: white;")
		self.bt_tryTarget_y.setStyleSheet("background-color: white;")
		self.bt_tryTarget_r.setStyleSheet("background-color: blue;")
		self.bt_tryTarget_d.setStyleSheet("background-color: white;")
		self.bt_tryTarget_id.setStyleSheet("background-color: white;")
		self.show_combox_unitDegree()
		self.changeNow = 3

	def pressed_tryTarget_d(self):
		self.bt_tryTarget_x.setStyleSheet("background-color: white;")
		self.bt_tryTarget_y.setStyleSheet("background-color: white;")
		self.bt_tryTarget_r.setStyleSheet("background-color: white;")
		self.bt_tryTarget_d.setStyleSheet("background-color: blue;")
		self.bt_tryTarget_id.setStyleSheet("background-color: white;")
		self.show_combox_unitMeter()
		self.changeNow = 4

	def pressed_tryTarget_id(self):
		self.bt_tryTarget_x.setStyleSheet("background-color: white;")
		self.bt_tryTarget_y.setStyleSheet("background-color: white;")
		self.bt_tryTarget_r.setStyleSheet("background-color: white;")
		self.bt_tryTarget_d.setStyleSheet("background-color: white;")
		self.bt_tryTarget_id.setStyleSheet("background-color: blue;")
		self.show_combox_Interger()
		self.changeNow = 5

	# -
	def checkbox_stateChanged(self):
		if self.cb_remote.isChecked():
			self.statusButton.bt_remote = 1
		else:
			self.statusButton.bt_remote = 0
	
	def checkbox_showDetail_stateChanged(self):
		if self.chb_showDetail.isChecked():
			self.is_showDetail = 1
		else:
			self.is_showDetail = 0
			
	def show_combox_unitMeter(self):
		self.cb_unit.clear()
		self.lbv_unit.setText("m")
		self.cb_unit.addItem("0.001")
		self.cb_unit.addItem("0.002")
		self.cb_unit.addItem("0.005")
		self.cb_unit.addItem("0.01")
		self.cb_unit.addItem("0.1")
		self.cb_unit.addItem("1")
		self.cb_unit.addItem("2")
		self.cb_unit.addItem("5")

	def show_combox_unitDegree(self):
		self.cb_unit.clear()
		self.lbv_unit.setText("Độ")
		self.cb_unit.addItem("0.01")
		self.cb_unit.addItem("0.1")
		self.cb_unit.addItem("0.2")
		self.cb_unit.addItem("0.5")
		self.cb_unit.addItem("1")
		self.cb_unit.addItem("2")
		self.cb_unit.addItem("5")

	def show_combox_Interger(self):
		self.cb_unit.clear()
		self.lbv_unit.setText("")
		self.cb_unit.addItem("1")
		self.cb_unit.addItem("2")
		self.cb_unit.addItem("5")

	def out(self):
		QApplication.quit()
		print('out')

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	# ---------------------------
	def set_dateTime(self):
		time_now = datetime.now()
		# dd/mm/YY H:M:S
		dt_string = time_now.strftime("%d/%m/%Y\n%H:%M:%S")
		self.lbv_date.setText(dt_string)

	def set_valueBattery(self, str_value): # str_value: string()
		self.lbv_battery.setText(str_value)
	
	def set_labelColor(self):
		# ---- Safety
		if (self.statusColor.lbc_safety_up == 0):
			self.lbc_safety0.setStyleSheet("background-color: green; color: white;")
			self.lbc_safety_up.setStyleSheet("background-color: green; color: white")

		elif (self.statusColor.lbc_safety_up == 1):
			self.lbc_safety0.setStyleSheet("background-color: red; color: white")
			self.lbc_safety_up.setStyleSheet("background-color: red; color: white")
		else:
			self.lbc_safety0.setStyleSheet("background-color: yellow;")
			self.lbc_safety_up.setStyleSheet("background-color: yellow;")
		# -
		if (self.statusColor.lbc_safety_ahead == 0):
			self.lbc_safety1.setStyleSheet("background-color: green; color: white")
			self.lbc_safety_ahead.setStyleSheet("background-color: green; color: white")

		elif (self.statusColor.lbc_safety_ahead == 1):
			self.lbc_safety1.setStyleSheet("background-color: red; color: white")
			self.lbc_safety_ahead.setStyleSheet("background-color: red; color: white")
		else:
			self.lbc_safety1.setStyleSheet("background-color: yellow;")
			self.lbc_safety_ahead.setStyleSheet("background-color: yellow;")
		# -
		if (self.statusColor.lbc_safety_behind == 0):
			self.lbc_safety2.setStyleSheet("background-color: green; color: white")
			self.lbc_safety_behind.setStyleSheet("background-color: green; color: white")

		elif (self.statusColor.lbc_safety_behind == 1):
			self.lbc_safety2.setStyleSheet("background-color: red; color: white")
			self.lbc_safety_behind.setStyleSheet("background-color: red; color: white")
		else:
			self.lbc_safety2.setStyleSheet("background-color: yellow;")
			self.lbc_safety_behind.setStyleSheet("background-color: yellow;")

		# ---- Battery
		if (self.statusColor.lbc_battery == 0):
			self.lbv_battery.setStyleSheet("background-color: white; color: black;")
			self.lb_v.setStyleSheet("color: black;")
		elif (self.statusColor.lbc_battery == 1):
			self.lbv_battery.setStyleSheet("background-color: green; color: white;")
			self.lb_v.setStyleSheet("color: white;")
		elif (self.statusColor.lbc_battery == 2):
			self.lbv_battery.setStyleSheet("background-color: orange; color: black;")
			self.lb_v.setStyleSheet("color: black;")
		elif (self.statusColor.lbc_battery == 3):
			self.lbv_battery.setStyleSheet("background-color: red; color: white;")
			self.lb_v.setStyleSheet("color: white;")
		elif (self.statusColor.lbc_battery == 4):
			self.lbv_battery.setStyleSheet("background-color: yellow; color: black;")
			self.lb_v.setStyleSheet("color: black;")
		else: # -- charging
			self.lbv_battery.setStyleSheet("background-color: white; color: black;")
			self.lb_v.setStyleSheet("color: black;")

		# ---- Thanh trang thai AGV
		if (self.statusColor.cb_status == 0):
			self.cb_status.setStyleSheet("background-color: green; color: white;")
		elif (self.statusColor.cb_status == 1):
			self.cb_status.setStyleSheet("background-color: orange; color: black;")	
		elif (self.statusColor.cb_status == 2):
			self.cb_status.setStyleSheet("background-color: red; color: white;")
		else:
			self.cb_status.setStyleSheet("background-color: white;")

		# ---- Trang thai che do dang hoat dong
		if (self.valueLable.modeRuning == self.modeRun_byhand):
			self.bt_passHand.setStyleSheet("background-color: blue;")	
			self.bt_passAuto.setStyleSheet("background-color: white;")	

		elif (self.valueLable.modeRuning == self.modeRun_auto):
			self.bt_passHand.setStyleSheet("background-color: white;")	
			self.bt_passAuto.setStyleSheet("background-color: blue;")
		else:	
			self.bt_passHand.setStyleSheet("background-color: white;")	
			self.bt_passAuto.setStyleSheet("background-color: white;")

		# -- Button Clear error
		if (self.statusColor.lbc_button_clearError == 1):
			self.lbc_button_clearError.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_button_clearError == 0):
			self.lbc_button_clearError.setStyleSheet("background-color: white; color: black")

		# -- Button Power
		if (self.statusColor.lbc_button_power == 1):
			self.lbc_button_power.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_button_power == 0):
			self.lbc_button_power.setStyleSheet("background-color: white; color: black")

		# -- Blsock
		if (self.statusColor.lbc_blsock1 == 1):
			self.lbc_blsock1.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_blsock1 == 0):
			self.lbc_blsock1.setStyleSheet("background-color: white; color: black")

		if (self.statusColor.lbc_blsock2 == 1):
			self.lbc_blsock2.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_blsock2 == 0):
			self.lbc_blsock2.setStyleSheet("background-color: white; color: black")

		# -- EMG
		if (self.statusColor.lbc_emg1 == 1):
			self.lbc_emg1.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_emg1 == 0):
			self.lbc_emg1.setStyleSheet("background-color: white; color: black")	

		if (self.statusColor.lbc_emg2 == 1):
			self.lbc_emg2.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_emg2 == 0):
			self.lbc_emg2.setStyleSheet("background-color: white; color: black")	

		# - Safety Relay
		if (self.statusColor.lbc_safety_relay == 1):
			self.lbc_safetyRelay.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_safety_relay == 0):
			self.lbc_safetyRelay.setStyleSheet("background-color: white; color: black")	

		# -- Port: RTC Board
		if (self.statusColor.lbc_port_rtc == 1):
			self.lbc_port_rtc.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rtc == 0):
			self.lbc_port_rtc.setStyleSheet("background-color: red; color: white;")

		# -- Port: RS485
		if (self.statusColor.lbc_port_rs485 == 1):
			self.lbc_port_rs485.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rs485 == 0):
			self.lbc_port_rs485.setStyleSheet("background-color: red; color: white;")

		# -- Port: NAV350
		if (self.statusColor.lbc_port_nav350 == 1):
			self.lbc_port_nav350.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_nav350 == 0):
			self.lbc_port_nav350.setStyleSheet("background-color: red; color: white;")

		# -- Sensor Up
		if (self.statusColor.lbc_limitAbove == 1):
			self.lbc_limitAbove.setStyleSheet("background-color: blue; color: white;")
			self.lbc_limitAbove2.setStyleSheet("background-color: blue; color: white;")
		elif (self.statusColor.lbc_limitAbove == 0):
			self.lbc_limitAbove.setStyleSheet("background-color: white; color: black;")
			self.lbc_limitAbove2.setStyleSheet("background-color: white; color: black;")

		# -- Sensor Down
		if (self.statusColor.lbc_limitBelow == 1):
			self.lbc_limitBelow.setStyleSheet("background-color: blue; color: white;")
			self.lbc_limitBelow2.setStyleSheet("background-color: blue; color: white;")
		elif (self.statusColor.lbc_limitBelow == 0):
			self.lbc_limitBelow.setStyleSheet("background-color: white; color: black;")
			self.lbc_limitBelow2.setStyleSheet("background-color: white; color: black;")

		# -- Sensor detect lift
		if (self.statusColor.lbc_checkTray == 1):
			self.lbc_checkTray.setStyleSheet("background-color: blue; color: white;")
			self.lbc_checkTray2.setStyleSheet("background-color: blue; color: white;")

		elif (self.statusColor.lbc_checkTray == 0):
			self.lbc_checkTray.setStyleSheet("background-color: white; color: black;")
			self.lbc_checkTray2.setStyleSheet("background-color: white; color: black;")

		# -- CAN port
		self.lbc_can_rtc.setStyleSheet("background-color: blue; color: white")

		if (self.statusColor.lbc_can_psu == 1):
			self.lbc_can_psu.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_can_psu == 0):
			self.lbc_can_psu.setStyleSheet("background-color: red; color: white;")

		if (self.statusColor.lbc_can_mcu == 1):
			self.lbc_can_mcu.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_can_mcu == 0):
			self.lbc_can_mcu.setStyleSheet("background-color: red; color: white;")

		if (self.statusColor.lbc_can_hcu == 1):
			self.lbc_can_hcu.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_can_hcu == 0):
			self.lbc_can_hcu.setStyleSheet("background-color: red; color: white;")

		if (self.statusColor.lbc_can_oc == 1):
			self.lbc_can_oc.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_can_oc == 0):
			self.lbc_can_oc.setStyleSheet("background-color: red; color: white;")

		# - 
		if self.statusColor.lbc_tryTarget_status == 0:
			self.lbc_tryTarget_StopState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_MoveState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_DownState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_completed.setStyleSheet("background-color: white;")

		elif self.statusColor.lbc_tryTarget_status == 1:
			self.lbc_tryTarget_StopState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_MoveState.setStyleSheet("background-color: blue;")
			self.lbc_tryTarget_DownState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_completed.setStyleSheet("background-color: white;")

		elif self.statusColor.lbc_tryTarget_status == 2:
			self.lbc_tryTarget_StopState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_MoveState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_DownState.setStyleSheet("background-color: blue;")
			self.lbc_tryTarget_completed.setStyleSheet("background-color: white;")

		elif self.statusColor.lbc_tryTarget_status == 3:
			self.lbc_tryTarget_StopState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_MoveState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_DownState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_completed.setStyleSheet("background-color: blue;")

		elif self.statusColor.lbc_tryTarget_status == 4:
			self.lbc_tryTarget_StopState.setStyleSheet("background-color: blue;")
			self.lbc_tryTarget_MoveState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_DownState.setStyleSheet("background-color: white;")
			self.lbc_tryTarget_completed.setStyleSheet("background-color: white;")

	def set_labelValue(self): # App_lbv()
		self.lbv_angleCompare.setText(str(self.dial_angleCompare.value()))
		# --
		self.lbv_battery.setText(self.valueLable.lbv_battery)

		self.lbv_coordinates_x.setText(self.valueLable.lbv_coordinates_x)
		self.lbv_coordinates_y.setText(self.valueLable.lbv_coordinates_y)
		self.lbv_coordinates_r.setText(self.valueLable.lbv_coordinates_r)

		self.lbv_coordinates_x1.setText(self.valueLable.lbv_coordinates_x)
		self.lbv_coordinates_y1.setText(self.valueLable.lbv_coordinates_y)
		self.lbv_coordinates_r1.setText(self.valueLable.lbv_coordinates_r)

		self.lbv_coordinates_x2.setText(self.valueLable.lbv_coordinates_x)
		self.lbv_coordinates_y2.setText(self.valueLable.lbv_coordinates_y)
		self.lbv_coordinates_r2.setText(self.valueLable.lbv_coordinates_r)

		self.lbv_numberReflector.setText(self.valueLable.lbv_numbeReflector)

		self.lbv_route_target.setText(self.valueLable.lbv_route_target)
		self.lbv_jobRuning.setText(self.valueLable.lbv_jobRuning)
		self.lbv_goalFollow_id.setText(self.valueLable.lbv_goalFollow_id)
		self.lbv_route_point0.setText(self.valueLable.lbv_route_point0)
		self.lbv_route_point1.setText(self.valueLable.lbv_route_point1)
		self.lbv_route_point2.setText(self.valueLable.lbv_route_point2)
		self.lbv_route_point3.setText(self.valueLable.lbv_route_point3)
		self.lbv_route_point4.setText(self.valueLable.lbv_route_point4)
		self.lbv_route_job1.setText(self.valueLable.lbv_route_job1)
		self.lbv_route_job2.setText(self.valueLable.lbv_route_job2)
		self.lbv_route_job1_mean.setText(self.valueLable.lbv_route_job1_mean)
		self.lbv_route_job2_mean.setText(self.valueLable.lbv_route_job2_mean)

		self.lbv_route_message.setText(self.valueLable.lbv_route_message)

		# self.lbv_coorAverage_x.setText(self.valueLable.lbv_coorAverage_x)
		# self.lbv_coorAverage_y.setText(self.valueLable.lbv_coorAverage_y)
		# self.lbv_coorAverage_r.setText(self.valueLable.lbv_coorAverage_r)
		# self.lbv_coorAverage_times.setText(self.valueLable.lbv_coorAverage_times)

		self.lbv_velLeft.setText(self.valueLable.lbv_velLeft)
		self.lbv_velRight.setText(self.valueLable.lbv_velRight)

		self.lbv_notification_driver1.setText(self.valueLable.lbv_notification_driver1)
		self.lbv_notification_driver2.setText(self.valueLable.lbv_notification_driver2)

		# - 
		self.lbv_reflectorLoc.setText(self.valueLable.lbv_numbeReflector)
		self.lbv_reflectorDetect.setText(self.valueLable.lbv_reflectorDetect)
		self.lbv_reflectorDetect_1.setText(self.valueLable.lbv_reflectorDetect)
		# self..setText(self.valueLable.)

		# -
		self.lbv_temp.setText(self.valueLable.lbv_temp)

		# - 
		self.lbv_voltage.setText(self.valueLable.lbv_voltage)

		# - nuc info
		self.lbv_mac.setText(self.valueLable.lbv_mac)
		self.lbv_namePc.setText(self.valueLable.lbv_namePc)
		self.lbv_ipEthernet.setText(self.valueLable.lbv_ipEthernet)
		self.lbv_ip.setText(self.valueLable.lbv_ipWifi)
		self.lbv_cpu.setText(self.valueLable.lbv_cpu_usage)
		self.lbv_cpu2.setText(self.valueLable.lbv_cpu_usage)
		self.lbv_tempCpu.setText(self.valueLable.lbv_cpu_temp)

		self.lbv_ram.setText(self.valueLable.lbv_ram)
		self.lbv_runtime.setText(self.valueLable.lbv_runtime)
		self.lbv_QualityWifi.setText(self.valueLable.lbv_wifiQuality)
		self.lbv_SignalWifi.setText(self.valueLable.lbv_wifiSignal)
		self.lbv_SignalWifi2.setText(self.valueLable.lbv_wifiSignal)

		self.lbv_pingServer.setText(self.valueLable.lbv_pingServer)		
		self.lbv_pingServer2.setText(self.valueLable.lbv_pingServer)

	def controlShow_followMode(self):		
		# if (self.valueLable.modeRuning == self.modeRun_launch):
		# 	self.modeRuning = self.modeRun_launch

		if (self.valueLable.modeRuning == self.modeRun_byhand):
			self.modeRuning = self.modeRun_byhand

		elif (self.valueLable.modeRuning == self.modeRun_auto):
			self.modeRuning = self.modeRun_auto
		else:
			self.modeRuning = self.modeRun_auto
		# --
		if (self.modeRuning == self.modeRun_launch): # -- Khoi Dong
			self.fr_launch.show()
			self.fr_run.hide()
			self.show_launch()
		else:
			self.fr_run.show()
			self.fr_launch.hide()

			if (self.modeRuning == self.modeRun_auto): # -- Tu dong
				self.fr_control.show()
				self.fr_setting.hide()
				self.fr_handMode_conveyor.hide()
				self.fr_handMode_move.hide()
				self.fr_listTask.show()
				self.isShow_moveHand = 1

				if self.is_showDetail == 1:
					self.fr_AGVamimation.hide()
					self.fr_listPoint.show()
				else:
					self.fr_AGVamimation.show()
					self.fr_listPoint.hide()

			elif (self.modeRuning == self.modeRun_byhand):
				if self.isShow_setting == 1 and self.status_show_setting == 1:
					self.fr_control.hide()
					self.fr_info.hide()
					self.fr_setting.show()

					if self.fr_function_no == 0:
						self.fr_getWarehouse.show()
						self.fr_SpecialPoint.hide()
						self.fr_nav350.hide()

					elif self.fr_function_no == 1:
						self.fr_getWarehouse.hide()
						self.fr_SpecialPoint.hide()
						self.fr_nav350.show()

					elif self.fr_function_no == 2:
						self.fr_getWarehouse.hide()
						self.fr_SpecialPoint.show()
						self.fr_nav350.hide()

				elif self.isShow_setting == 1 and self.status_show_setting == 0:
					self.fr_control.show()
					self.fr_info.hide()
					self.fr_setting.hide()

					self.fr_agv.hide()
					self.fr_password.show()
					# self.fr_info.hide()
					# self.fr_setting.hide()
					self.fr_handMode_conveyor.hide()
					self.fr_handMode_move.show()
					self.fr_handMode_move.setEnabled(False)

				else:
					self.fr_handMode_move.setEnabled(True)
					# - Hiển thị màn hình thông tin
					if self.isShow_info == 1:
						self.fr_control.hide()
						self.fr_info.show()
						self.fr_setting.hide()

					# - Hiển thị chức năng điều khiển tay.
					else:
						self.fr_control.show()
						self.fr_setting.hide()
						self.fr_info.hide()
						self.fr_listTask.hide()

						if (self.isShow_moveHand == 1):
							self.fr_handMode_conveyor.hide()
							self.fr_handMode_move.show()
						else:
							self.fr_handMode_conveyor.show()
							self.fr_handMode_move.hide()

	def coorAverage_run(self):
		if (self.bt_coorAverage_status == 1):
			self.countTime_coorAverage += 1.
			self.total_x += self.robotPoseNow.position.x
			self.total_y += self.robotPoseNow.position.y
			euler = self.quaternion_to_euler(self.robotPoseNow.orientation)
			self.total_angle += euler

		else:
			self.timeSave_coorAverage = rospy.Time.now()
			if (self.countTime_coorAverage > 4):
				d_x = self.total_x/self.countTime_coorAverage
				d_y = self.total_y/self.countTime_coorAverage
				d_a = self.total_angle/self.countTime_coorAverage
				d_degree = degrees(d_a)
				if d_degree < 0:
					d_degree = 360 + d_degree

				# self.lbv_coorAverage_times.setText(str(self.countTime_coorAverage))
				self.lbv_coorAverage_x.setText(str(round(d_x, 3)))
				self.lbv_coorAverage_y.setText(str(round(d_y, 3)))
				self.lbv_coorAverage_r.setText(str(round(d_degree, 2)))

				if self.ck_linkCoor.isChecked() == 1:
					self.valueLable.lbv_tryTarget_x = d_x
					self.valueLable.lbv_tryTarget_y = d_y
					self.valueLable.lbv_tryTarget_r = d_degree
					self.lbv_tryTarget_x.setText(str(round(self.valueLable.lbv_tryTarget_x, 3)))
					self.lbv_tryTarget_y.setText(str(round(self.valueLable.lbv_tryTarget_y, 3)))
					self.lbv_tryTarget_r.setText(str(round(self.valueLable.lbv_tryTarget_r, 2)))

			self.countTime_coorAverage = 0
			self.total_x = 0.0
			self.total_y = 0.0
			self.total_angle = 0.0

	def show_launch(self):
		self.lbv_launhing.setText(self.valueLable.lbv_launhing)
		self.lbv_numberLaunch.setText(str(self.valueLable.lbv_numberLaunch))
		# --
		value = self.valueLable.percentLaunch
		if (value < 0):
			value = 0

		if (value > 100):
			value = 100

		self.pb_launch.setValue(value)
		# --
		# -- Port: RTC Board
		if (self.statusColor.lbc_port_rtc == 1):
			self.lbc_lh_rtc.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rtc == 0):
			self.lbc_lh_rtc.setStyleSheet("background-color: red; color: white;")

		# -- Port: RS485
		if (self.statusColor.lbc_port_rs485 == 1):
			self.lbc_lh_driver.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rs485 == 0):
			self.lbc_lh_driver.setStyleSheet("background-color: red; color: white;")

		# -- Port: NAV350
		if (self.statusColor.lbc_port_nav350 == 1):
			self.lbc_lh_lidar.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_nav350 == 0):
			self.lbc_lh_lidar.setStyleSheet("background-color: red; color: white;")

	def show_password(self):
		data = ""
		lenght = len(self.password_data)

		for i in range(lenght):
			data += "*"
		
		self.lbv_pw_data.setText(data)
		
		if self.isShow_setting == 0:
			if (self.password_data == self.password_right):
				self.bt_pw_agree.setEnabled(True)
			else:
				self.bt_pw_agree.setEnabled(False)
		else:
			if (self.password_data == '1222'):
				self.bt_pw_agree.setEnabled(True)
			else:
				self.bt_pw_agree.setEnabled(False)			

	# def control_tryTarget(self):
