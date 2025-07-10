#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Phùng Quý Dương
Company: STI Viet Nam
date: 20/3/2025
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import logging

import sys
import time
import threading
import signal
import json

import os
import re  
import subprocess
import argparse
from datetime import datetime

from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication, QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer, QDateTime, Qt
import sqlite3

sys.path.append('/opt/ros/humble/lib/python3/dist-packages')
#sys.path.append('/home/stivietnam/ros2_ws/install/lib/python3/dist-packages')

from message_pkg.msg import *
from std_msgs.msg import Int16, Bool, Int8
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose

# from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees, asin, copysign
import numpy as np 

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw).

    Parameters:
    x, y, z, w: Quaternion components.

    Returns:
    roll, pitch, yaw: Euler angles in radians.
    """
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = copysign(pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class statusButton:
	def __init__(self):
		self.bt_passAuto = 0
		self.bt_passHand = 0
		self.bt_cancelMission = 0
		self.bt_setting = 0
		self.bt_clearError = 0
		# - 
		self.bt_moveHand = 0
		# -
		self.bt_disPointA = 0
		self.bt_disPointB = 0
		# -
		self.bt_hideSetting = 0
		# -
		self.bt_speaker = 0
		self.bt_charger = 0
		self.bt_lifter = 0
		self.bt_auxiliaryLights = 0

		self.vs_speed = 50
		self.ck_remote = 0
		self.ck_magline = 0
		self.bt_resetFrameWork = 0

		self.bt_disableBrake = 0
		self.bt_confirm = 0
		self.soundtype = 0
		self.ledtype = 0
		self.bt_dir = -1

class statusColor:
	def __init__(self):
		self.lbc_safety_ahead = 0
		self.lbc_safety_behind = 0
		self.lbc_safety_circle = 0
		# --
		self.cb_status = 0
		self.lbc_battery = 0

		# --
		self.lbc_button_clearError = 0
		self.lbc_button_power = 0
		self.lbc_emg = 0

		self.lbc_badersock = 0

		# -
		self.lbc_port_rtcBoard = 0
		self.lbc_port_Magline = 0
		self.lbc_port_RFID = 0
		self.lbc_port_Driver = 0
		# - 
		self.lbc_setpose = 0

		# - oc 
		self.lbc_limit_up = 0
		self.lbc_detect_lifter = 0
		self.lbc_limit_down = 0

		# - dàn dò
		self.lbc_magline_t1 = 0
		self.lbc_magline_t2 = 0
		self.lbc_magline_t3 = 0
		self.lbc_magline_t4 = 0
		self.lbc_magline_t5 = 0
		self.lbc_magline_t6 = 0
		self.lbc_magline_t7 = 0
		self.lbc_magline_t8 = 0
		self.lbc_magline_t9 = 0
		self.lbc_magline_t10 = 0
		self.lbc_magline_t11 = 0
		self.lbc_magline_t12 = 0
		self.lbc_magline_t13 = 0
		self.lbc_magline_t14 = 0
		self.lbc_magline_t15 = 0
		self.lbc_magline_t16 = 0	

		self.lbc_magline_s1 = 0
		self.lbc_magline_s2 = 0
		self.lbc_magline_s3 = 0
		self.lbc_magline_s4 = 0
		self.lbc_magline_s5 = 0
		self.lbc_magline_s6 = 0
		self.lbc_magline_s7 = 0
		self.lbc_magline_s8 = 0
		self.lbc_magline_s9 = 0
		self.lbc_magline_s10 = 0
		self.lbc_magline_s11 = 0
		self.lbc_magline_s12 = 0
		self.lbc_magline_s13 = 0
		self.lbc_magline_s14 = 0
		self.lbc_magline_s15 = 0
		self.lbc_magline_s16 = 0	

class valueLable:
	def __init__(self):
		self.modeRuning = 0

		self.lbv_name_agv = ''
		self.lbv_ip = ''
		self.lbv_battery = ''
		self.lbv_date = ''

		self.lbv_rfid_lastcode = ''
		self.lbv_rfid_code = ''
		self.lbv_direction = ''

		self.lbv_numbeReflector = ''
		self.lbv_pingServer = ''
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
		self.lbv_route_message = ''
		
		self.lbv_conveyorA = ''
		self.lbv_conveyorB = ''

		self.lbv_coorAverage_x = ''
		self.lbv_coorAverage_y = ''
		self.lbv_coorAverage_r = ''
		self.lbv_coorAverage_times = ''
		self.lbv_deltaDistance = ''

		self.lbv_mac = ''
		self.lbv_namePc = ''

		self.lbv_launhing = ''
		self.lbv_numberLaunch = ''
		self.percentLaunch = 0
		self.listError = ['A', 'B', 'C']
		self.listError_pre = []

		self.lbv_notification_driver1 = ''
		self.lbv_notification_driver2 = ''

		self.lbv_nuc_mac = ''
		self.lbv_nuc_name = ''
		self.lbv_nuc_ip = ''

		self.lbv_cpu_usage = ''
		self.lbv_cpu_temp = ''
		self.lbv_ram = ''
		self.lbv_ping = ''
		self.lbv_wifi_quality = ''
		self.lbv_wifi_signal = ''
		self.lbv_ap_mac = ''
		self.lbv_runtime = ''

		self.lbv_velLeft = ''
		self.lbv_velRight = ''

		self.lbv_process = 0
		# self.setPose_tagID = ''

		# self.lbv_tagID = ''
		# self.lbv_tagDistance = ''
		# self. = ''
		# self. = ''
		# self. = ''
		# self. = ''


class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		# -
		loadUi("/home/stivietnam/ros2_ws/src/app_ros/interface/app.ui", self)
		# --

		# Đường dẫn tuyệt đối của thư mục chứa file Python đang chạy
		self.script_dir = os.path.dirname(os.path.realpath(__file__))

		# Ghép thêm thư mục con "interface" và tên file ảnh
		self.image_path = os.path.join(self.script_dir, "interface")

		self.statusButton = statusButton()
		self.statusColor  = statusColor()
		self.valueLable   = valueLable()
		# --
		# self.statusButton.bt_disableBrake = 1
		self.statusButton.bt_speaker = 1
		self.statusButton.bt_charger = 1
		# --
		# self.setWindowTitle("my name")
		# --
		# self.fr_run.show()
		# self.fr_agv.show()

		# self.fr_password.hide()
		# self.fr_setting.hide()
		# self.fr_controlHand.hide()
		# -
		self.bt_extand_show.released.connect(self.released_extand_show)
		self.bt_extand_hide.released.connect(self.released_extand_hide)
		# -- -- -- 
		self.bt_exit.pressed.connect(self.out)
		self.bt_exit1.pressed.connect(self.out)
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
		# -
		self.bt_lifter_up.pressed.connect(self.pressed_lifter_up)
		self.bt_lifter_down.pressed.connect(self.pressed_lifter_down)
		self.bt_lifter_reset.pressed.connect(self.pressed_lifter_reset)
		# -
		self.bt_brake_on.clicked.connect(self.clicked_brake_on)
		self.bt_brake_off.clicked.connect(self.clicked_brake_off)

		# -
		self.bt_dir1.pressed.connect(self.pressed_bt_dir1)
		self.bt_dir1.released.connect(self.released_bt_dir1)

		self.bt_dir2.pressed.connect(self.pressed_bt_dir2)
		self.bt_dir2.released.connect(self.released_bt_dir2)

		self.bt_dir3.pressed.connect(self.pressed_bt_dir3)
		self.bt_dir3.released.connect(self.released_bt_dir3)

		self.bt_dir4.pressed.connect(self.pressed_bt_dir4)
		self.bt_dir4.released.connect(self.released_bt_dir4)

		# ------ moveHand
		self.bt_forwards.clicked.connect(self.clicked_forwards)
		self.bt_backwards.clicked.connect(self.clicked_backwards)
		# -
		self.bt_rotation_left.clicked.connect(self.clicked_rotation_left)
		self.bt_rotation_right.clicked.connect(self.clicked_rotation_right)
		# -
		self.bt_stop.clicked.connect(self.clicked_stop)
		# - 
		self.bt_forwards_rfid.clicked.connect(self.clicked_forwards_rfid)
		self.bt_backwards_line.clicked.connect(self.clicked_backwards_line)

		# -- Set speed manual
		self.bt_upSpeed.pressed.connect(self.pressed_upSpeed)
		self.bt_reduceSpeed.pressed.connect(self.pressed_reduceSpeed)
	
		# -- Setting devices
		self.bt_setting.pressed.connect(self.pressed_setting)
		self.bt_setting.released.connect(self.released_setting)
		self.setting_status = 0
		self.timeSave_setting = time.time()
		# -
		self.bt_hideSetting.clicked.connect(self.clicked_hideSetting)
		self.bt_nextInfoPage.clicked.connect(self.clicked_nextInfopage)
		self.bt_backInfoPage.clicked.connect(self.clicked_backInfopage)
		self.bt_hideTranjectory.clicked.connect(self.clicked_hideTranjectory)
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

		# -
		self.ck_remote.stateChanged.connect(self.stateChanged_ck_remote)
		self.ck_magline.stateChanged.connect(self.stateChanged_ck_magline)
		self.ck_remote.setEnabled(False)

		self.lb_charger.setEnabled(False)
		self.bt_charger_on.setEnabled(False)
		self.bt_charger_off.setEnabled(False)

		self.bt_resetFrameWork.pressed.connect(self.pressed_resetFrameWork)
		self.bt_resetFrameWork.released.connect(self.released_resetFrameWork)

		self.bt_confirm.pressed.connect(self.pressed_confirm)
		self.bt_confirm.released.connect(self.released_confirm)

        # -- combo Box of speakerhu
		self.cb_listSpeak.addItem(" Tắt loa")
		self.cb_listSpeak.addItem(" Âm 1 - Khởi động")
		self.cb_listSpeak.addItem(" Âm 2 - Bằng tay")
		self.cb_listSpeak.addItem(" Âm 3 - Di chuyển")
		self.cb_listSpeak.addItem(" Âm 4 - Chờ Xác nhận")
		self.cb_listSpeak.addItem(" Âm 5 - Chờ Thang máy")
		self.cb_listSpeak.addItem(" Âm 6 - Lỗi")

		self.cb_listLed.addItem(" Tắt led")
		self.cb_listLed.addItem(" Led 1 - Lỗi")
		self.cb_listLed.addItem(" Led 2 - Di chuyển")
		self.cb_listLed.addItem(" Led 3 - Lùi vào kệ")
		self.cb_listLed.addItem(" Led 4 - Thao tác kệ")
		self.cb_listLed.addItem(" Led 5 - Hoàn thành")
		self.cb_listLed.addItem(" Led 6 - Cảnh báo vật cản") 
	
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
		self.modeRuning = self.modeRun_launch
		# --
		self.password_data = ""
		# --
		self.timeSave_cancelMisson = time.time()
		self.cancelMission_status = 0
		# -- 
		self.isShow_setting = 0
		self.isShow_settingPage = 1
		# -
		self.robotPoseNow = Pose()
		self.pointA = Point()
		self.pointB = Point()
		# -
		self.countTime_coorAverage = 0
		self.total_x = 0.0
		self.total_y = 0.0
		self.total_angle = 0.0
		# -
		self.isShow_moveHand = 1
		# self.enable_buttonInfo = 0
		# self.bt_funucIn_HOLD.setEnabled(False)

	def process_fast(self):
		# self.pb_qualityWifi.setValue(self.valueLable.lbv_qualityWifi)
		# --
		# if self.valueLable.lbv_process == 11:
		# 	self.bt_confirm.setStyleSheet("background-color: rgb(80,249,255);")
		# 	self.bt_confirm.setText('XÁC NHẬN')
			
		if self.valueLable.lbv_process == 10:
			self.bt_confirm.setStyleSheet("background-color: yellow;")
			self.bt_confirm.setText('CHỜ XÁC NHẬN')

		else:
			self.bt_confirm.setStyleSheet("background-color: rgb(80,249,255);")
			self.bt_confirm.setText('XÁC NHẬN')

		# - Combo Box
		if (self.valueLable.listError != self.valueLable.listError_pre):
			self.valueLable.listError_pre = self.valueLable.listError
			self.cb_status.clear()
			lg = len(self.valueLable.listError) 
			for i in range(lg):
				self.cb_status.addItem(self.valueLable.listError[i])
		# -
        # - cb of list speak
		self.statusButton.soundtype = self.cb_listSpeak.currentIndex()

		# - cb of list led
		self.statusButton.ledtype = self.cb_listLed.currentIndex()
	
		# - 
		if self.valueLable.modeRuning == 1:
			self.statusButton.bt_setting = self.isShow_setting
		# -- 
		# self.coorAverage_run()
		# # --
		self.set_labelValue()
		# # --
		self.set_labelColor()
		# # --
		self.controlShow_followMode()
		# # --
		# self.update_setting()

		# -- show check devices
		if (self.setting_status == 1):
			delta_t = time.time() - self.timeSave_setting
			# -- Chi kich hoat khi dang o che do Bang Tay.
			if (delta_t > 1.5): #and self.valueLable.modeRuning == 1:
				self.isShow_setting = 1
				self.password_data = ""
		else:
			self.timeSave_setting = time.time()

		# -- add 21/01/2022 - show cancelMission
		if (self.cancelMission_status == 1):
			delta_c = time.time() - self.timeSave_cancelMisson
			if (delta_c > 0.5):
				self.fr_agv.hide()
				self.fr_password.show()
				self.clicked_stop()
		else:
			self.timeSave_cancelMisson = time.time()
		# --
		self.show_password()
		
	# -
	def pressed_resetFrameWork(self):
		self.statusButton.bt_resetFrameWork = 1
		self.bt_resetFrameWork.setStyleSheet("background-color: blue;")
		
	def released_resetFrameWork(self):
		self.statusButton.bt_resetFrameWork = 0
		self.bt_resetFrameWork.setStyleSheet("background-color: white;")

	def pressed_confirm(self):
		self.statusButton.bt_confirm = 1
		# self.bt_confirm.setStyleSheet("background-color: blue;")
		# self.bt_confirm.setText('ĐANG XÁC NHẬN')
		
	def released_confirm(self):
		self.statusButton.bt_confirm = 0
		# self.bt_confirm.setStyleSheet("background-color: rgb(80,249,255);")

	# -
	def stateChanged_ck_remote(self):
		self.clicked_stop()

	# -
	def stateChanged_ck_magline(self):
		# self.statusButton.bt_moveHand = 0
		self.clicked_stop()
		if self.statusButton.ck_magline == 1:
			self.ck_remote.setEnabled(True)
		else:
			self.ck_remote.setEnabled(False)

	# -
	def released_extand_show(self):
		self.isShow_moveHand = 0
		self.clicked_stop()
		# -
		self.statusButton.bt_moveHand = 0
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: blue;")

	def released_extand_hide(self):
		self.isShow_moveHand = 1
		self.clicked_stop()

	# -
	def pressed_cancelMission(self):
		self.bt_cancelMission.setStyleSheet("background-color: blue;")
		self.cancelMission_status = 1
		self.clicked_stop()

	def released_cancelMission(self):
		self.bt_cancelMission.setStyleSheet("background-color: white;")
		self.cancelMission_status = 0
		self.isShow_setting = 0
		self.clicked_stop()
	# -
	def pressed_passHand(self):
		self.statusButton.bt_passHand = 1
		self.statusButton.bt_passAuto = 0
		self.clicked_stop()
		self.bt_passHand.setStyleSheet("background-color: blue;")
		
	def released_passHand(self):
		self.statusButton.bt_passHand = 0
		self.statusButton.bt_passAuto = 0
		self.clicked_stop()
		self.bt_passHand.setStyleSheet("background-color: white;")
		self.isShow_setting = 0
	# -
	def pressed_passAuto(self):
		self.statusButton.bt_passAuto = 1
		self.statusButton.bt_passHand = 0
		self.clicked_stop()
		
	def released_passAuto(self):
		self.statusButton.bt_passAuto = 0
		self.statusButton.bt_passHand = 0
		self.isShow_setting = 0
		self.clicked_stop()

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
		self.statusButton.bt_speaker = 1
		self.bt_speaker_on.setStyleSheet("background-color: blue;")
		self.bt_speaker_off.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_speaker_off(self):
		self.statusButton.bt_speaker = 0
		self.bt_speaker_off.setStyleSheet("background-color: blue;")
		self.bt_speaker_on.setStyleSheet("background-color: white;")
		self.clicked_stop()

	# - 
	def clicked_charger_on(self):
		self.statusButton.bt_charger = 1
		self.bt_charger_on.setStyleSheet("background-color: blue;")
		self.bt_charger_off.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_charger_off(self):
		self.statusButton.bt_charger = 0
		self.bt_charger_off.setStyleSheet("background-color: blue;")
		self.bt_charger_on.setStyleSheet("background-color: white;")
		self.clicked_stop()

	# - 
	def pressed_lifter_up(self):
		self.statusButton.bt_lifter = 2
		self.bt_lifter_up.setStyleSheet("background-color: blue;")
		self.bt_lifter_down.setStyleSheet("background-color: white;")
		self.bt_lifter_reset.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def pressed_lifter_down(self):
		self.statusButton.bt_lifter = 1
		self.bt_lifter_up.setStyleSheet("background-color: white;")
		self.bt_lifter_down.setStyleSheet("background-color: blue;")
		self.bt_lifter_reset.setStyleSheet("background-color: white;")
		self.clicked_stop()

	def pressed_lifter_reset(self):
		self.statusButton.bt_lifter = 0
		self.bt_lifter_up.setStyleSheet("background-color: white;")
		self.bt_lifter_down.setStyleSheet("background-color: white;")
		self.bt_lifter_reset.setStyleSheet("background-color: blue;")
		self.clicked_stop()

	# - 
	def pressed_bt_dir1(self):
		self.statusButton.bt_dir = 1
		self.bt_dir1.setStyleSheet("background-color: blue;")

	def released_bt_dir1(self):
		self.statusButton.bt_dir = -1
		self.bt_dir1.setStyleSheet("background-color: white;")

	def pressed_bt_dir2(self):
		self.statusButton.bt_dir = 2
		self.bt_dir2.setStyleSheet("background-color: blue;")

	def released_bt_dir2(self):
		self.statusButton.bt_dir = -1
		self.bt_dir2.setStyleSheet("background-color: white;")

	def pressed_bt_dir3(self):
		self.statusButton.bt_dir = 3
		self.bt_dir3.setStyleSheet("background-color: blue;")

	def released_bt_dir3(self):
		self.statusButton.bt_dir = -1
		self.bt_dir3.setStyleSheet("background-color: white;")

	def pressed_bt_dir4(self):
		self.statusButton.bt_dir = 4
		self.bt_dir4.setStyleSheet("background-color: blue;")

	def released_bt_dir4(self):
		self.statusButton.bt_dir = -1
		self.bt_dir4.setStyleSheet("background-color: white;")

	# -
	def clicked_brake_on(self):
		self.statusButton.bt_disableBrake = 1
		self.bt_brake_on.setStyleSheet("background-color: blue;")
		self.bt_brake_off.setStyleSheet("background-color: white;")
		self.clicked_stop()
		
	def clicked_brake_off(self):
		self.statusButton.bt_disableBrake = 0
		self.bt_brake_off.setStyleSheet("background-color: blue;")
		self.bt_brake_on.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# --
	def clicked_forwards(self):
		self.statusButton.bt_moveHand = 1
		self.bt_forwards.setStyleSheet("background-color: blue;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
		self.bt_forwards_rfid.setStyleSheet("background-color: white;")
		self.bt_backwards_line.setStyleSheet("background-color: white;")
	# - 
	def clicked_backwards(self):
		self.statusButton.bt_moveHand = 2
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: blue;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
		self.bt_forwards_rfid.setStyleSheet("background-color: white;")
		self.bt_backwards_line.setStyleSheet("background-color: white;")
	# - 
	def clicked_rotation_left(self):
		self.statusButton.bt_moveHand = 3
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: blue;")
		self.bt_stop.setStyleSheet("background-color: white;")
		self.bt_forwards_rfid.setStyleSheet("background-color: white;")
		self.bt_backwards_line.setStyleSheet("background-color: white;")
	# - 
	def clicked_rotation_right(self):
		self.statusButton.bt_moveHand = 4
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: blue;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
		self.bt_forwards_rfid.setStyleSheet("background-color: white;")
		self.bt_backwards_line.setStyleSheet("background-color: white;")
	# - 
	def clicked_stop(self):
		self.statusButton.bt_moveHand = 0
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: blue;")
		self.bt_forwards_rfid.setStyleSheet("background-color: white;")
		self.bt_backwards_line.setStyleSheet("background-color: white;")

	def clicked_forwards_rfid(self):
		self.statusButton.bt_moveHand = 7
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
		self.bt_forwards_rfid.setStyleSheet("background-color: blue;")
		self.bt_backwards_line.setStyleSheet("background-color: white;")

	def clicked_backwards_line(self):
		self.statusButton.bt_moveHand = 8
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
		self.bt_forwards_rfid.setStyleSheet("background-color: white;")
		self.bt_backwards_line.setStyleSheet("background-color: blue;")

	# - 
	def pressed_upSpeed(self):
		self.statusButton.vs_speed += 10
		if self.statusButton.vs_speed >= 100:
			self.statusButton.vs_speed = 100

	def pressed_reduceSpeed(self):
		self.statusButton.vs_speed -= 10
		if self.statusButton.vs_speed < 5:
			self.statusButton.vs_speed = 5

	# --  --
	def clicked_password_agree(self):
		self.fr_agv.show()
		self.fr_password.hide()
		self.statusButton.bt_cancelMission = 1
		self.password_data = ""

	def clicked_password_cancel(self):
		self.fr_agv.show()
		self.fr_password.hide()
		self.password_data = ""

	def process_normal(self):
		self.set_dateTime()
		self.lbv_ip.setText(self.valueLable.lbv_nuc_ip)
		self.lbv_name_agv.setText(self.valueLable.lbv_name_agv)
		self.lbv_mac.setText(self.valueLable.lbv_nuc_mac)
		self.lbv_namePc.setText(self.valueLable.lbv_nuc_name)
			
	def process_slow(self):
		self.set_valueBattery(self.valueLable.lbv_battery)
		
	def clicked_hideSetting(self):
		self.isShow_setting = 0
		self.password_data = ""

	def clicked_nextInfopage(self):
		self.isShow_settingPage = 2

	def clicked_backInfopage(self):
		self.isShow_settingPage = 1

	def clicked_hideTranjectory(self):
		self.isShow_setting = 0

	def pressed_setting(self):
		self.bt_setting.setStyleSheet("background-color: blue;")	
		self.setting_status = 1
		self.clicked_stop()

	def released_setting(self):
		self.bt_setting.setStyleSheet("background-color: white;")	
		self.setting_status = 0
		# --
		self.isShow_setting = 1
		self.clicked_stop()
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

	def out(self):
		QApplication.quit()
		print('out')

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
		if (self.statusColor.lbc_safety_ahead == 0):
			self.label.clear()
		elif (self.statusColor.lbc_safety_ahead == 1):
			self.lb_warning_ahead.setPixmap(QPixmap(os.path.join(self.image_path, "warning.png")))
			self.lb_warning_ahead.setScaledContents(True)  # Tùy chọn: cho ảnh vừa khung label
		else:
			self.lb_warning_ahead.setPixmap(QPixmap(os.path.join(self.image_path, "warning_yellow.png")))
			self.lb_warning_ahead.setScaledContents(True)  # Tùy chọn: cho ảnh vừa khung label

		# -
		if (self.statusColor.lbc_safety_behind == 0):
			self.lbc_safety_behind.setStyleSheet("background-color: green; color: white")
		elif (self.statusColor.lbc_safety_behind == 1):
			self.lbc_safety_behind.setStyleSheet("background-color: red; color: white")
		else:
			self.lbc_safety_behind.setStyleSheet("background-color: yellow;")

		# -
		# if (self.statusColor.lbc_safety_circle == 0):
		# 	self.lbc_safety_circle.setStyleSheet("background-color: green; color: white")
		# else:
		# 	self.lbc_safety_circle.setStyleSheet("background-color: red; color: white")

		# ---- Battery
		if (self.statusColor.lbc_battery == 0):
			self.lbv_battery.setStyleSheet("background-color: white; color: black;")
			# self.lb_v.setStyleSheet("color: black;")
		elif (self.statusColor.lbc_battery == 1):
			self.lbv_battery.setStyleSheet("background-color: green; color: white;")
			# self.lb_v.setStyleSheet("color: white;")
		elif (self.statusColor.lbc_battery == 2):
			self.lbv_battery.setStyleSheet("background-color: orange; color: black;")
			# self.lb_v.setStyleSheet("color: black;")
		elif (self.statusColor.lbc_battery == 3):
			self.lbv_battery.setStyleSheet("background-color: red; color: white;")
			# self.lb_v.setStyleSheet("color: white;")
		elif (self.statusColor.lbc_battery == 4):
			self.lbv_battery.setStyleSheet("background-color: yellow; color: black;")
			# self.lb_v.setStyleSheet("color: black;")
		else: # -- charging
			self.lbv_battery.setStyleSheet("background-color: white; color: black;")
			# self.lb_v.setStyleSheet("color: black;")

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
			self.lbc_button_clearError.setStyleSheet("background-color: blue;")
		elif (self.statusColor.lbc_button_clearError == 0):
			self.lbc_button_clearError.setStyleSheet("background-color: white;")

		# -- Button Power
		if (self.statusColor.lbc_button_power == 1):
			self.lbc_button_power.setStyleSheet("background-color: blue;")
		elif (self.statusColor.lbc_button_power == 0):
			self.lbc_button_power.setStyleSheet("background-color: white;")

		# -- EMG
		if (self.statusColor.lbc_emg == 1):
			self.lbc_emg.setStyleSheet("background-color: blue;")
		elif (self.statusColor.lbc_emg == 0):
			self.lbc_emg.setStyleSheet("background-color: white;")	

		# -- Port: RTC Board
		if (self.statusColor.lbc_port_rtcBoard == 1):
			self.lbc_port_rtc.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rtcBoard == 0):
			self.lbc_port_rtc.setStyleSheet("background-color: red; color: white;")

		# -- Port: Magline
		if (self.statusColor.lbc_port_Magline == 1):
			self.lbc_port_magline.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_Magline == 0):
			self.lbc_port_magline.setStyleSheet("background-color: red; color: white;")

		# -- Port: RFID
		if (self.statusColor.lbc_port_RFID == 1):
			self.lbc_port_rfid.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_RFID == 0):
			self.lbc_port_rfid.setStyleSheet("background-color: red; color: white;")

		# -- Port: Driver
		if (self.statusColor.lbc_port_Driver == 1):
			self.lbc_port_driver.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_Driver == 0):
			self.lbc_port_driver.setStyleSheet("background-color: red; color: white;")

		# --
		self.statusButton.ck_remote = self.ck_remote.isChecked()
		self.statusButton.ck_magline = self.ck_magline.isChecked()

		# -- OC 
		if self.statusColor.lbc_limit_up == 1:
			self.lbc_limit_up.setStyleSheet("background-color: blue;")
		else:
			self.lbc_limit_up.setStyleSheet("background-color: white;")

		if self.statusColor.lbc_limit_down == 1:
			self.lbc_limit_down.setStyleSheet("background-color: blue;")
		else:
			self.lbc_limit_down.setStyleSheet("background-color: white;")		

		if self.statusColor.lbc_detect_lifter == 1:
			self.lbc_detect_lifter.setStyleSheet("background-color: blue;")
		else:
			self.lbc_detect_lifter.setStyleSheet("background-color: white;")

		# - OC 
		if self.statusColor.lbc_badersock == 1:
			self.lbc_badersock.setStyleSheet("background-color: blue;")
		else:
			self.lbc_badersock.setStyleSheet("background-color: white;")

		# -- Magline trước
		if self.statusColor.lbc_magline_t1 == 1:
			self.lbc_magline_t1.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t1.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t2 == 1:
			self.lbc_magline_t2.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t2.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t3 == 1:
			self.lbc_magline_t3.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t3.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t4 == 1:
			self.lbc_magline_t4.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t4.setStyleSheet("background-color: white;")	

		if self.statusColor.lbc_magline_t5 == 1:
			self.lbc_magline_t5.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t5.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t6 == 1:
			self.lbc_magline_t6.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t6.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t7 == 1:
			self.lbc_magline_t7.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t7.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t8 == 1:
			self.lbc_magline_t8.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t8.setStyleSheet("background-color: white;")

		if self.statusColor.lbc_magline_t9 == 1:
			self.lbc_magline_t9.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t9.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t10 == 1:
			self.lbc_magline_t10.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t10.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t11 == 1:
			self.lbc_magline_t11.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t11.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t12 == 1:
			self.lbc_magline_t12.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t12.setStyleSheet("background-color: white;")	

		if self.statusColor.lbc_magline_t13 == 1:
			self.lbc_magline_t13.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t13.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t14 == 1:
			self.lbc_magline_t14.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t14.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t15 == 1:
			self.lbc_magline_t15.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t15.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_t16 == 1:
			self.lbc_magline_t16.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_t16.setStyleSheet("background-color: white;")

		# -- Magline sau
		if self.statusColor.lbc_magline_s1 == 1:
			self.lbc_magline_s1.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s1.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s2 == 1:
			self.lbc_magline_s2.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s2.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s3 == 1:
			self.lbc_magline_s3.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s3.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s4 == 1:
			self.lbc_magline_s4.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s4.setStyleSheet("background-color: white;")	

		if self.statusColor.lbc_magline_s5 == 1:
			self.lbc_magline_s5.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s5.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s6 == 1:
			self.lbc_magline_s6.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s6.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s7 == 1:
			self.lbc_magline_s7.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s7.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s8 == 1:
			self.lbc_magline_s8.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s8.setStyleSheet("background-color: white;")

		if self.statusColor.lbc_magline_s9 == 1:
			self.lbc_magline_s9.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s9.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s10 == 1:
			self.lbc_magline_s10.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s10.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s11 == 1:
			self.lbc_magline_s11.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s11.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s12 == 1:
			self.lbc_magline_s12.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s12.setStyleSheet("background-color: white;")	

		if self.statusColor.lbc_magline_s13 == 1:
			self.lbc_magline_s13.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s13.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s14 == 1:
			self.lbc_magline_s14.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s14.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s15 == 1:
			self.lbc_magline_s15.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s15.setStyleSheet("background-color: white;")			

		if self.statusColor.lbc_magline_s16 == 1:
			self.lbc_magline_s16.setStyleSheet("background-color: yellow;")
		else:
			self.lbc_magline_s16.setStyleSheet("background-color: white;")

	def set_labelValue(self): # -
		self.lbv_battery.setText(self.valueLable.lbv_battery)

		self.lbv_rfid_lastcode.setText(self.valueLable.lbv_rfid_lastcode)
		self.lbv_rfid_code.setText(self.valueLable.lbv_rfid_code)
		self.lbv_direction.setText(self.valueLable.lbv_direction)

		self.lbv_jobRuning.setText(self.valueLable.lbv_jobRuning)
		# self.lbv_goalFollow_id.setText(self.valueLable.lbv_goalFollow_id)
		self.lbv_route_point0.setText(self.valueLable.lbv_route_point0)
		self.lbv_route_point1.setText(self.valueLable.lbv_route_point1)
		self.lbv_route_point2.setText(self.valueLable.lbv_route_point2)
		self.lbv_route_point3.setText(self.valueLable.lbv_route_point3)
		self.lbv_route_point4.setText(self.valueLable.lbv_route_point4)
		self.lbv_route_job1.setText(self.valueLable.lbv_route_job1)
		self.lbv_route_job2.setText(self.valueLable.lbv_route_job2)

		self.lbv_route_target.setText(self.valueLable.lbv_route_target)

		self.lbv_route_message.setText(self.valueLable.lbv_route_message)

		# self.lbv_setpose_idTag.setText(self.valueLable.setPose_tagID)

		# self.lbv_tagID.setText(self.valueLable.lbv_tagID)
		# self.lbv_tagDistance.setText(self.valueLable.lbv_tagDistance)

		self.pb_speed.setValue(self.statusButton.vs_speed)

		# - 
		self.lbv_cpu.setText(self.valueLable.lbv_cpu_usage)
		self.lbv_tempCpu.setText(self.valueLable.lbv_cpu_temp)

		self.lbv_ram.setText(self.valueLable.lbv_ram)
		self.lbv_runtime.setText(self.valueLable.lbv_runtime)

		self.lbv_SignalWifi.setText(self.valueLable.lbv_wifi_signal)
		self.lbv_QualityWifi.setText(self.valueLable.lbv_wifi_quality)

		self.lbv_ap_mac.setText(self.valueLable.lbv_ap_mac)

		self.lbv_pingServer.setText(self.valueLable.lbv_ping)
		self.lbv_pingServer2.setText(self.valueLable.lbv_ping + ' ms')

		# - 
		self.lbv_velLeft.setText(self.valueLable.lbv_velLeft)
		self.lbv_velRight.setText(self.valueLable.lbv_velRight)

	def controlShow_followMode(self):
		#if (self.valueLable.modeRuning == self.modeRun_launch):
		#	self.modeRuning = self.modeRun_launch

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
				if self.isShow_setting == 1:
					# self.fr_control.hide()
					# self.fr_setting.show()
					self.fr_btConfirm.hide()
					self.fr_listPoint.show()
				else:
					# self.fr_control.show()
					# self.fr_setting.hide()
					self.fr_btConfirm.show()
					self.fr_listPoint.hide()

				self.fr_handMode_extand.hide()
				self.fr_handMode_move.hide()
				self.fr_automode.show()

				self.isShow_moveHand = 1
				# -
				# self.enable_buttonInfo = 0

			elif (self.modeRuning == self.modeRun_byhand):
				
				if self.isShow_setting == 1:
					# self.fr_control.hide()
					self.fr_setting.show()
					self.fr_handMode_extand.hide()
					self.fr_handMode_move.hide()
					self.fr_automode.hide()

					if self.isShow_settingPage == 1:
						self.fr_infoDevice.show()
						self.fr_infoDevice2.hide()
					else:
						self.fr_infoDevice.hide()
						self.fr_infoDevice2.show()						

				else:
					# self.fr_control.show()
					self.fr_setting.hide()
					self.fr_automode.hide()

					if (self.isShow_moveHand == 1):
						self.fr_handMode_extand.hide()
						self.fr_handMode_move.show()
					else:
						self.fr_handMode_extand.show()
						self.fr_handMode_move.hide()
				# self.enable_buttonInfo = 1
			else:
				print ("ioiuoiuoi")

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
		if (self.statusColor.lbc_port_rtcBoard == 1):
			self.lbc_lh_rtcBoard.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rtcBoard == 0):
			self.lbc_lh_rtcBoard.setStyleSheet("background-color: red; color: white;")

		# -- Port: Magline
		if (self.statusColor.lbc_port_Magline == 1):
			self.lbc_lh_Magline.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_Magline == 0):
			self.lbc_lh_Magline.setStyleSheet("background-color: red; color: white;")

		# -- Port: RFID
		if (self.statusColor.lbc_port_RFID == 1):
			self.lbc_lh_RFID.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_RFID == 0):
			self.lbc_lh_RFID.setStyleSheet("background-color: red; color: white;")

		# -- Port: Driver
		if (self.statusColor.lbc_port_Driver == 1):
			self.lbc_lh_driver.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_Driver == 0):
			self.lbc_lh_driver.setStyleSheet("background-color: red; color: white;")

	def show_password(self):
		data = ""
		lenght = len(self.password_data)

		for i in range(lenght):
			data += "*"
		
		self.lbv_pw_data.setText(data)

		if (self.password_data == self.password_right):
			self.bt_pw_agree.setEnabled(True)
		else:
			self.bt_pw_agree.setEnabled(False)

		self.bt_pw_agree.setEnabled(True)
