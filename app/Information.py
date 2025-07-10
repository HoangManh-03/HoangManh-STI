#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Le Duc Anh
Company: STI Viet Nam
Date: 13/03/2024
"""

import rospy
import os
import subprocess
import re
# from ros_canBus.msg import *
from sti_msgs.msg import *
from message_pkg.msg import *
from std_msgs.msg import Int16, Bool, Int8
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose, TwistWithCovarianceStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

import math
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import * 
from PyQt5.QtGui import * 
from PyQt5.QtCore import * 
from threading import Event
import subprocess

class Mission_Error(QThread):
#Init
    def __init__(self, threadID):
        QThread.__init__(self)
        rospy.init_node('app_ros', anonymous=False)
        self.shutdown_flag = Event()
        self.is_exist = 1
        self.Sub_topic()
        self.listError = []
        self.list_logError = []
        self.Pub_topic()
        self.target_info = ""
        self.angle = 0
        self.point0 = ""
        self.point1 = ""
        self.point2 = ""
        self.point3 = ""
        self.point4 = ""
        self.mode = 0
        self.before = 0
        self.after = 0
        self.robotPose_nav_x = ""
        self.robotPose_nav_y = ""
        self.robotPose_nav_z = ""
        self.signal_lift = 0
        self.signal_volume = 0
        self.signal_charge = 0
        self.app_button.vs_speed = 50
        # self.name_card = rospy.get_param("name_card", "wlo2")
        
        self.name_card = "wlo2"
        
        self.shelves_x = 0.0
        self.shelves_y = 0.0
        self.offset = 0.0
        self.Request = NN_cmdRequest()
        
        self.degree_z = 0.0
        self.Request.target_id = 0
        self.Request.target_x = 0.0
        self.Request.target_y = 0.0
        self.Request.target_z = 0.0
        self.Request.before_mission = 0
        self.Request.after_mission = 0
        self.Request.list_id = [0,0,0,0,0]
        self.Request.list_x = [0.0,0.0,0.0,0.0,0.0]
        self.Request.list_y = [0.0,0.0,0.0,0.0,0.0]
        self.Request.list_speed = [0,0,0,0,0]
        self.Request.offset = 0.0
        self.test = "hello world"
        self.speed = 0
        self.Start_getPoint = 0


        
#RUN
    def run(self):
        self.rate = rospy.Rate(20)
        while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
            # self.Request.offset = self.offset()
            self.controlAll()
            self.pos_z()
            self.pub_button.publish(self.app_button)
            self.send_mission()
            self.speed = abs(self.raw_vel.twist.twist.linear.x)
            self.rate.sleep()
        self.is_exist = 0

    def Disconnect_traffic(self, mode):
        if mode == 0:
            subprocess.Popen(["rosnode", "kill", "/stiClient"])
        elif mode == 1:
            subprocess.Popen(["roslaunch", "sti_control", "stiClient.launch"])
    
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################

# gui du lieu len cmdRequest
    def send_mission(self):
        z = math.sqrt((float(self.Request.target_x - self.shelves_x)**2) + (float(self.Request.target_y - self.shelves_y)**2))
        
        self.Request.offset = z
        if self.Start_getPoint == 0:
            pass
        else:
            self.pub_get_point.publish(self.Request)
        
    def stop_mission(self):
        self.cancelMission_control.data = 1
        self.pub_cancelMission.publish(self.cancelMission_control)
        print("stop mission")
        
#################################################################################################
#################################### Control button in manual ###################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################

    def move_up(self):
        self.app_button.bt_backwards = False
        self.app_button.bt_forwards = True
        self.app_button.bt_rotation_left = False
        self.app_button.bt_rotation_right = False
        self.app_button.bt_stop = False
        print("up")
        
    def move_down(self):
        self.app_button.bt_backwards = True
        self.app_button.bt_forwards = False
        self.app_button.bt_rotation_left = False
        self.app_button.bt_rotation_right = False
        self.app_button.bt_stop = False
        print("down")

    def move_left(self):
        self.app_button.bt_backwards = False
        self.app_button.bt_forwards = False
        self.app_button.bt_rotation_left = True
        self.app_button.bt_rotation_right = False
        self.app_button.bt_stop = False
        print("left")
        
    def move_right(self):
        self.app_button.bt_backwards = False
        self.app_button.bt_forwards = False
        self.app_button.bt_rotation_left = False
        self.app_button.bt_rotation_right = True
        self.app_button.bt_stop = False
        print("right")

    def move_stop(self):
        self.app_button.bt_backwards = False
        self.app_button.bt_forwards = False
        self.app_button.bt_rotation_left = False
        self.app_button.bt_rotation_right = False
        self.app_button.bt_stop = True
        print("stop")

    def up_speed(self):
        if self.app_button.vs_speed < 100:
            self.app_button.vs_speed += 10
        print("speed up")
    def down_speed(self):
        if self.app_button.vs_speed > 10:
            self.app_button.vs_speed -= 10
        print("speed down")
    
    def Manual_lift_up(self):
        self.app_button.bt_lift_up = True
        self.app_button.bt_lift_down = False
    def Manual_lift_down(self):
        self.app_button.bt_lift_up = False
        self.app_button.bt_lift_down = True
    def Manual_lift_stop(self):
        self.app_button.bt_lift_up = False
        self.app_button.bt_lift_down = False
        
    def Manual_brake(self):
        if self.app_button.bt_disableBrake == True:
            self.app_button.bt_disableBrake = False
        else:
            self.app_button.bt_disableBrake = True

    def Manual_volume(self):
        if self.app_button.bt_spk_on == False:
            self.app_button.bt_spk_on = True
            self.app_button.bt_spk_off = False
        elif self.app_button.bt_spk_on == True:
            self.app_button.bt_spk_on = False
            self.app_button.bt_spk_off = True

    def reset_EMC(self):
        self.app_button.bt_clearError = True
    def Unreset_EMC(self):
        self.app_button.bt_clearError = False
        
    def Manual_charge(self):
        if self.app_button.bt_chg_on == False:
            self.app_button.bt_chg_on = True
            self.app_button.bt_chg_off = False
        elif self.app_button.bt_chg_on == True:
            self.app_button.bt_chg_on = False
            self.app_button.bt_chg_off = True
    
    def To_hand(self):  #Information.mode = 1
        self.app_button.bt_passHand = True
        self.app_button.bt_passAuto = False
        
    def To_auto(self):  #Information.mode = 2
        self.app_button.bt_passAuto = True
        self.app_button.bt_passHand = False
        
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################

#Control all
#PUB
    def Pub_topic(self):
        self.pub_cancelMission = rospy.Publisher("/cancelMission_control", Int16, queue_size = 4)
        self.cancelMission_control = Int16()
        # --
        self.pub_button = rospy.Publisher("/app_button", App_button, queue_size = 4)
        self.app_button = App_button()
        
        #-- 
        self.pub_get_point = rospy.Publisher("/NN_cmdRequest", NN_cmdRequest, queue_size = 10)
        
#SUB
    def Sub_topic(self):
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

        # -- CPD Boad
        rospy.Subscriber("/lift_status", Lift_status, self.callback_OC_board) # lay thong tin trang thai mach dieu khien ban nang.
        self.OC_status = Lift_status()

        # -- Status Port
        rospy.Subscriber("/status_port", Status_port, self.callback_statusPort) 
        self.status_port = Status_port()

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

        rospy.Subscriber("/status_port", Status_port, self.callBack_port)
        self.status_port = Status_port()

        rospy.Subscriber("/cancelMission_status", Int16, self.callBack_cancelMission)
        self.cancelMission_status = Int16()
        # --
        self.pub_cancelMission = rospy.Publisher("/cancelMission_control", Int16, queue_size = 4)
        self.cancelMission_control = Int16()

        rospy.Subscriber("/nav350_reflectors", Reflector_array, self.callback_nav350Reflectors) 
        self.nav350_reflectors = Reflector_array()
        self.arrReflector = []
        
        # mach main
        rospy.Subscriber("/POWER_request", POWER_request, self.callback_mainRequest)
        self.main_request = POWER_request()
        
        #raw vel tinh van toc
        rospy.Subscriber("/raw_vel", TwistWithCovarianceStamped, self.callback_rawVel)
        self.raw_vel = TwistWithCovarianceStamped()
#CALLBACK
    def callback_rawVel(self,data):
        self.raw_vel = data

    def callback_mainRequest(self, data):
        self.main_request = data

    def callback_nav350Reflectors(self, data):
        self.nav350_reflectors = data
        # self.anlis_ref2()
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
        
    def callback_OC_board(self, data):
        self.OC_status = data

    def callback_statusPort(self, data):
        self.status_port = data

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

    def callBack_launch(self, data):
        self.status_launch = data

    def callBack_port(self, data):
        self.status_port = data

#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################

#PROCESS ERROR
    def process_error(self):
        lg_err = len(self.NN_infoRespond.listError)
        self.listError = []
        if (lg_err == 0):
            self.listError.append( self.convert_errorAll(0))
        else:
            length = len(self.listError)
            if length > 15:
                self.listError = []
            # -
            for i in range(lg_err):
                self.listError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )
                # -
                if self.NN_infoRespond.listError[i] < 400:
                    self.listError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )

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
    
    def pos_z(self):
        angle = self.quaternion_to_euler(self.robotPose_nav.pose.orientation)
        if angle < 0:
            angle_robot = 2*pi + angle
        else:
            angle_robot = angle
        self.angle = str( round( degrees(angle_robot), 3) )
        # --
        self.target = str(self.NN_cmdRequest.target_id) + "\n" + str(round(self.NN_cmdRequest.target_x, 3)) + "\n" + str(round(self.NN_cmdRequest.target_y, 3)) + "\n" + str(round(degrees(self.NN_cmdRequest.target_z), 2)) + "\n" + str(round(self.NN_cmdRequest.offset, 3))

    def controlAll(self):
        # -- Mode show
        if (self.NN_infoRespond.mode == 0):   # - launch
            self.mode = 0

        elif (self.NN_infoRespond.mode == 1): # -- md_by_hand
            self.mode = 1

        elif (self.NN_infoRespond.mode == 2): # -- md_auto
            self.mode = 2

        # # -- Battery
        if (self.main_info.charge_current > 0.1):
            self.Status_battery = 4
        else:
            if (self.main_info.voltages < 23.5):
                self.Status_battery = 3
            elif (self.main_info.voltages >= 23.5 and self.main_info.voltages < 24.5):
                self.Status_battery = 2
            else:
                self.Status_battery = 1

        bat = round(self.main_info.voltages, 1)
        if bat > 25.5:
            bat = 25.5
        self.Info_battery = "  " + str(bat)

        # -- status AGV
        lg_err = len(self.NN_infoRespond.listError)
        self.listError = []
        if (lg_err == 0):
            self.listError.append( self.convert_errorAll(0))
        else:
            length = len(self.listError)
            if length > 15:
                self.listError = []
            # -
            for i in range(lg_err):
                self.listError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )
                # -
                if self.NN_infoRespond.listError[i] < 400:
                    self.listError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )
        # -
        self.lbv_name_agv = self.NN_infoRequest.name_agv
        self.lbv_numbeReflector = str(self.nav350_data.number_reflectors)
        self.lbv_reflectorDetect = str(self.nav350_reflectors.num_reflector)

        # -- dung cho show info
        self.robotPose_nav_x = str(round(self.robotPose_nav.pose.position.x, 3))
        self.robotPose_nav_y = str(round(self.robotPose_nav.pose.position.y, 3))
        
        #Dung cho get point
        self.float_RobotPose_x = round(self.robotPose_nav.pose.position.x, 3)
        self.float_RobotPose_y = round(self.robotPose_nav.pose.position.y, 3)
        angle = self.quaternion_to_euler(self.robotPose_nav.pose.orientation)
        
        
        if angle < 0:
            angle_robot = 2*pi + angle
        else:
            angle_robot = angle
        self.robotPose_nav_z = str( round( degrees(angle_robot), 3) )
        
        #Dung cho get point
        self.float_RobotPose_z = round( degrees(angle_robot), 3)
        # --
        self.target_info = "None"
        # # -
        self.target_info = str(self.NN_cmdRequest.target_id) + "\n" + str(round(self.NN_cmdRequest.target_x, 3)) + "\n" + str(round(self.NN_cmdRequest.target_y, 3)) + "\n" + str(round(degrees(self.NN_cmdRequest.target_z), 2)) + "\n" + str(round(self.NN_cmdRequest.offset, 3))
        self.before = str(self.NN_cmdRequest.before_mission)
        self.after = str(self.NN_cmdRequest.after_mission)

    def convert_position(self, distance, angle):
        x = 0
        y = 0
        x = distance*cos(angle)
        y = distance*sin(angle)
        return x, y

    def anlis_ref2(self):
        self.arrReflector = []

        max_x = -1000
        max_y = -1000

        length = self.nav350_reflectors.num_reflector
        for i in range(length):
            dis = self.nav350_reflectors.reflectors[i].Polar_Dist/1000.
            ang0 = radians(self.nav350_reflectors.reflectors[i].Polar_Phi/1000.)
            ang = self.limitAngle(ang0)

            p_x, p_y = self.convert_position(dis, ang)

        if max_x < abs(p_x):
            max_x = abs(p_x)

        if max_y < abs(p_y):
            max_y = abs(p_y)

        rate_show = 0.0
        rate_xy = float(max_x/max_y)
        if rate_xy > 0.5:
            rate_show = 1.0 # (max_x*1000)/431.
        else:
            rate_show = 1.0 # (max_y*1000)/811.
        # ----
        
        rate_show = 0.07
        # print ("----------")
        for i in range(length):
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

    #Show error
    def controlColor(self):
        self.safety_above = self.safety_NAV.data
        # -- HC_info
        self.safety_ahead = self.HC_info.zone_sick_ahead
        self.safety_behind = self.HC_info.zone_sick_behind
        # --
        self.statusColor.lbc_button_clearError = self.main_info.stsButton_reset
        self.statusColor.lbc_button_power = self.main_info.stsButton_power
        self.statusColor.lbc_emg = self.main_info.EMC_status
        self.statusColor.lbc_blsock = self.HC_info.vacham

        # -- Port
        self.statusColor.lbc_port_rtc    = self.status_port.rtc
        self.statusColor.lbc_port_rs485  = self.status_port.driverall
        self.statusColor.lbc_port_nav350 = self.status_port.nav350

        # --
        self.statusColor.lbc_limit_up = self.OC_status.sensorUp.data
        self.statusColor.lbc_limit_down = self.OC_status.sensorDown.data
        self.statusColor.lbc_detect_lifter = self.OC_status.sensorLift.data
    

#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################


    def convert_vel(self):
        linear_velocity = self.raw_vel.linear.x  # Lấy vận tốc tuyến tính từ linear.x
        angular_velocity = self.raw_vel.angular.z  # Lấy vận tốc góc từ angular.z
        return linear_velocity, angular_velocity

        
        
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#JOB tren traffic gui xuong o che o tu dong
    def show_job(self, val):
        job_now = 'Không\nXác Định'
        switcher={
            0:'...', #
            1:'Kiểm Tra Lại Nhiệm Vụ', # 
            2:'Thực Hiện Nhiệm Vụ Trước', # 
            3:'Kiểm Tra Trạng Thái Kệ',
            4:'Di Chuyển Ra Khỏi Vị Trí', # 
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

#CONVERT ERROR
    def convert_errorAll(self, val):
        switcher={
            0:'AGV Hoạt Động Bình Thường',
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
            477:'Không Có Kệ Tại Vị Trí', # 
            411:'Vướng Vật Cản - Di Chuyển Giữa Các Điểm', #
            412:'Vướng Vật Cản - Di Chuyển Vào Vị Trí Kệ', # 
            431:'AGV Không Giao Tiếp Với Phần Mềm Traffic', #
            451:'Điện Áp Của AGV Đang Rất Thấp', # 
            452:'AGV Không Sạc Được Pin', #
            453:'Không Phát Hiện Được Đủ Gương', #

        }
        return switcher.get(val, 'UNK')
    
#MISSION
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

class Reflector:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.localID = 0
		self.globalID = 0
