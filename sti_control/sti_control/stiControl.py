#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Phùng Quý Dương
Company: STI Viet Nam
Date  : 20/03/2025

--> Information
    + Code AGV AD sử dụng cho AGV Line từ demo

"""

import sys
import os
import re
import json
import time
import subprocess
import rclpy
from rclpy.node import Node

from decimal import *

# from tf_transformations import euler_from_quaternion, quaternion_from_euler
# import tf_transformations
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Twist
from std_msgs.msg import Int16, Int8, Bool
# import pretty_errors
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from message_pkg.msg import *
from sensor_msgs.msg import Imu
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees
#--------------------------------------------------------------------------------- ROS

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
    
class ros_control(LifecycleNode):
    def __init__(self):
        super().__init__('stiControl')
        self.get_logger().info("ROS 2 Node Initialized!")
        self.killnode = 0

        self.declare_parameters(

            namespace='',
            parameters=[
                ('rate', 30),
                ('enb_debug', 0)
            ]
        )
        self.rate = self.get_parameter('rate').value
        self.enb_debug = self.get_parameter('enb_debug').value

        ############################### Subscribe topic #################
        # -- cancel mission
        self.create_subscription(Int16, "/cancelMission_control", self.callback_cancelMission, 1)
        self.cancelMission_control = Int16()		
        self.flag_cancelMission = 0
        self.status_cancel = 0

        # -- MAIN - POWER
        self.create_subscription(PowerInfo, "/power_info", self.callback_main, 1)
        self.pub_requestMain = self.create_publisher(PowerRequest, "/power_request", 10)
            
        self.main_info = PowerInfo()
        self.power_request = PowerRequest()
        self.timeStampe_main = time.time()  
        self.voltage = 24.5
        self.killnode = 0

        # -- HC 82
        self.create_subscription(HcInfo, "/hc_info", self.callback_hc, 1)
        self.pub_HC = self.create_publisher(HcRequest, "/hc_request", 10)	# dieu khien den bao va den ho tro camera
        self.HC_info = HcInfo()
        self.HC_request = HcRequest()
        self.timeStampe_HC = time.time()

        # self.pub_LED = self.create_publisher(Int8, "/led_query", 10)
        # self.led_query = Int8()

        # -- OC 82
        self.create_subscription(LiftStatus, "/lift_status", self.callback_oc, 1)
        self.lift_status = LiftStatus()
        self.timeStampe_OC = time.time()

        self.pub_OC = self.create_publisher(LiftRequest, "/lift_request", 10)	# Dieu khien ban nang.
        self.lift_control = LiftRequest()

        # MAGNETIC LINE
        self.create_subscription(MagneticLine, "/magneticLine_front", self.callback_magline_front, 1) # lay thong tin trang thai mach dieu khien ban nang.
        self.magline_front = MagneticLine()
        self.timeStampe_magline_front = time.time()

        self.create_subscription(MagneticLine, "/magneticLine_behind", self.callback_magline_behind, 1) # lay thong tin trang thai mach dieu khien ban nang.
        self.magline_behind = MagneticLine()
        self.timeStampe_magline_behind = time.time()

        # RFID READER
        self.create_subscription(RFID, "/rfid_front_respond", self.callback_rfid_front, 1) # lay thong tin trang thai mach dieu khien ban nang.
        self.rfid_front = RFID()
        self.timeStampe_rfid_front = time.time()

        self.create_subscription(RFID, "/rfid_behind_respond", self.callback_rfid_behind, 1) # lay thong tin trang thai mach dieu khien ban nang.
        self.rfid_behind = RFID()
        self.timeStampe_rfid_behind = time.time()

        # -- App
        self.create_subscription(AppButtonAgvmag, "/app_button", self.callback_appButton, 1) # lay thong tin trang thai mach dieu khien ban nang.
        self.app_button = AppButtonAgvmag()

        # -- Communicate with Server
        self.create_subscription(NNcmdRequest, "/NN_cmdRequest", self.callback_cmdRequest, 1)
        self.NN_cmdRequest = NNcmdRequest()

        # -- Safety Zone
        self.create_subscription(ZoneLidar2Head, "/safety_zone", self.callback_safetyZone, 1)
        self.zoneRobot = ZoneLidar2Head()
        self.is_readZone = 0

        # -- Port physical
        self.create_subscription(StatusPort, "/status_port", self.callback_port, 1)
        self.status_port = StatusPort()

        # -- Traffic request info
        self.create_subscription(NNinfoRequest, "/NN_infoRequest", self.callback_infoRequest, 1)
        self.NN_infoRequest = NNinfoRequest()
        self.timeStampe_server = time.time()

        # -- move respond
        self.create_subscription(MoveRespond, "/respond_move", self.callback_moveRespond, 1)
        self.move_respond = MoveRespond() # sub from move_base
        self.timeStampe_statusGoalControl = time.time()

        # -- Driver1
        self.create_subscription( DriverRespond, '/driver1_respond', self.callback_driver1, 10)
        self.driver1_respond = DriverRespond()
        self.timeStampe_driver1 = time.time()

        # -- Driver2
        self.create_subscription( DriverRespond, '/driver2_respond', self.callback_driver2, 10)
        self.driver2_respond = DriverRespond()
        self.timeStampe_driver2 = time.time()

        # DRIVER
        self.create_subscription(McInfo, "/mc_info", self.callback_mc, 1)
        self.mc_info = McInfo()
        self.timeStampe_mc = time.time()

        # calller info
        self.create_subscription(Int8, "/respond_callDevice", self.callback_respondCallDevice, 1)
        self.data_callDevice = 0

        ########################### Publish Topic #####################
        # cancel mission request
        self.pub_cancelMission = self.create_publisher(Int16, "/cancelMission_status", 10)	
        self.cancelMission_status = Int16()	

        # cmd vel
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.velPs2 = Twist()

        # - request move
        self.pub_moveReq = self.create_publisher(MoveRequest, "/request_move", 10)
        self.move_req = MoveRequest()
        self.enb_move = 0                    # cho phep navi di chuyen
        self.status_stop = 0

        # - AGV info
        self.pub_infoRespond = self.create_publisher(NNinfoRespond, "/NN_infoRespond", 10)
        self.NN_infoRespond = NNinfoRespond()

        # -- task driver
        self.pub_taskDriver = self.create_publisher(Int16, "/task_driver", 20)		
        self.task_driver = Int16()
        self.taskDriver_nothing = 0
        self.taskDriver_resetRead = 1
        self.taskDriver_Read = 2
        self.task_driver.data = self.taskDriver_Read

        # - brake
        self.pub_disableBrake = self.create_publisher(Int8, "/disable_brake", 20)		
        self.disable_brake = Int8()

        # caller request
        self.pub_controlCallDevice = self.create_publisher(Int8, "/request_callDevice", 10)				
        self.callDevice_query = Int8()
        
        # setup timer 
        self.timer = self.create_timer(1.0/self.rate, self.run)

        ############################### VARIABLES ########################
        self.flag_requirResetparking = 0 # do truoc do co loi Parking: ko nhin thay Tag
        self.completed_backward = 0

        # 	
        # -- HZ	
        self.FrequencePubBoard = 10.
        self.pre_timeBoard = 0
        # -- Mode operate
        self.mode_by_hand = 1
        self.mode_auto = 2
        self.mode_operate = self.mode_by_hand    # Lưu chế độ hoạt động.

        # -- Target
        self.target_x = 0.		 # lưu tọa độ điểm đích hiện tại.			
        self.target_y = 0.
        self.target_z = 0.
        self.target_tag = 0.
        self.target_dir = 0
        self.moving_dir = 0
        self.target_id = 0

        self.process = -1        # tiến trình đang xử lý
        self.before_mission = 0  # nhiệm vụ trước khi di chuyển.
        self.after_mission = 0   # nhiệm vụ sau khi di chuyển.

        self.completed_before_mission = 0	 # Báo nhiệm vụ trước đã hoàn thành.
        self.completed_after_mission = 0	 # Báo nhiệm vụ sau đã hoàn thành.
        self.completed_move = 0			 	 # Báo di chuyển đã hoàn thành.
        self.completed_moveSimple = 0      # bao da den dich.
        self.completed_moveSpecial = 0     # bao da den aruco.
        self.completed_reset = 0             # hoan thanh reset.
        self.completed_MissionSetpose = 0 	#
        self.completed_checkLift = 0 	# kiem tra ke co hay ko sau khi nang. 
        # -- Flag
        self.flag_afterChager = 0
        self.flag_checkLiftError = 0 # cờ báo ko có kệ khi nâng. 
        self.flag_Auto_to_Byhand = 0
        self.flag_read_client = 0
        self.flag_error = 0
        self.flag_warning = 0
        self.pre_mess = ""               # lưu tin nhắn hiện tại.

        # -- Check:
        self.is_get_pose = 0
        self.is_mission = 0
        self.is_read_prepheral = 0       # ps2 - sti_read
        self.is_request_client = 0
        self.is_set_pose = 0
        self.is_zone_lidar = 0

        self.completed_wakeup = 0        # khi bật nguồn báo 1 sau khi setpose xong.
        # -- Status to server:
        self.statusU300L = 0
        self.statusU300L_ok = 0
        self.statusU300L_warning = 1
        self.statusU300L_error = 2	
        self.statusU300L_cancelMission = 5	
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
        self.EMC_resetOn = True
        self.EMC_resetOff = False
        self.EMC_reset = self.EMC_resetOff
        # -- EMC write
        self.EMC_writeOn = True
        self.EMC_writeOff = False
        self.EMC_write = self.EMC_writeOff

        # -- Mission server
        self.STATUSTASK_LIFTERROR = 64 # trang thái nâng kệ nhueng ko có kệ.
        self.SERVERMISSION_LIFTUP = 65 # 1 65
        self.SERVERMISSION_LIFTUP_WAIT = 3
        self.SERVERMISSION_LIFTDOWN = 66 # 2 66
        self.SERVERMISSION_CHARGER = 6
        self.SERVERMISSION_UNKNOWN = 0
        self.SERVERMISSION_LIFTDOWN_CHARGER = 10 

        # -- Lift task.
        self.liftTask = 0
        self.liftUp = 2
        self.liftDown = 1
        self.liftStop = 0
        self.liftResetOn = 1
        self.liftResetOff = 0
        self.liftReset = self.liftStop
        self.flag_commandLift = 0
        self.liftTask_byHand = self.liftStop

        # -- Speaker
        self.speaker = 0
        self.speaker_requir = 0 # luu trang thai cua loa
        self.enb_spk = 1

        self.SPK_OFF = 0
        self.SPK_START = 1
        self.SPK_MANUAL = 2
        self.SPK_MOVE = 3
        self.SPK_CONFIRM = 4
        self.SPK_ESCALATOR = 5
        self.SPK_ERROR = 6

        # -- Led
        self.led_effect = 0
        self.LED_OFF = 0
        self.LED_ERROR = 1 			# 1
        self.LED_SIMPLERUN = 2 		# 2
        self.LED_SPECIALRUN = 3 	# 3
        self.LED_PERFORM = 4 		# 4
        self.LED_COMPLETED = 5  	# 5	
        self.LED_STOPBARRIER = 6  	# 6

        # -- Charger
        self.CHARGER_ON = True
        self.CHARGER_OFF = False		
        self.charger_requir = self.CHARGER_OFF
        self.charger_write = self.charger_requir
        self.charger_valueOrigin = 0.2
        # -- Voltage
        self.timeCheckVoltage_charger = 1800 # s => 30 minutes.
        self.timeCheckVoltage_normal = 60     # s
        self.pre_timeVoltage = 0   # s
        self.valueVoltage = 0
        self.step_readVoltage = 0
        # -- Cmd_vel
        self.vel_ps2 = Twist()       # gui toc do
        # -- ps2 
        self.time_ht = time.time()
        self.time_tr = time.time()
        self.rate_cmdvel = 10. 
        # -- Error Type
        self.error_move = 0
        self.error_perform = 0
        self.error_device = 0  # camera(1) - MC(2) - Main(3) - SC(4)

        self.numberError = 0
        self.lastTime_checkLift = 0.0
        # -- add new
        self.enb_debug = 1
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
        self.timeStampe_reflectors = time.time()
        # -- add 23/12/2021:
        self.pose_parkingRuning = Pose()
        print ("launch")
        # -- add 27/12/2021
        self.cancelbackward_pose = Pose()
        self.cancelbackward_offset = 0.0

        # -- add 18/01/2022 : sua loi di lai cac diem cu khi mat ket noi server.
        self.list_id_unknown = [0, 0, 0, 0, 0]
        self.flag_listPoint_ok = 0

        # -- add 19/01/2022 : Check error lost server.
        self.name_card = "wlp0s20f3"
        # self.name_card = "wlo2"
        self.address = "192.168.1.25" # "172.21.15.224"
        self.saveTime_checkServer = time.time()
        self.saveStatus_server = 0
        # -- add 30/03/2022 : co bao loi qua tai dong co.
        self.flagError_overLoad = 0
        # -- add 15/04/2022
        self.flag_listPointEmpty = 0

        # --
        self.flag_remote = 0
        self.flag_magline = 0
        self.flag_Byhand_to_auto = 0

        #-- 
        self.save_liftStatus = 0 # lưu trạng thái bàn nâng gần nhất ở chế độ tự động

        # -- 
        self.sub_processWait = 0

        # --
        self.save_cartDir = 0 # lưu trạng thái hướng kệ
        self.ID_WH = 281 # id warehouse
        self.listDir_ID_Trolley=[[292, 2], [345, 1]]

        # -- read data backup
        self.time_writeBackup = time.time()
        self.file_backup = '/home/stivietnam/ros2_ws/src/sti_control/data/backup.json'
        try:
            with open(self.file_backup, 'r') as file:
                data = json.load(file)
                self.save_liftStatus = data["lift_status"]
                self.save_cartDir = data["cart_dir"]

                print("lift_status: ",  self.save_liftStatus)
                print("cart_dir: ",  self.save_cartDir)

        except Exception as e:
            print("Read json file backup error ", e)

    def callback_moveRespond(self, data):
        self.move_respond = data
        self.timeStampe_statusGoalControl = time.time()

    def callback_cancelMission(self, dat):
        self.cancelMission_control = dat

    def callback_infoRequest(self, dat):
        self.NN_infoRequest = dat
        self.timeStampe_server = time.time()

    def callback_reconnect(self, dat):
        self.status_reconnect = dat

    def callback_main(self, dat):
        self.main_info = dat
        self.voltage = round(self.main_info.voltages, 2)
        self.timeStampe_main = time.time()

    def callback_hc(self, dat):
        self.HC_info = dat
        self.timeStampe_HC = time.time()

    def callback_oc(self, dat):
        self.lift_status = dat
        self.timeStampe_OC = time.time()

    def callback_magline_front(self, dat):
        self.magline_front = dat
        self.timeStampe_magline_front = time.time()

    def callback_magline_behind(self, dat):
        self.magline_front = dat
        self.timeStampe_magline_behind = time.time()

    def callback_rfid_front(self, dat):
        self.rfid_front = dat
        self.timeStampe_rfid_front = time.time()

    def callback_rfid_behind(self, dat):
        self.rfid_behind = dat
        self.timeStampe_rfid_behind = time.time()

    def callback_appButton(self, dat):
        self.app_button = dat

    def callback_cmdRequest(self, dat):
        self.NN_cmdRequest = dat
        self.timeStampe_server = time.time()
        # -- add 18/01/2022
        self.flag_listPoint_ok = 0

    def callback_infoRequest(self, dat):
        self.NN_infoRequest = dat

    def callback_safetyZone(self, dat):
        self.zoneRobot = dat
        self.is_readZone = 1

    def callback_port(self, dat):
        self.status_port = dat

    # def callback_NNstatusOtherSystem(self, dat):
    # 	self.data_NNstatusOtherSystem = dat

    def callback_driver1(self, dat):
        self.driver1_respond = dat
        self.timeStampe_driver1 = time.time()

    def callback_driver2(self, dat):
        self.driver2_respond = dat
        self.timeStampe_driver2 = time.time()


    def callback_respondCallDevice(self, data):
        self.data_callDevice = data.data

    def callback_mc(self, dat):
        self.mc_info = dat
        self.timeStampe_mc = time.time()

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
                    self.get_logger().info(f"{mess}: {val}")
                elif typ == "warn":
                    self.get_logger().warn(f"{mess}: {val}")
                else:
                    self.get_logger().error(f"{mess}: {val}")
            self.pre_mess = mess

    def pub_move_req(self, ena, ls, status_stop):
        req_move = MoveRequest()

        req_move.enable = ena
        req_move.target_id = ls.target_id
        req_move.target_dir = ls.target_dir
        req_move.moving_dir = ls.moving_dir

        req_move.list_id = ls.list_id
        req_move.list_code = ls.list_code
        req_move.list_dir = ls.list_dir
        req_move.list_speed = ls.list_speed
        req_move.list_encoder = ls.list_encoder

        #req_move.status_stop = status_stop

        req_move.mission = ls.mission
        req_move.speed_move_by_hand = ls.speed_move_by_hand

        self.pub_moveReq.publish(req_move)

    def convertLed(self, ipt):
        switcher={
            0:0,
            1:5,
            2:1,
            3:2,
            4:6,
            5:3,
            6:4
        }
        return switcher.get(ipt, 0)

    def pub_cmdVel(self, twist , rate , time):
        self.time_ht = time 
        # print self.time_ht - self.time_tr
        # print 1/float(rate)
        if self.time_ht - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = self.time_ht
            self.pub_vel.publish(twist)
        # else :
            # rospy.logwarn("Hz /cmd_vel OVER !! - %f", 1/float(self.time_ht - self.time_tr) )  

    def Main_pub(self, charge, sound, EMC_write, EMC_reset):
        # charge, sound_on, sound_type, EMC_write, EMC_reset, OFF_5v , OFF_22v, led_button1, led_button2, a_coefficient , b_coefficient 
        mai = PowerRequest()
        mai.charge = charge
        if sound == 0:
            mai.sound_on = False
        else:
            mai.sound_on = True
            mai.sound_type = sound
        mai.emg_write = EMC_write
        mai.emg_reset = EMC_reset
        
        self.pub_requestMain.publish(mai)

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

    def pub_park(self, modeRun, poseBefore, poseTarget, offset):
        park = Parking_request()
        park.modeRun = modeRun
        park.poseBefore = poseBefore
        park.poseTarget = poseTarget
        park.offset = offset

        self.pub_parking.publish(park)

    def euler_to_quaternion(self, euler):
        quat = Quaternion()
        odom_quat = quaternion_from_euler(0, 0, euler)
        quat.x = odom_quat[0]
        quat.y = odom_quat[1]
        quat.z = odom_quat[2]
        quat.w = odom_quat[3]
        return quat

    def readbatteryVoltage(self): # 
        # time_curr = time.time()
        # delta_time = (time_curr - self.pre_timeVoltage)
        # if self.charger_requir == self.CHARGER_ON:
        # 	self.flag_afterChager = 1
        # 	if self.step_readVoltage == 0:  # bat sac.
        # 		self.charger_write = self.CHARGER_ON
        # 		if (delta_time > self.timeCheckVoltage_charger):
        # 			self.pre_timeVoltage = time_curr
        # 			self.step_readVoltage = 1

        # 	elif self.step_readVoltage == 1: # tat sac va doi.
        # 		self.charger_write = self.CHARGER_OFF
        # 		if (delta_time > self.timeCheckVoltage_normal*3):
        # 			self.pre_timeVoltage = time_curr
        # 			self.step_readVoltage = 2

        # 	elif self.step_readVoltage == 2: # do pin.	
        # 		bat = round(self.main_info.voltages, 1)*10
        # 		# print "charger --"
        # 		if  bat > 255:
        # 			self.valueVoltage = 255
        # 		elif bat < 0:
        # 			self.valueVoltage = 0
        # 		else:
        # 			self.valueVoltage = bat

        # 		self.pre_timeVoltage = time_curr
        # 		self.step_readVoltage = 3

        # 	elif self.step_readVoltage == 3: # doi.
        # 		if (delta_time > 2):
        # 			self.pre_timeVoltage = time_curr
        # 			self.step_readVoltage = 0

        # elif self.charger_requir == self.CHARGER_OFF:
        # 	if self.flag_afterChager == 1:   # sau khi tat sac doi T s roi moi do dien ap.
        # 		self.pre_timeVoltage = time_curr
        # 		self.flag_afterChager = 0
        # 		self.charger_write = self.CHARGER_OFF
        # 		self.step_readVoltage = 0
        # 	else:
        # 		if (delta_time > self.timeCheckVoltage_normal):
        # 			self.pre_timeVoltage = time_curr

        bat = round(self.main_info.voltages, 1)*10
        # print "normal --"
        if  bat > 255:
            self.valueVoltage = 255
        elif bat < 0:
            self.valueVoltage = 0
        else:
            self.valueVoltage = int(bat)

    def run_manual(self):
        cmd_vel = Twist()
        val_linear = (self.app_button.vs_speed/100.)*0.35
        val_rotate = (self.app_button.vs_speed/100.)*0.3

        if (self.app_button.bt_movehand == 1):
            if (self.zoneRobot.zone_ahead == 1):
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
            else:
                cmd_vel.linear.x = val_linear
                cmd_vel.angular.z = 0.0

        if (self.app_button.bt_movehand == 2):
            if (self.zoneRobot.zone_behind == 1):
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
            else:
                cmd_vel.linear.x = -val_linear
                cmd_vel.angular.z = 0.0

        if (self.app_button.bt_movehand == 3):
            self.move_req.speed_move_by_hand = 0
            if (self.zoneRobot.zone_ahead == 1 or self.zoneRobot.zone_behind == 1):
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0				
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = val_rotate

        if (self.app_button.bt_movehand == 4):
            self.move_req.speed_move_by_hand = 0
            if (self.zoneRobot.zone_ahead == 1 or self.zoneRobot.zone_behind == 1):
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0				
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = -val_rotate

        if (self.app_button.bt_movehand == 0):
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        return cmd_vel

    def quaternion_to_euler(self, qua):
        # quat = (qua.x, qua.y, qua.z, qua.w )
        a, b, euler = euler_from_quaternion(qua.x, qua.y, qua.z, qua.w )
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

    def detectLost_goalControl(self):
        delta_t = time.time() - self.timeStampe_statusGoalControl
        if (delta_t.to_sec() > 2):
            return 1
        return 0

    def detectLost_magline_front(self):
        delta_t = time.time() - self.timeStampe_magline_front
        if (delta_t.to_sec() > 2):
            return 1
        return 0

    def detectLost_magline_behind(self):
        delta_t = time.time() - self.timeStampe_magline_behind
        if (delta_t.to_sec() > 2):
            return 1
        return 0

    def detectLost_rfid_front(self):
        delta_t = time.time() - self.timeStampe_rfid_front
        if (delta_t.to_sec() > 2):
            return 1
        return 0

    def detectLost_rfid_behind(self):
        delta_t = time.time() - self.timeStampe_rfid_behind
        if (delta_t.to_sec() > 2):
            return 1
        return 0
				
	# def detectLost_mc(self):
	# 	delta_t = time.time() - self.timeStampe_mc
	# 	if (delta_t.to_sec() > 0.8):
	# 		return 1
	# 	return 0

    def detectLost_driver1(self):
        delta_t = time.time() - self.timeStampe_driver1
        if (delta_t.to_sec() > 0.8):
            return 1
        return 0

    def detectLost_driver2(self):
        delta_t = time.time() - self.timeStampe_driver2
        if (delta_t.to_sec() > 0.8):
            return 1
        return 0

    def detectLost_hc(self):
        delta_t = time.time() - self.timeStampe_HC
        if (delta_t.to_sec() > 0.4):
            return 1
        return 0

    def detectLost_oc(self):
        delta_t = time.time() - self.timeStampe_OC
        if (delta_t.to_sec() > 0.4):
            return 1
        return 0

    def detectLost_main(self):
        delta_t = time.time() - self.timeStampe_main
        if (delta_t.to_sec() > 1.0):
            return 1
        return 0

	# -- add 19/01/2020
    def get_ipAuto(self, name_card): # name_card : str()
        try:
            address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
            # print ("address: ", address)
            return 1
        except Exception:
            return 0
            
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

	# -- add 19/01/2022 : Check error lost server.
	# def detectLost_server(self):
	# 	delta_t = time.time() - self.timeStampe_server
	# 	if (delta_t.to_sec() > 15):
	# 		return 1
	# 	return 0

    def detectLost_server(self):
        delta_t = time.time() - self.timeStampe_server
        if (delta_t.to_sec() > 15):
            delta_s = time.time() - self.saveTime_checkServer
            if (delta_s.to_sec() > 5):
                self.saveTime_checkServer = time.time()
                self.saveStatus_server = self.check_server()

            if (self.saveStatus_server == 1):
                return 2
            elif (self.saveStatus_server == 2):
                return 3
            return 1
        return 0

    def find_element(self, value_find, list_in):
        lenght = len(list_in)
        for i in range(lenght):
            if (value_find == list_in[i]):
                return 1
        return 0

    def synthetic_error(self):
        listError_now = []

        # -- Goal Control
        # if self.detectLost_goalControl() == 1:
        # 	listError_now.append(282)

        # if self.detectLost_magline_front() == 1:
        # 	listError_now.append(283)

        # if self.detectLost_magline_behind() == 1:
        # 	listError_now.append(284)

        # if self.detectLost_rfid_front() == 1:
        # 	listError_now.append(285)

        # # -- EMG
        # if self.main_info.EMC_status == 1:
        # 	listError_now.append(121)

        # # -- Va cham
        # if self.HC_info.vacham == 1:
        # 	listError_now.append(122)

        # # -- lost HC
        # if (self.detectLost_hc() == 1):
        # 	listError_now.append(351)

        # # -- lost CAN HC
        # if (self.HC_info.status == -1):
        # 	listError_now.append(352)

        # # -- OC: ket noi
        # if (self.detectLost_oc() == 1):
        # 	listError_now.append(341)

        # # -- OC-CAN
        # if (self.lift_status.status == -2):
        # 	listError_now.append(342)

        # # -- MAIN: CAN ket noi 
        # if (self.main_info.CAN_status == 0):
        # 	listError_now.append(322)

        # # -- lost Main
        # if (self.detectLost_main() == 1):
        # 	listError_now.append(321)

        # # -- Lost Driver 1
        # if self.detectLost_driver1() == 1: 
        # 	listError_now.append(251)

        # #-- Error Driver 1
        # summation1 = self.driver1_respond.alarm_all + self.driver1_respond.alarm_overload + self.driver1_respond.warning
        # if (summation1 != 0):
        # 	listError_now.append(252)

        # #-- Lost Driver 2
        # if self.detectLost_driver2() == 1: 
        # 	listError_now.append(261)

        # #-- Error Driver 2
        # summation2 = self.driver2_respond.alarm_all + self.driver2_respond.alarm_overload + self.driver2_respond.warning
        # if (summation2 != 0):
        # 	listError_now.append(262)

        # # -- Ban Nang khong bat duoc cam bien
        # if (self.lift_status.status == 238 or self.lift_status.status == -1):
        # 	listError_now.append(141)

        # #-- Nâng kệ nhưng không có kệ.
        # if self.flag_checkLiftError == 1:
        # 	listError_now.append(471)

        # #-- 19/01/2022 - Mat giao tiep voi Server 
        # sts_sr = self.detectLost_server()
        # if sts_sr == 1: # lost server
        # 	listError_now.append(431)
        # elif sts_sr == 2: # lost server: Ping
        # 	listError_now.append(432)
        # elif sts_sr == 3: # lost server: IP- wifi
        # 	listError_now.append(433)

        # # --- Low battery
        # if self.voltage < 23:
        # 	listError_now.append(451)

        # #-- Co vat can khi di chuyen giua cac diem
        # if (self.move_respond.status == 2):    
        # 	listError_now.append(411)
        
        # # -- AGV dừng chờ thang máy hoặc dứng tránh AGV khác
        # if self.move_respond.status == 3:
        # 	listError_now.append(442)
        
        # # -- AGV ra khỏi line từ
        # if self.move_respond.error == 1:
        # 	listError_now.append(347)

        # # -- Thông báo AGV dừng báo cháy
        # if self.data_NNstatusOtherSystem.status_fireAlarm == 1:
        # 	listError_now.append(443)

        #-- Co vat can khi di chuyen parking
        # if (self.parking_status.warning == 1):
        # 	listError_now.append(412)

        #-- AGV dung do da di het danh sach diem.     >> Bỏ qua
        # if self.move_respond.misson == 1 or self.move_respond.misson == 3:
        # 	if self.move_respond.complete_misson == 2:
        # 		listError_now.append(441)
        # 		# -- add 18/01/2022
        # 		self.flag_listPoint_ok = 1

        return listError_now

    def resetAll_variable(self):
        # self.enb_move = 0

        # if self.parking_status.status == 11:  # khi doi lenh. no reset truoc khi parking nhan ra no da hoan thanh.
        # 	self.flag_requirBackward = 1

        self.enb_parking = 0

        self.completed_before_mission = 0
        self.completed_after_mission = 0
        self.completed_move = 0
        self.completed_moveSimple = 0
        self.completed_moveSpecial = 0
        self.completed_backward = 0
        self.completed_MissionSetpose = 0

        self.completed_checkLift = 0
        self.flag_checkLiftError = 0

        self.enb_mission = 0
        self.mission = 0
        self.lifttable = 0
        self.conveyor = 0

        # -- add 18/01/2022
        self.flag_listPoint_ok = 0

        # rospy.logwarn("Update new target from: X= %s | Y= %s to X= %s| Y= %s", self.target_x, self.target_y, self.NN_cmdRequest.target_x, self.NN_cmdRequest.target_y)
        self.target_id = self.NN_cmdRequest.target_id
        self.target_dir = self.NN_cmdRequest.target_dir
        self.target_tag = self.NN_cmdRequest.tag
        self.before_mission = self.NN_cmdRequest.before_mission
        self.after_mission = self.NN_cmdRequest.after_mission
        self.log_mess("info", "Update new target: ID_new = ", self.NN_cmdRequest.target_id)
        # self.log_mess("info", "Update new target: moving_dir_new = ", self.NN_cmdRequest.moving_dir)
        # -- add 12/11/2021
        self.flag_resetFramework = 0
        self.flag_Auto_to_Byhand = 0
        # -- add 30/03/2022 : co bao loi qua tai dong co.
        # self.flagError_overLoad = 0
        # -- add 15/04/2022
        self.flag_listPointEmpty = 0

        # -- 
        self.sub_processWait = 0

    def writeBackup(self):
        json_out = {"lift_status": self.save_liftStatus, "cart_dir": self.save_cartDir}
        temp_file_path = self.file_backup + '.tmp'
        
        try:
            with open(temp_file_path, 'w', encoding='utf-8') as temp_file:
                json.dump(json_out, temp_file, ensure_ascii=False, indent=4)
                temp_file.flush()
                os.fsync(temp_file.fileno())
            os.rename(temp_file_path, self.file_backup)

        except Exception as e:
            print("Write json file backup error", e)

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS
    
    def run(self):
        if self.process == -1: # khi moi khoi dong len
            # mode_operate = mode_hand
            time.sleep(0.2)
            self.mode_operate = self.mode_by_hand
            self.led = 0
            self.speaker_requir = self.SPK_START
            self.process = 0
            self.enb_parking = 0

        elif self.process == 0:	# chờ cac node khoi dong xong.
            ct = 8
            if ct == 8:
                self.process = 1

        elif self.process == 1: # reset toan bo: Main - MC - OC.
            self.flag_error = 0
            self.completed_before_mission = 0
            self.completed_after_mission = 0
            self.completed_move = 0
            self.completed_checkLift = 0
            self.liftTask = 0
            self.error_device = 0
            self.error_move = 0
            self.error_perform = 0
            self.process = 2

        elif self.process == 2: # kiem tra toan bo thiet bi ok.
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
                    
            # -- add 30/03/2022 : co bao loi qua tai dong co.
            if count_error == 0 and count_warning == 0: # and self.flagError_overLoad == 0:
                self.flag_error = 0
                self.flag_warning = 0

            elif count_error == 0 and count_warning > 0:
                self.flag_error = 0
                self.flag_warning = 1

            else:
                self.flag_error = 1
                self.flag_warning = 0

            if (self.flag_error == 1):
                self.statusU300L = self.statusU300L_error
            else:
                if (self.flag_warning == 1):
                    self.statusU300L = self.statusU300L_warning
                else:
                    self.statusU300L = self.statusU300L_ok

            # -- ERROR
            if self.app_button.bt_clear_error == 1 or self.main_info.status_btn_reset == 1:
                if self.flag_requirResetparking == 1:
                    self.enb_parking = 0
                    self.flag_requirResetparking = 0

                # -- dung parking neu co loi. - thay doi
                # self.enb_parking = 2

                self.EMC_write = self.EMC_writeOff
                self.EMC_reset = self.EMC_resetOn
                # -- sent clear error
                self.flag_error = 0
                self.error_device = 0
                self.error_move = 0
                self.error_perform = 0

                self.flag_checkLiftError = 0
                self.task_driver.data = self.taskDriver_resetRead

                # -- add 30/03/2022 : co bao loi qua tai dong co.
                self.flagError_overLoad = 0

                # -- xoa loi ban nang.
                if (self.lift_status.status == -1):
                    self.liftReset = self.liftResetOn
                else:
                    self.liftReset = self.liftResetOff
            else:
                self.task_driver.data = self.taskDriver_Read
                self.EMC_reset = self.EMC_resetOff

                # -- Add new: 23/12: Khi mat ket Driver, EMG duoc keo len.
                # if (self.flag_error == 1):
                # 	if (self.find_element(251, self.listError) == 1 or self.find_element(261, self.listError) == 1):
                # 		self.EMC_write = self.EMC_writeOn
                # 	else:
                # 		self.EMC_write = self.EMC_writeOff
                

            # if (self.error_device != 0 or self.error_perform != 0 or self.error_move != 0):
            # 	self.flag_error = 1	

            self.process = 3

        elif self.process == 3: # read app
            if self.app_button.bt_pass_hand == 1 and self.app_button.bt_pass_auto == 0:
                if self.mode_operate == self.mode_auto: # keo co bao dang o tu dong -> chuyen sang bang tay.
                    self.flag_Auto_to_Byhand = 1
                self.mode_operate = self.mode_by_hand
                self.enb_move = 0

            elif self.app_button.bt_pass_hand == 0 and self.app_button.bt_pass_auto == 1:
                self.mode_operate = self.mode_auto
                self.enb_move = 0

            if self.mode_operate == self.mode_by_hand:
                self.process = 30

            elif self.mode_operate == self.mode_auto:
                self.process = 40
                self.disable_brake.data = 0
    # ------------------------------------------------------------------------------------
    # -- BY HAND:
        elif self.process == 30:
            self.job_doing = 20

            # if self.parking_status.status != 0:
            # 	self.enb_parking = 0

                # ------------------------------------------------------------
            if self.flag_error == 0:
                # -- Send vel
                if self.lift_status.status == 0 or self.lift_status.status >= 3:  # Đang thực hiện nhiệm vụ ở chế độ auto -> ko cho phép di chuyển.
                    # -- Move
                    if self.app_button.ck_remote == 0:    # tắt chức năng remote
                        if self.app_button.ck_magline == 1:    # chức năng di chuyển bám line

                            self.enb_move = self.app_button.bt_movehand

                            if self.app_button.bt_movehand == 0:
                                self.move_req.speed_move_by_hand = 0
                            else:
                                self.move_req.speed_move_by_hand = self.app_button.vs_speed

                        else:
                            self.enb_move = 0
                            self.move_req.speed_move_by_hand = 0		
                            self.pub_cmdVel(self.run_manual(), self.rate_cmdvel, time.time())

                else:
                    self.enb_move = 0
                    self.move_req.speed_move_by_hand = 0
                    self.pub_cmdVel(Twist(), self.rate_cmdvel, time.time())
                    
            else: # -- Has error
                self.enb_move = 0
                self.move_req.speed_move_by_hand = 0
                self.pub_cmdVel(Twist(), self.rate_cmdvel, time.time())

            # - update lệnh traffic
            self.move_req.target_id = self.NN_cmdRequest.target_id

            if self.target_id == self.ID_WH and self.save_cartDir > 0:
                self.move_req.target_dir = self.save_cartDir
            else:
                self.move_req.target_dir = self.target_dir
            # self.move_req.target_dir = self.NN_cmdRequest.target_dir
            
            self.move_req.moving_dir = self.NN_cmdRequest.moving_dir
            self.move_req.list_id = self.NN_cmdRequest.list_id
            self.move_req.list_code = self.NN_cmdRequest.list_code
            self.move_req.list_dir = self.NN_cmdRequest.list_dir
            self.move_req.list_speed = self.NN_cmdRequest.list_speed
            self.move_req.list_encoder = self.NN_cmdRequest.list_encoder
            self.move_req.mission = self.NN_cmdRequest.offset
            
            # ------------------------------------------------------------
            # -- Lift
            if self.app_button.bt_lifter == 2:
                self.liftTask_byHand = self.liftUp
            elif self.app_button.bt_lifter == 1:
                self.liftTask_byHand = self.liftDown
            else:
                self.liftTask_byHand = self.liftStop

            self.liftTask = self.liftTask_byHand

            # -- Speaker
            if self.app_button.bt_speaker == True:
                self.enb_spk = 1
                if self.app_button.bt_setting == 1:
                    if self.app_button.soundtype == 0:
                        self.speaker_requir = self.SPK_OFF
                    elif self.app_button.soundtype == 1:
                        self.speaker_requir = self.SPK_START
                    elif self.app_button.soundtype == 2:
                        self.speaker_requir = self.SPK_MANUAL
                    elif self.app_button.soundtype == 3:
                        self.speaker_requir = self.SPK_MOVE
                    elif self.app_button.soundtype == 4:
                        self.speaker_requir = self.SPK_CONFIRM
                    elif self.app_button.soundtype == 5:
                        self.speaker_requir = self.SPK_ESCALATOR
                    elif self.app_button.soundtype == 6:
                        self.speaker_requir = self.SPK_ERROR
                else:
                    self.speaker_requir = self.SPK_MANUAL

            else:
                self.enb_spk = 0
                self.speaker_requir = self.SPK_OFF

            # led
            if self.app_button.bt_setting == 1:
                if self.app_button.ledtype == 0:
                    self.led_effect = self.LED_SIMPLERUN
                elif self.app_button.ledtype == 1:
                    self.led_effect = self.LED_ERROR
                elif self.app_button.ledtype == 2:
                    self.led_effect = self.LED_SIMPLERUN
                elif self.app_button.ledtype == 3:
                    self.led_effect = self.LED_SPECIALRUN
                elif self.app_button.ledtype == 4:
                    self.led_effect = self.LED_PERFORM
                elif self.app_button.ledtype == 5:
                    self.led_effect = self.LED_COMPLETED
                elif self.app_button.ledtype == 6:
                    self.led_effect = self.LED_STOPBARRIER
            else:
                self.led_effect = self.LED_SIMPLERUN

            # -- Charger
            # if self.app_button.bt_charger == True:
            # 	self.charger_requir = self.CHARGER_ON
            # else:
            # 	self.charger_requir = self.CHARGER_OFF

            # -- Keep shaft.
            self.disable_brake.data = self.app_button.bt_disable_brake

            if self.flag_Byhand_to_auto == 1:
                self.flag_Byhand_to_auto = 0
                self.enb_move = 0
            # --
            if self.app_button.bt_reset_framework == 1:
                self.resetAll_variable()

            self.process = 2

    # -- RUN AUTO:
        elif self.process == 40: # -- kiem tra loi
            self.flag_Byhand_to_auto = 1

            if self.flag_error == 1: # thay doi != 0
                self.job_doing = 30
                self.enb_mission = 0
                self.process = 2

                if self.find_element(121, self.listError) == 1 or self.find_element(347, self.listError) == 1:
                    self.pub_cmdVel(Twist(), self.rate_cmdvel, time.time())
                else:
                    self.enb_move = 0
                    self.pub_cmdVel(Twist(), self.rate_cmdvel, time.time())

            else:
                # self.enb_move = 0
                self.process = 41

        elif self.process == 41:    # kiem tra muc tieu thay doi
            # if self.NN_cmdRequest.after_mission == self.SERVERMISSION_LIFTUP or self.NN_cmdRequest.after_mission == 1:
            # 	is_id_inListDirCast = 0
            # 	dir_cart = 0
            # 	# -- Luu huong ke
            # 	for id in self.listDir_ID_Trolley:
            # 		if id[0] == self.NN_cmdRequest.target_id:
            # 			is_id_inListDirCast = 1
            # 			dir_cart = id[1]
            # 			break
                    
            # 	if is_id_inListDirCast:
            # 		self.save_cartDir = dir_cart
            # 		print("luu huong xe day: ", self.save_cartDir)
            # 	else:
            # 		self.save_cartDir = 0
            # 		print("reset huong xe day")
            print(self.target_id , self.NN_cmdRequest.target_id)
            if ( self.target_id != self.NN_cmdRequest.target_id) or ( self.target_dir != self.NN_cmdRequest.target_dir):
                """ Không cho phép đổi lệnh khi đang thao tác:
                    1, Nâng/Hạ.
                    2, Đang đi vào/ra điểm thao tác.
                    3, Đang đi vào/ra khỏi sạc.
                """
                a1 = 0
                a2 = 0
                # - Đang thao tác Nâng/Hạ. OK
                if self.lift_status.status == -1 or self.lift_status.status == 1 or self.lift_status.status == 2:
                    a1 = 1

                # - Đang đi vào điểm thao tác
                if self.move_respond.mode_move == 6 and self.move_respond.completed == 0:
                    a2 = 1
                        
                # --
                if a1 == 1 or a2 == 1:
                    self.job_doing = 9
                    self.log_mess("warn", "Have new target but must Waiting perform done ....", 0)
                    self.process = 2
                else:
                    self.resetAll_variable()
                    # print('Tao bi reset o day')
                    self.process = 42
            else:
                # - Nếu target ko đổi mà nhiện vụ muốn thay đổi (lấy hoặc trả hàng luôn tại đó).
                if self.completed_after_mission == 1:
                    if self.after_mission != self.NN_cmdRequest.after_mission:
                        self.completed_after_mission = 0
                        self.log_mess("info", "After mission change to ", self.NN_cmdRequest.after_mission)
                        self.after_mission = self.NN_cmdRequest.after_mission

                self.process = 42

        elif self.process == 42: 
            if self.flag_Auto_to_Byhand == 1: 
                # Bắt buộc agv phải di chuyển khi chuyển lại chế độ sang tự động
                self.completed_moveSimple = 0
                self.completed_moveSpecial = 0

                # -- 
                self.sub_processWait = 0

                # -- add 18/01/2022
                if (self.flag_listPoint_ok == 1):
                    self.NN_cmdRequest.list_id = self.list_id_unknown
                    self.flag_listPoint_ok = 0

                self.job_doing = 1
                # thuc hien lai nhiem vu nang, ha, sac sau khi chuyen che do tu tu dong sang bang tay.
                if self.completed_after_mission == 0 and self.completed_before_mission == 0:
                    if self.before_mission == self.SERVERMISSION_UNKNOWN:
                        
                        # -- Hoang them
                        if self.save_liftStatus == self.liftUp:
                            self.liftTask = self.liftUp				
                            if self.lift_status.status == 4:  # Hoàn thành
                                self.liftTask = self.liftStop
                                self.flag_Auto_to_Byhand = 0
                                self.completed_checkLift = 0

                        elif self.save_liftStatus == self.liftDown:
                            self.liftTask = self.liftDown
                            if self.lift_status.status == 3:  # Hoàn thành
                                self.liftTask = self.liftStop
                                self.flag_Auto_to_Byhand = 0

                        else:
                            self.flag_Auto_to_Byhand = 0
                        
                        # --
                    else:
                        self.flag_Auto_to_Byhand = 0

                elif self.completed_after_mission == 0 and self.completed_before_mission == 1:
                    if self.before_mission == self.SERVERMISSION_UNKNOWN:
                        # -- Hoang them
                        if self.save_liftStatus == self.liftUp:
                            self.liftTask = self.liftUp				
                            if self.lift_status.status == 4:  # Hoàn thành
                                self.liftTask = self.liftStop
                                self.flag_Auto_to_Byhand = 0
                                self.completed_checkLift = 0

                        elif self.save_liftStatus == self.liftDown:
                            self.liftTask = self.liftDown
                            if self.lift_status.status == 3:  # Hoàn thành
                                self.liftTask = self.liftStop
                                self.flag_Auto_to_Byhand = 0

                        else:
                            self.flag_Auto_to_Byhand = 0

                        # --
                        # self.flag_Auto_to_Byhand = 0
                            
                    elif self.before_mission == self.SERVERMISSION_LIFTDOWN or self.before_mission == 2: # Hạ
                        self.liftTask = self.liftDown	

                        if self.lift_status.status == 3:  # Hoàn thành
                            self.liftTask = self.liftStop
                            self.flag_Auto_to_Byhand = 0

                    elif self.before_mission == self.SERVERMISSION_LIFTUP or self.before_mission == 1: # Nâng
                        self.liftTask = self.liftUp				
                        if self.lift_status.status == 4:  # Hoàn thành
                            self.liftTask = self.liftStop
                            self.flag_Auto_to_Byhand = 0
                            self.completed_checkLift = 0

                elif self.completed_after_mission == 1 and self.completed_before_mission == 1:
                    if self.after_mission == self.SERVERMISSION_UNKNOWN:
                        # -- Hoang them
                        if self.save_liftStatus == self.liftUp:
                            self.liftTask = self.liftUp				
                            if self.lift_status.status == 4:  # Hoàn thành
                                self.liftTask = self.liftStop
                                self.flag_Auto_to_Byhand = 0
                                self.completed_checkLift = 0

                        elif self.save_liftStatus == self.liftDown:
                            self.liftTask = self.liftDown
                            if self.lift_status.status == 3:  # Hoàn thành
                                self.liftTask = self.liftStop
                                self.flag_Auto_to_Byhand = 0

                        else:
                            self.flag_Auto_to_Byhand = 0
                            
                        # --
                        # self.flag_Auto_to_Byhand = 0
                            
                    elif self.after_mission == self.SERVERMISSION_LIFTDOWN or self.before_mission == 2: # Hạ
                        self.liftTask = self.liftDown				
                        if self.lift_status.status == 3:  # Hoàn thành
                            self.liftTask = self.liftStop
                            self.flag_Auto_to_Byhand = 0

                    elif self.after_mission == self.SERVERMISSION_LIFTUP or self.before_mission == 1: # Nâng
                        self.liftTask = self.liftUp				
                        if self.lift_status.status == 4:  # Hoàn thành
                            self.liftTask = self.liftStop
                            self.flag_Auto_to_Byhand = 0

                    elif self.after_mission == self.SERVERMISSION_LIFTDOWN_CHARGER or self.after_mission == self.SERVERMISSION_CHARGER: # Hạ - Sac
                        self.liftTask = self.liftDown				
                        if self.lift_status.status == 3:  # Hoàn thành
                            # self.charger_requir = self.CHARGER_ON
                            self.liftTask = self.liftStop
                            self.flag_Auto_to_Byhand = 0

                    # elif self.after_mission == self.SERVERMISSION_CHARGER: # Sac
                    # 	self.charger_requir = self.CHARGER_ON
                    # 	self.flag_Auto_to_Byhand = 0

                    else: 
                        self.flag_Auto_to_Byhand = 0

                # -- add 18/11
                else:
                    self.flag_Auto_to_Byhand = 0

                self.process = 2
            else:
                self.process = 43
        
        elif self.process == 43: 	# Thực hiện nhiệm vụ trước.
            # self.completed_before_mission = 1
            if self.completed_before_mission == 0: # chua thuc hien
                self.job_doing = 2 
                if self.before_mission == 0:
                    # self.charger_requir = self.CHARGER_OFF
                    self.log_mess("info", "Before mission Not have", self.before_mission)
                    self.completed_before_mission = 1
                    # -- add new
                    self.completed_checkLift = 1
                    
                elif self.before_mission == self.SERVERMISSION_LIFTDOWN or self.before_mission == 2: # Hạ
                    # self.charger_requir = self.CHARGER_OFF
                    self.liftTask = self.liftDown

                    # -- 
                    self.save_liftStatus = self.liftTask
                    
                    if self.lift_status.status == 3:  # Hoàn thành
                        self.log_mess("info", "Before mission completed", self.SERVERMISSION_LIFTDOWN)
                        self.liftTask = self.liftStop
                        self.completed_before_mission = 1

                elif self.before_mission == self.SERVERMISSION_LIFTUP or self.before_mission == 1: # Nâng
                    # self.charger_requir = self.CHARGER_OFF
                    self.liftTask = self.liftUp		

                    # --
                    self.save_liftStatus = self.liftTask

                    if self.lift_status.status == 4:  # Hoàn thành
                        self.log_mess("info", "Before mission completed", self.SERVERMISSION_LIFTUP)
                        self.liftTask = self.liftStop
                        self.completed_before_mission = 1
                        self.completed_checkLift = 0
                    
                self.process = 2     # 44 Archie 
            else:
                self.process = 44

        elif self.process == 44:	# Thuc hien kiểm tra kệ có trên bàn nâng ko.
            if self.completed_checkLift == 0:
                self.job_doing = 3
                if self.before_mission == self.SERVERMISSION_LIFTUP or self.before_mission == 1 or (self.before_mission == self.SERVERMISSION_UNKNOWN and self.save_liftStatus == self.liftUp): # Nâng
                    if self.lift_status.sensor_lift == 1:  # có kệ thì thời gian tăng
                        self.lastTime_checkLift = time.time()
                    else:
                        pass

                    # t = (time.time() - self.lastTime_checkLift)%60  # -> Ko có kệ thì khoảng thời gian = 0, có kê thời gian tăng đàn 
                    if (time.time() - self.lastTime_checkLift) >= 0.8:   # thời gian ko có kệ
                        self.flag_checkLiftError = 1
                        self.process = 2
                        self.status_stop = 1
                        # self.completed_checkLift = 1
                    else:                                                     # thời gian có kệ, dù thực tế ko có nhưng vẫn cho agv chạy tiếp khoảng 0.8s
                        self.flag_checkLiftError = 0
                        self.process = 46
                        self.status_stop = 0
                        
                    if self.completed_moveSimple == 1 and self.completed_moveSpecial == 1:  # di chuyển đã xong, ko cần check có kệ hay ko nữa
                        self.completed_checkLift = 1
                        self.flag_checkLiftError = 0
                        self.process = 2
                        self.status_stop = 0

                else:
                    self.completed_checkLift = 1
                    self.process = 2
                    self.status_stop = 0

            else:
                self.process = 46

        elif self.process == 46:	# Thuc hien di chuyen diem thuong.
            # self.completed_moveSimple = 1
            if self.completed_moveSimple == 1:      # 
                self.process = 47
            else:
                self.job_doing = 5
                if (len(self.NN_cmdRequest.list_id) != 0) and (len(self.NN_cmdRequest.list_code) != 0):
                    self.enb_move = 5
                    self.move_req.target_id = self.target_id

                    # -- Hoang them dieu kien
                    if self.target_id == self.ID_WH and self.save_cartDir > 0:
                        self.move_req.target_dir = self.save_cartDir
                    else:
                        self.move_req.target_dir = self.target_dir
                    # self.move_req.target_dir = self.target_dir

                    self.move_req.moving_dir = self.NN_cmdRequest.moving_dir
                    self.move_req.list_id = self.NN_cmdRequest.list_id
                    self.move_req.list_code = self.NN_cmdRequest.list_code
                    self.move_req.list_dir = self.NN_cmdRequest.list_dir
                    self.move_req.list_speed = self.NN_cmdRequest.list_speed
                    self.move_req.list_encoder = self.NN_cmdRequest.list_encoder
                    self.move_req.mission = self.NN_cmdRequest.offset
                    
                else:
                    self.log_mess("warn", "ERROR: Target of List point wrong !!!", 0)

                # -- add 19/01/2022 : chuyen vung sick.
                if self.move_respond.mode_move == 5 and self.move_respond.completed == 1 and self.move_req.target_id == self.move_respond.target_id and self.move_req.target_dir == self.move_respond.target_dir:
                    self.completed_moveSimple = 1
                    self.log_mess("info", "Move simple completed", 0)
                    
                self.process = 2

        # -- Parking	
        
        elif self.process == 47:   #  Di chuyền lùi vào kệ
            # self.completed_moveSpecial = 1
            if self.completed_moveSpecial == 1:
                self.completed_move = 1
                self.process = 34
            else:
                self.job_doing = 6

                self.enb_move = 6
                self.move_req.target_id = self.target_id

                if self.target_id == self.ID_WH and self.save_cartDir > 0:
                    self.move_req.target_dir = self.save_cartDir
                else:
                    self.move_req.target_dir = self.target_dir
                # self.move_req.target_dir = self.target_dir
                
                self.move_req.moving_dir = self.NN_cmdRequest.moving_dir
                self.move_req.list_id = self.NN_cmdRequest.list_id
                self.move_req.list_code = self.NN_cmdRequest.list_code
                self.move_req.list_dir = self.NN_cmdRequest.list_dir
                self.move_req.list_speed = self.NN_cmdRequest.list_speed
                self.move_req.list_encoder = self.NN_cmdRequest.list_encoder
                self.move_req.mission = self.NN_cmdRequest.offset

                # -- add 19/01/2022 : chuyen vung sick.
                if self.move_respond.mode_move == 6 and self.move_respond.completed == 1:
                    self.completed_moveSpecial = 1
                    self.log_mess("info", "Move Special completed", 0)

                self.process = 2    

    # ------------------------------------------------------------------------------------
        elif self.process == 34:	# -- Thực hiện nhiệm vụ sau.
            # self.completed_after_mission = 1
            if self.completed_after_mission == 0: # chua thuc hien
                self.job_doing = 7
                if self.after_mission == 0:
                    self.log_mess("info", "Last mission Not have Suf: ", self.after_mission)
                    self.completed_after_mission = 1
                    
                elif self.after_mission == self.SERVERMISSION_LIFTDOWN: # Hạ
                    self.liftTask = self.liftDown

                    # --
                    self.save_liftStatus = self.liftTask

                    if self.lift_status.status == 3: # Hoàn thành
                        self.log_mess("info", "Last mission completed: ", self.SERVERMISSION_LIFTDOWN)
                        self.liftTask = self.liftStop
                        self.completed_after_mission = 1

                elif self.after_mission == self.SERVERMISSION_LIFTUP: # Nâng
                    self.liftTask = self.liftUp			

                    # -- 
                    self.save_liftStatus = self.liftTask

                    if self.lift_status.status == 4: # Hoàn thành
                        self.log_mess("info", "Last mission completed: ", self.SERVERMISSION_LIFTUP)
                        self.liftTask = self.liftStop
                        self.completed_after_mission = 1

                # -- 
                elif self.after_mission == self.SERVERMISSION_LIFTUP_WAIT:  # Nâng chờ xác nhận
                    if self.sub_processWait == 0:
                        self.liftTask = self.liftUp

                        # -- 
                        self.save_liftStatus = self.liftTask

                        if self.lift_status.status == 4:
                            self.liftTask = self.liftStop
                            self.sub_processWait = 1

                    elif self.sub_processWait == 1:
                        # -- Hoang them
                        self.callDevice_query.data = 1     # - gửi tín hiệu tới bộ gọi
                        # --
                        self.log_mess("info", "Đang chờ tin hiệu từ nút nhấn: ", 0)
                        self.job_doing = 10
                        if self.app_button.bt_confirm == 1 or self.data_callDevice == 1:
                            self.job_doing = 11
                            self.completed_after_mission = 1
                            self.callDevice_query.data = 0
                            self.sub_processWait = 0
                            self.log_mess("info", "Last mission completed: ", self.SERVERMISSION_LIFTUP_WAIT)
                        
                elif self.after_mission == self.SERVERMISSION_CHARGER: # sac
                    # self.charger_requir = self.CHARGER_ON
                    self.log_mess("info", "Last mission completed: ", self.SERVERMISSION_CHARGER)
                    self.completed_after_mission = 1

                elif self.after_mission == self.SERVERMISSION_LIFTDOWN_CHARGER: # sac
                    # if self.sub_processWait == 0:
                    #     self.liftTask = self.liftDown
                    #     # -- 
                    #     self.save_liftStatus = self.liftTask

                    #     if self.lift_status.status == 3: # Hoàn thành
                    #         self.liftTask = self.liftStop
                    #         self.sub_processWait = 1
                    #         # --

                    # elif self.sub_processWait == 1:

                    #     self.log_mess("info", "Đang chờ tin hiệu từ nút nhấn: ", 0)
                    #     self.job_doing = 10
                    #     if self.app_button.bt_confirm == 1:
                    #         self.job_doing = 11
                    #         self.completed_after_mission = 1
                    #         self.sub_processWait = 0
                    #         self.log_mess("info", "Last mission completed: ", self.SERVERMISSION_LIFTDOWN_CHARGER)

                    self.liftTask = self.liftDown

                    # --
                    self.save_liftStatus = self.liftTask

                    if self.lift_status.status == 3: # Hoàn thành
                        self.log_mess("info", "Last mission completed: ", self.SERVERMISSION_LIFTDOWN)
                        self.liftTask = self.liftStop
                        self.completed_after_mission = 1

                        # self.charger_requir = self.CHARGER_ON # turn on charger				

                self.process = 2
            else:
                self.process = 35

        elif self.process == 35:
            self.job_doing = 8
            self.process = 2
            self.log_mess("warn", "Wating new Target ...", 0)
            
        # -- Tag + Offset:
        if self.mode_operate == self.mode_auto:
            if self.completed_move == 1:
                self.NN_infoRespond.tag = self.NN_cmdRequest.tag
                self.NN_infoRespond.offset = self.NN_cmdRequest.offset
            else:
                self.NN_infoRespond.tag = 0
                self.NN_infoRespond.offset = 0

            if self.completed_before_mission == 1 and self.completed_after_mission == 0 and self.flag_checkLiftError == 0:
                self.NN_infoRespond.task_status = self.before_mission
            elif self.completed_before_mission == 1 and self.completed_after_mission == 0 and self.flag_checkLiftError == 1:
                self.NN_infoRespond.task_status = self.STATUSTASK_LIFTERROR
            if self.completed_before_mission == 1 and self.completed_after_mission == 1:
                self.NN_infoRespond.task_status = self.after_mission

        self.NN_infoRespond.rfid_last_code = self.move_respond.rfid_last_code
        self.NN_infoRespond.rfid_code = self.move_respond.rfid_code
        self.NN_infoRespond.direction = self.move_respond.dir_now
        self.NN_infoRespond.status = self.statusU300L    # Status: Error
        self.NN_infoRespond.error_perform = self.process
        self.NN_infoRespond.error_moving = self.flag_error
        self.NN_infoRespond.error_device = self.numberError
        self.NN_infoRespond.list_error = self.listError
        self.NN_infoRespond.process = self.job_doing

        # -- Battery - ok
        self.readbatteryVoltage()
        self.NN_infoRespond.battery = int(self.valueVoltage)
        
        if self.flag_cancelMission == 0:
            # -- mode respond server
            if self.mode_operate == self.mode_by_hand:         # Che do by Hand
                self.NN_infoRespond.mode = 1

            elif self.mode_operate == self.mode_auto:          # Che do Auto
                self.NN_infoRespond.mode = 2
        else:
            self.NN_infoRespond.mode = 5

        # -- Respond Client
        self.pub_infoRespond.publish(self.NN_infoRespond)    # Pub Client

        # -- Request Navigation
        # self.log_mess("info", "Giá trị gửi đi moving là ...", self.move_req.moving_dir)
        self.pub_move_req(self.enb_move, self.move_req, self.status_stop)  # Pub Navigation

        # -- Speaker && led
        if self.app_button.bt_setting == 0:
            # speaker
            if self.flag_error == 1 or self.flag_warning == 1:
                self.speaker_requir = self.SPK_ERROR

            else:
                # print("job_doing: ", self.job_doing)
                if self.job_doing == 10:
                    self.speaker_requir = self.SPK_CONFIRM
                elif self.job_doing == 20:
                    self.speaker_requir = self.SPK_MANUAL
                else:
                    self.speaker_requir = self.SPK_MOVE

            if self.flag_error == 1:
                self.led_effect = self.LED_ERROR
            else:
                if self.completed_before_mission == 0 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
                    self.led_effect = self.LED_PERFORM

                elif self.completed_before_mission == 1 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
                    if self.find_element(411, self.listError) == 1:
                        self.led_effect = self.LED_STOPBARRIER
                    
                    elif self.find_element(471, self.listError) == 1:
                        self.led_effect = self.LED_PERFORM	

                    else:
                        self.led_effect = self.LED_SIMPLERUN

                elif self.completed_before_mission == 1 and self.completed_moveSimple == 1 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:	
                    if self.find_element(411, self.listError) == 1:
                        self.led_effect = self.LED_STOPBARRIER
                    else:
                        self.led_effect = self.LED_SPECIALRUN     # 
                    
                elif self.completed_before_mission == 1 and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 0:	
                    self.led_effect = self.LED_PERFORM

                elif self.completed_before_mission == 1 and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 1:
                    self.led_effect = self.LED_COMPLETED

                else:
                    self.led_effect = self.LED_SIMPLERUN
                
        # -- -- -- pub Board
        time_curr = time.time()
        d = (time_curr - self.pre_timeBoard)
        if (d > float(1/self.FrequencePubBoard)): # < 20hz 
            self.pre_timeBoard = time_curr
            # -- Request OC:
            self.lift_control.control = self.liftTask
            self.lift_control.reset = self.liftReset
            self.pub_OC.publish(self.lift_control)

            # -- Request Main: charger, sound, EMC_write, EMC_reset
            if self.enb_spk == 1:
                # tat Loa khi sac thanh cong!
                if self.charger_write == self.CHARGER_ON and self.main_info.charge_current >= self.charger_valueOrigin and self.flag_error == 0:
                    self.speaker = self.SPK_OFF
                else:
                    self.speaker = self.speaker_requir
            else:
                self.speaker = self.SPK_OFF

            self.Main_pub(self.charger_write, self.speaker, self.EMC_write, self.EMC_reset)  # MISSION

            # -- Request HC:
            self.HC_request.rgb1 = self.led_effect
            self.HC_request.rgb2 = self.led_effect
            self.pub_HC.publish(self.HC_request)

            # self.led_query.data = self.convertLed(self.led)
            # self.pub_LED.publish(self.led_query)

            # -- Request task Driver:
            self.pub_taskDriver.publish(self.task_driver)

            # ---------------- Brake ---------------- #
            self.pub_disableBrake.publish(self.disable_brake)
            
        # -- cancel mission
        # if self.cancelMission_control.data == 1:
        # 	self.flag_cancelMission = 1
        # 	self.cancelMission_status.data = 1

        # if self.NN_cmdRequest.id_command == 0:
        # 	self.flag_cancelMission = 0
        # 	self.cancelMission_status.data = 0
        # 	# -- 
        # 	if self.cancelMission_control.data == 1:
        # 		self.cancelMission_status.data = 1

        # write file backup
        if time.time() - self.time_writeBackup > 3.:
            self.writeBackup()
            self.time_writeBackup = time.time()

        self.pub_cancelMission.publish(self.cancelMission_status)

        # -- 
        self.pub_controlCallDevice.publish(self.callDevice_query)

        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = ros_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

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
