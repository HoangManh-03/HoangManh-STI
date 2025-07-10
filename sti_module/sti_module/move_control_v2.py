#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import json
import sys
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from decimal import *
from math import degrees, radians

from message_pkg.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

from lib_pidController import *

class Rfid2Head():
    def __init__(self, move_byHead, move_byTail):
        self.reset_rfid_code()

        self.move_byHead = move_byHead
        self.move_byTail = move_byTail

        self.saveStatusMove = -1

    def reset_rfid_code(self):
        self.pre_code_front = 0
        self.now_code_front = 0
        # --
        self.pre_code_behind = 0
        self.now_code_behind = 0

    def getRfidFront(self, data_rfid_front):
        data = 0
        if data_rfid_front.status == 1:
            data = data_rfid_front.id
        return data

    def getRfidBehind(self, data_rfid_behind):
        data = 0
        if data_rfid_behind.status == 1:
            data = data_rfid_behind.id
        return data

    def updateRfidFront(self, data_rfid_front):
        d_front = self.getRfidFront(data_rfid_front)
        if d_front > 0:
            if self.now_code_front != d_front:
                self.pre_code_front = self.now_code_front
                self.now_code_front = d_front

    def updateRfidBehind(self, data_rfid_behind):
        d_behind = self.getRfidBehind(data_rfid_behind)
        if d_behind > 0:
            if self.now_code_behind != d_behind:
                self.pre_code_behind = self.now_code_behind
                self.now_code_behind = d_behind

    def getInfoRFIDByDirectMoving(self, dir_move_agv):
        _dir = dir_move_agv
        if self.saveStatusMove == -1 and dir_move_agv != 0:
            self.saveStatusMove = _dir

        if _dir == self.move_byTail:
            if self.saveStatusMove == self.move_byHead:
                if self.now_code_front != self.now_code_behind:
                    self.pre_code_behind = self.now_code_front

                self.saveStatusMove = _dir
            
            return self.now_code_behind, self.pre_code_behind
        
        elif _dir == self.move_byHead:
            if self.saveStatusMove == self.move_byTail:
                if self.now_code_behind != self.now_code_front:
                    self.pre_code_front = self.now_code_behind
                self.saveStatusMove = _dir

            return self.now_code_front, self.pre_code_front
        
        else:
            if self.saveStatusMove == self.move_byTail:
                return self.now_code_behind, self.pre_code_behind
            elif self.saveStatusMove == self.move_byHead:
                return self.now_code_front, self.pre_code_front

        
        # else:
        #     return self.now_code_front, self.pre_code_front


class StatusAGV():
    def __init__(self):
        self.reset()

    def reset(self):
        self.dir_now = -1
        self.dir_follow = 0
        self.RFID_lastCode = 0
        self.RFID_code = 0
        self.status_move = 0
        self.war_agv = 0
        self.err_agv = 0

class MoveControl(LifecycleNode):
    def __init__(self):
        super().__init__('move_control')
        self.get_logger().warning("ROS 2 Node Move Control Initialized!")
        self.killnode = 0

        # -- param const
        self.move_stop = 0
        self.moveByHead = 1
        self.moveByTail = 2
        self.moveRotation = 3

        # -- param turn
        self.turn90Left = 3
        self.turn90Right = 1
        self.turn180 = 2

        self.declare_parameters(
            namespace='',
            parameters=[
                ('vel_linear_max',              0.4),
                ('vel_linear_min',              0.1),
                ('vel_angular_max',             0.7),
                ('vel_angular_min',             0.12),
                ('vel_rotation',                0.1),
                ('vel_moveByHandDefaul',        0.3),
                ('vel_findDirFirst',            0.2),
                ('vel_moveStop',                0.1)
            ]
        )
        
        self.vel_linear_max             = self.get_parameter('vel_linear_max').value
        self.vel_linear_min             = self.get_parameter('vel_linear_min').value

        self.vel_angular_max            = self.get_parameter('vel_angular_max').value
        self.vel_angular_min            = self.get_parameter('vel_angular_min').value

        self.vel_rotation               = self.get_parameter('vel_rotation').value
        self.vel_moveByHandDefaul       = self.get_parameter('vel_moveByHandDefaul').value

        self.vel_findDirFirst           = self.get_parameter('vel_findDirFirst').value
        self.vel_moveStop               = self.get_parameter('vel_moveStop').value

        self.vel_moveHorizontalLine     = self.get_parameter('vel_moveStop').value

        self.allRFID = Rfid2Head(self.moveByHead, self.moveByTail)

        # -- Publisher 
        # -- move respond
        self.pub_respond = self.create_publisher(MoveRespond, '/respond_move', 10)
        self.pub_move = MoveRespond()
        self.rate_pubRespondMove = 30
        self.saveTime_pubRespondMove = time.time()

        # -- line controller
        self.pub_lineController = self.create_publisher(LineControllerRequest, 'line_controller_request', 10)
        self.mess_pub_lineController = LineControllerRequest()

        # -- field request
        self.pub_requestFields = self.create_publisher(Int8, 'HC_fieldRequest', 10)
        self.oldselectfield = 0

        # -- cmd_vel
        self.pub_cmdVel = self.create_publisher(Twist, 'cmd_vel', qos_profile = QoSProfile(
                depth = 10,
                reliability = QoSReliabilityPolicy.RELIABLE,    # BEST_EFFORT: sensor | RELIABLE: control
                durability  = QoSDurabilityPolicy.VOLATILE     # TRANSIENT_LOCAL: latch | VOLATILE: no latch
            ))
        self.rate_cmdvel = 30. 
        self.saveTime_pubVel = time.time()

        # -- Subcriber
        # -- move request
        self.sub_moveRequest = self.create_subscription(
            MoveRequest,
            "/request_move",
            self.callback_move,
            10)
        self.sub_moveRequest

        self.req_move = MoveRequest()
        self.is_request_move = False

        # -- hc
        self.sub_hcInfo = self.create_subscription(
            HcInfo,
            "hc_info",
            self.callback_zone,
            10)
        self.sub_hcInfo

        self.zone_lidar = HcInfo()
        self.is_check_zone = False

        # -- front magline
        self.sub_maglineFront = self.create_subscription(
            MagneticLine,
            "magneticLine_front",
            self.callBack_maglineFront,
            10)
        self.sub_maglineFront

        self.is_magline_front = False
        self.data_mangline_front = MagneticLine()

        # -- behind magline
        self.sub_maglineBehind = self.create_subscription(
            MagneticLine,
            "magneticLine_behind",
            self.callBack_maglineBehind,
            10)
        self.sub_maglineBehind

        self.is_magline_behind = False
        self.data_mangline_behind = MagneticLine()

        # -- front rfid
        self.sub_rfidFront = self.create_subscription(
            RFID,
            "rfid_front_respond",
            self.callback_rfidFront,
            10)
        self.sub_rfidFront

        self.is_rfid_front = False
        self.data_RFID_front = RFID()

        # -- behind rfid
        self.sub_rfidBehind = self.create_subscription(
            RFID,
            "rfid_behind_respond",
            self.callback_rfidBehind,
            10)
        self.sub_rfidBehind

        self.is_rfid_behind = False
        self.data_RFID_behind = RFID()

        # -- MC
        self.sub_mc = self.create_subscription(
            McInfo,
            "mc_info",
            self.callback_MC,
            10)
        self.sub_mc

        self.is_mc = False
        self.data_mc = McInfo()

        # -- Dir by Hand
        self.sub_dir = self.create_subscription(
            AppButtonAgvmag,
            "app_button",
            self.callback_updateDirAGV,
            10)
        self.sub_dir

        self.data_dirUpdate = -1

        # -- 
        self.infoAGV = StatusAGV()
        # --
        # self.followLineController = LineFollowerPID(self.pub_cmdVel)
        self.followLineController = LineFollowerFUZZYPID(self.pub_cmdVel)
        
        # -- param
        self.process = 0

        self.target_id = 0
        self.target_dir = -1
        self.moving_dir = -1
        self.list_id = [0, 0, 0, 0, 0]
        self.list_code = [0, 0, 0, 0, 0]
        self.list_dir = [0, 0, 0, 0, 0]
        self.list_speed = [0, 0, 0, 0, 0]

        self.mode_move = 0
        self.dir_move = 0

        # -- tiến hay lùi gặp vạch ngang
        self.forward_horizontal_line = 2
        self.backward_horizontal_line = -2
        self.status_go_horizontal_line = self.backward_horizontal_line

        self.old_id_follow = 0

        self.completed_backward = 0     # bao da den aruco.
        self.completed_all = 0
        self.completed_reset = 0
        self.waiting_newVelId = 0

        self.end_of_list = False
        self.flag_checkDir = 1
        self.flag_runningFindDirect = 0
        self.flag_findDirectMove = 0
        self.is_idTarget = 0

        # -- 
        self.is_idVelZero = 0

        # --
        # self.mode_moveReset = 0 
        # self.mode_moveAheadByHand = 1
        # self.mode_moveBehindByHand = 2
        # self.mode_moveAuto = 3
        # self.mode_moveStopHorizontaLine = 4

        # --
        self.mode_moveStopByHand = 0 
        self.mode_moveAheadByHand = 1
        self.mode_moveBehindByHand = 2
        self.mode_moveRotationLeft = 3
        self.mode_moveRotationRight = 4
        self.mode_moveAuto = 5
        self.mode_moveStopHorizontaLine = 6

        

        # -- function control vel
        self.target_velNow = 0.

        # -- 
        self.subProcessTurn = 0
        self.saveTimeWhenStop = time.time()
        self.num_on1bit = 0
        self.high_1bit = 0

        # -- 
        self.oldCodeRFID = 0
        self.saveTimeFreeRfid = time.time()

        # -- 
        self.job_doing = 0

        # -- 
        self.flag_misson0 = 0

        # -- update mode By hand
        self.subTurnByHand = 0
        self.completed_byHand = 0

        # -- 
        self.mode_Auto = 1
        self.mode_ByHand = 2
        self.mode_operation = self.mode_ByHand

        # -- 
        self.just_move = 0

        # -- 
        self.codeNoInList = 0
        self.save_oldDirAGV = 0

        # -- turn follow by
        self.needStopByAheadortail = 0 # 0: follow by head, 2 follow by tail
        self.is_pointRotationSpecial = 0

        self.just_stop = 0

        # -- check loi quay khi traffic update diem khi chay
        self.just_updateInfoCode = 0

        self.ID_vel0_notTarget = 0
        self.save_vel0_notTarget = 0.

        # -- 
        self.time_writeBackupAGV = time.time()

        # -- 
        self.id_agvNow = -1
        self.id_agvNextFollow = -1

        # -- param
        self.vel_movePointStop = 0.1
        self.vel_inZone2 = 0.2

        self.id_pointNow = 0

        # -- bo sung 15/8/2024
        self.before_cross = 1
        self.after_cross = 2
        self.status_cross = 0

        # -- Create timer run 
        self.rate_main = 100
        self.timer_period_main = 1/self.rate_main
        self.timer_main = self.create_timer(self.timer_period_main, self.run)

        # -- Create timer follow line 
        self.rate_follow = 15
        self.timer_period_follow = 1/self.rate_follow
        self.timer_follow = self.create_timer(self.timer_period_follow, self.run_followLine)


    # -- Callback Function
    def callback_move(self, data):
        self.req_move = data
        self.is_request_move = True

    def callback_zone(self, data):
        self.zone_lidar = data
        self.is_check_zone = True	

    def callback_rawVel(self, data):
        self.vel_raw = data
        self.is_raw_vel = True

    def callback_odom(self, data):
        self.odom_rb = data
        self.is_odom_rb = True

    def callBack_maglineFront(self, data):
        self.data_mangline_front = data
        self.is_magline_front = True

    def callBack_maglineBehind(self, data):
        self.data_mangline_behind = data
        self.is_magline_behind = True

    def callback_rfidFront(self, data):
        self.data_RFID_front = data
        self.allRFID.updateRfidFront(data)
        self.is_rfid_front = True

    def callback_rfidBehind(self, data):
        self.data_RFID_behind = data
        self.allRFID.updateRfidBehind(data)
        self.is_rfid_behind = True

    def callback_MC(self, data):
        self.data_mc = data
        self.is_mc = True

    def callback_updateDirAGV(self, data):
        self.data_dirUpdate = data.bt_dir

    # -- 
    def shutdown_hook(self):
        for i in range(2):
            self.pub_cmdVel.publish(Twist())

        self.get_logger().warning("Shutting down. cmd_vel will be 0")

    def on_shutdown(self, state):
        self.killnode = 1
        # self.shutdown_hook()
        self.get_logger().warning("Shutting down node by lifecycle! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    def stop(self):
        # reset van toc
        self.followLineController.enable = 1
        # -- 
        self.infoAGV.status_move = self.move_stop

        for i in range(3):
            self.pub_cmdVel.publish(Twist())

    def Pub_cmdVel(self, twist , rate):
        if time.time() - self.saveTime_pubVel > float(1/rate) : # < 20hz 
            self.saveTime_pubVel = time.time()
            self.pub_cmdVel.publish(twist)

    def pub_respondMove(self, _mode_move, _target_id, _target_dir, _status, _completed, _error, _process, _rfid_lc, _rfid_c, _dir_now):
        if time.time() - self.saveTime_pubRespondMove > float(1/self.rate_pubRespondMove) : # < 20hz 
            self.saveTime_pubRespondMove = time.time()

            msg_move = MoveRespond()
            msg_move.mode_move = _mode_move
            msg_move.target_id = _target_id
            msg_move.target_dir = _target_dir
            msg_move.status = _status
            msg_move.completed = _completed
            msg_move.error = _error
            msg_move.process = _process
            msg_move.rfid_last_code = _rfid_lc
            msg_move.rfid_code = _rfid_c
            msg_move.dir_now = _dir_now
            msg_move.message = 'agv moving'

            self.pub_respond.publish(msg_move)

    # -- 
    def check_codeInList(self, code, list):
        for index, _code in enumerate(list):
            if code == _code:
                return index
            
        return -1

    def check_indexLegend(self, list):
        index = 0
        for i in list:
            if i != 0:
                index += 1
            else:
                break

        return index


    def followLine(self, _dir, _safety_skip = False):
        safety = 1
        if _safety_skip:
            safety = 0
            
        # -- vel
        velocity = self.target_velNow

        # -- reset status agv
        self.infoAGV.err_agv = self.followLineController.error
        self.infoAGV.war_agv = self.followLineController.warn

        # Gửi yêu cầu bám line từ
        self.followLineController.direction_move = _dir
        self.followLineController.velocity = velocity
        self.followLineController.safety = safety
        self.followLineController.enable = 2

        # -- Hoang update trc hay sau nga tu
        if self.status_cross == 0 and self.mode_operation == self.mode_Auto:
            if _dir == self.moveByHead:
                self.status_cross = self.before_cross
            elif _dir == self.moveByTail:
                self.status_cross = self.after_cross

        # update param
        if self.followLineController.curr_velocity != 0:
            self.infoAGV.status_move = _dir
        else:
            self.infoAGV.status_move = 0

        rfid_code = self.infoAGV.RFID_code
        rfid_preCode = self.infoAGV.RFID_lastCode

        if rfid_preCode != 0:
            index_nowCode = self.check_codeInList(rfid_code, self.req_move.list_code) 
            if index_nowCode == 0:
                if self.infoAGV.status_move == self.moveByHead:
                    if self.infoAGV.dir_now == -1 or self.mode_move == 1 or self.mode_move == 2: # neu chua co huong
                        self.infoAGV.dir_now = self.req_move.moving_dir

                elif self.infoAGV.status_move == self.moveByTail:
                    dir_now = (2 + self.req_move.moving_dir)%4
                    if dir_now == 0:
                        dir_now = 4

                    if self.infoAGV.dir_now == -1 or self.mode_move == 1 or self.mode_move == 2: # neu chua co huong
                        self.infoAGV.dir_now = dir_now

    def doNextDirection(self, angle_turn = 0, dir_magline = 0):
        dir_turn = 1
        num_encounter1bit = 1
        time_offset = 0.
        time_turn = 0.

        # -- reset emg
        self.infoAGV.war_agv = 0

        if angle_turn == self.turn90Left:
            dir_turn = 1
            num_encounter1bit = 1
            time_turn = 4.

        elif angle_turn == self.turn90Right:
            dir_turn = -1
            num_encounter1bit = 1
            time_turn = 4.

        elif angle_turn == self.turn180:
            dir_turn = 1
            num_encounter1bit = 2
            time_turn = 4.

        else:
            dir_turn = 0
            num_encounter1bit = 0
            time_turn = 0.

        if self.subProcessTurn == 0:
            if self.flag_misson0 == 1 or self.just_stop == 1:
                self.subProcessTurn = 2 #
                print("trang thai truoc la mission 0, quy trinh tiep theo la ", angle_turn)

                self.saveTimeWhenStop = time.time()

            else:
                # self.target_velNow = self.vel_moveStop
                self.saveTimeWhenStop = time.time()
                self.subProcessTurn = 1
                print("bat dau di chuyen gap vach ngang")

        elif self.subProcessTurn == 1:
            time_move = time.time()- self.saveTimeWhenStop

            if self.data_mc.sensor_1bit == 1 and time_move >= time_offset:
                self.high_1bit = 1
                self.num_on1bit = 0
                self.stop()
                self.subProcessTurn = 2 # da gap vach ngang
                print("da gap vach ngang, quy trinh tiep theo la ", angle_turn)
                self.saveTimeWhenStop = time.time()

            else:
                self.followLine(self.dir_move)

        elif self.subProcessTurn == 2:
            if dir_turn == 0:
                print("huong di chuyen trung target")
                return 1
            
            else:
                print(self.data_mc.sensor_1bit, self.high_1bit, self.num_on1bit)
                if self.data_mc.sensor_1bit == 1 and self.high_1bit == 0:
                    self.num_on1bit += 1
                    self.high_1bit = 1

                elif self.data_mc.sensor_1bit == 0:
                    self.high_1bit = 0

                time_move = time.time()- self.saveTimeWhenStop

                # -- detect line tu
                if dir_magline:
                    value_line = self.data_mangline_behind.value
                    print("follow magline behind, value: ", value_line)
                else:
                    value_line = self.data_mangline_front.value
                    print("follow magline front, value: ", value_line)

                sensor_center = 1 if value_line >= 5 and value_line <= 10 else 0
                # if self.data_mangline_behind.value == 20 and time_move > time_turn:
                if self.data_mc.sensor_1bit == 1 and self.num_on1bit >= num_encounter1bit and time_move > time_turn:
                    self.stop()
                    self.subProcessTurn = 0
                    self.high_1bit = 0
                    self.num_on1bit = 0
                    print("sensor center = ", sensor_center)
                    print("value front magline %s, value behind magline %s " %(self.data_mangline_front.value, self.data_mangline_behind.value))
                    return 1

                else:
                    self.infoAGV.status_move = self.moveRotation
                    twist = Twist()
                    twist.angular.z = self.vel_rotation*dir_turn
                    self.Pub_cmdVel(twist, self.rate_cmdvel)

        return 0
            

    def updateAll(self):
        if len(self.req_move.list_id) == 5 and len(self.req_move.list_code) == 5 and len(self.req_move.list_dir) == 5 and len(self.req_move.list_speed) == 5:
            self.target_id = self.req_move.target_id
            self.target_dir = self.req_move.target_dir
            # self.moving_dir = self.req_move.moving_dir
            self.list_id = self.req_move.list_id
            self.list_code = self.req_move.list_code
            self.list_dir = self.req_move.list_dir
            self.list_speed = self.req_move.list_speed

            # -- reset
            self.completed_reset = 0
            self.completed_backward = 0
            self.completed_all = 0
            # -- velocity
            # -- follow line
            # self.followMagline.reset()
            # self.followMagline2.reset()
            # -- 
            self.is_idTarget = 0
            self.num_on1bit = 0
            self.high_1bit = 0
            self.subProcessTurn = 0

            # -- Yêu cầu kiểm tra lại hướng làm việc
            self.flag_findDirectMove = 1
            # -- 
            self.infoAGV.status_move = self.move_stop

            self.oldCodeRFID = 0
            # -- 
            self.codeNoInList = 0
            # -- 
            self.ID_vel0_notTarget = 0
            # -- 
            self.just_move = 0

            # -- reset huong lam viec
            self.infoAGV.dir_follow = 0

            # -- Thêm điều kiện thay đổi target thì đi với vận tốc bé
            self.target_velNow = 0.1

            # -- 
            self.needStopByAheadortail = 0

            return 1
        
        return 0

    def resetAutoMan(self):
        self.completed_reset = 0
        self.completed_backward = 0
        self.completed_all = 0
        self.completed_byHand = 0
        # -- 
        self.infoAGV.war_agv = 0
        self.infoAGV.err_agv = 0
        # -- follow line
        # self.followMagline.reset()
        # self.followMagline2.reset()
        self.mess_pub_lineController.enable = 0
        # -- 
        # -- 
        # self.flag_checkDir = 0
        # self.flag_findDirectMove = 1
        self.flag_runningFindDirect = 0
        self.is_idTarget = 0
        self.is_idVelZero = 0
        self.waiting_newVelId = 0

        self.subProcessTurn = 0

        # --
        self.target_id = 0
        self.target_dir = -1

        # -- 
        self.infoAGV.status_move = self.move_stop

        self.oldCodeRFID = 0
        # -- 
        self.job_doing = 0 # reset

        # -- 
        self.flag_misson0 = 0 # set flag missin0  = 0
        # --
        self.codeNoInList = 0

        # --
        # self.flag_inEvevator = 0

        # -- 
        self.ID_vel0_notTarget = 0
        # -- 
        self.just_move = 0

        # -- 
        self.id_agvNow = -1
        self.id_agvNextFollow = -1

        # -- 
        self.needStopByAheadortail = 0

        self.num_on1bit = 0
        self.high_1bit = 0

    def resetAll(self):
        self.completed_reset = 0
        self.infoAGV.reset()
        self.allRFID.reset_rfid_code()
        self.completed_backward = 0
        self.completed_all = 0
        self.completed_byHand = 0
        # -- follow line
        # self.followMagline.reset()
        # self.followMagline2.reset()
        self.mess_pub_lineController.enable = 0
        # -- 
        self.flag_checkDir = 1
        self.flag_findDirectMove = 1
        self.flag_runningFindDirect = 1
        self.is_idTarget = 0
        self.is_idVelZero = 0
        self.waiting_newVelId = 0

        self.num_on1bit = 0
        self.high_1bit = 0
        self.subProcessTurn = 0

        # --
        self.target_id = 0
        self.target_dir = -1

        # -- 
        self.infoAGV.status_move = self.move_stop

        self.oldCodeRFID = 0
        # -- 
        self.job_doing = 0 # reset
        # -- 
        self.flag_misson0 = 0 # set flag missin0  = 0
        # --
        self.codeNoInList = 0

        self.just_move = 0
        # -- 
        self.id_agvNow = -1
        self.id_agvNextFollow = -1

        self.status_cross = 0
        # --
        self.needStopByAheadortail = 0

    def resetManualAuto(self):
        # -- dung lai
        self.stop()
        self.infoAGV.reset()
        # self.allRFID.reset_rfid_code()
        print("start reset manual!")

    def resetChangeMode(self):
        self.completed_byHand = 0.
        # -- 
        self.infoAGV.war_agv = 0
        self.infoAGV.err_agv = 0
        # -- 
        self.codeNoInList = 0

        # -- 
        self.needStopByAheadortail = 0

    def run_followLine(self):
        if self.process > 0:
            self.followLineController.run(self.data_mangline_front, self.data_mangline_behind, self.zone_lidar)
        
    def run(self):
        if self.process == 0: # check all data
            print("wait data")
            ck = 0
            if self.is_magline_front:
                ck += 1
            if self.is_magline_behind:
                ck += 1
            if self.is_rfid_front:
                ck += 1
            if self.is_rfid_behind:
                ck += 1
            if self.is_check_zone:
                ck += 1
            if self.is_request_move:
                ck += 1
            if ck == 6:
                print("recieve all data need (^.^)!")
                self.process = 1

        elif self.process == 1:
            if self.mode_move != self.req_move.enable:
                self.stop()
                self.resetChangeMode()
                print("Thay doi mode: ", self.req_move.enable)

                if self.req_move.enable == self.mode_moveAuto or self.req_move.enable == self.mode_moveStopHorizontaLine:
                    mo = self.mode_Auto
                else:
                    mo = self.mode_ByHand

                if mo == self.mode_ByHand and self.mode_operation == self.mode_Auto:
                    self.resetAutoMan()

                # -- reset follow line by hand
                if mo == self.mode_Auto and self.mode_operation == self.mode_ByHand:
                    if self.infoAGV.dir_now <= 0:
                        print("mode by hand sang auto update flag")
                        self.flag_checkDir = 1
                        self.flag_runningFindDirect = 1

                    else:
                        self.flag_findDirectMove = 1

                # if self.mode_move == self.mode_moveStopHorizontaLine and self.req_move.enable == self.mode_moveAuto:
                #     # neu dang vao sac ma doi lenh thi AGV dung lai
                #     self.stop()
                #     if self.dir_move == self.moveByTail:
                #         self.dir_move = self.moveByHead

                #     else:
                #         self.dir_move = self.moveByTail

                    # print("Doi lenh khi dang lui vach ngang")

                self.mode_move = self.req_move.enable
                self.mode_operation = mo

            # -- update huong
            if self.mode_operation == self.mode_ByHand:
                if self.data_dirUpdate > 0:
                    self.infoAGV.dir_now = self.data_dirUpdate

            if self.mode_move == self.mode_moveStopByHand: # reset all
                # self.process = -3
                pass

            elif self.mode_move == self.mode_moveAheadByHand: # chay bang tay tien
                self.process = 13

            elif self.mode_move == self.mode_moveBehindByHand: # chay bang tay lui
                self.process = 23

            elif self.mode_move == self.mode_moveAuto: # tu dong
                self.process = 33

            elif self.mode_move == self.mode_moveStopHorizontaLine: # tien/lui gap vach ngang
                self.process = 43

        elif self.process == 13: # chay bang tay tien
            self.target_velNow = (self.req_move.speed_move_by_hand/100.)*self.vel_linear_max
            if self.target_velNow <= self.vel_moveByHandDefaul:
                self.target_velNow = self.vel_moveByHandDefaul
            
            self.target_velNow = 0.6
            self.dir_move = self.moveByHead
            self.followLine(self.dir_move)
            self.process = 1

        elif self.process == 23: # chay bang tay lui
            self.target_velNow = (self.req_move.speed_move_by_hand/100.)*self.vel_linear_max
            if self.target_velNow <= self.vel_moveByHandDefaul:
                self.target_velNow = self.vel_moveByHandDefaul

            self.target_velNow = 0.6
            self.dir_move = self.moveByTail
            self.followLine(self.dir_move)
            self.process = 1

        elif self.process == 33: # kiem tra target co thay doi khong/ update target
            if self.req_move.target_id > 0 and self.req_move.target_dir > 0:
                if self.target_id != self.req_move.target_id or self.target_dir != self.req_move.target_dir:
                    self.stop()
                    ck = self.updateAll()
                    if ck == 1:
                        self.get_logger().warning("Change target: {} | request: {}".format(self.target_id, self.req_move.target_id))
                        self.process = 34
                        self.job_doing = 1 # có target mới

                    else:
                        self.get_logger().warning("Something Wrong!, wait the update Target")
                        self.process = 1

                else:
                    self.process = 34

            else:
                # AGV dang di chuyen, thuc hien di chuyen toi dich
                if self.infoAGV.status_move != 0:
                    self.job_doing = -1 # lỗi target khi agv đang di chuyen
                    print("target_id = 0 when AGV is MOVING, process update target")
                    self.process = 34

                else:
                    print("target_id = 0 when AGV is STOP, process update target")
                    self.job_doing = -2 # lỗi target khi agv đang dung
                    self.process = 1

                self.stop()
                self.process = 1

        elif self.process == 34: # kiem tra AGV da den vi tri cuoi chua
            if self.completed_all != 1:
                self.process = 35

            else:
                self.job_doing = 9
                print("Doi target moi")
                self.process = 1

        elif self.process == 35: # update list
            if len(self.req_move.list_id) == 5 and len(self.req_move.list_code) == 5 and len(self.req_move.list_dir) == 5 and len(self.req_move.list_speed) == 5:
                if self.list_id != self.req_move.list_id:
                    self.list_id = self.req_move.list_id
                
                if self.list_code != self.req_move.list_code:
                    self.list_code = self.req_move.list_code

                if self.list_dir != self.req_move.list_dir:
                    self.list_dir = self.req_move.list_dir

                if self.list_speed != self.req_move.list_speed:
                    self.list_speed = self.req_move.list_speed

                    self.end_of_list = False

            # kiem tra list co hop le khong
            if self.list_id[0] != 0:
                # -- update lai van toc
                if self.ID_vel0_notTarget > 0 and self.waiting_newVelId == 0:
                    index = self.check_codeInList(self.ID_vel0_notTarget, self.list_id)
                    if index != -1:
                        if self.list_speed[index] != 0:
                            print("diem tiep theo da co van toc khac khong")
                            if self.infoAGV.dir_now != self.list_dir[index]:
                                # self.target_velNow = self.save_vel0_notTarget*(1/3)
                                # -- 
                                self.target_velNow = self.vel_movePointStop
                                print("chia 3 van toc")
                            else:
                                self.target_velNow = self.save_vel0_notTarget
                                print("van toc giu nguyen")

                            if self.target_velNow <= self.vel_linear_min:
                                self.target_velNow = self.vel_linear_min
                                print("select vel min")

                            print("vel_target now = ", self.target_velNow)

                            self.ID_vel0_notTarget = 0
                # --

                if self.waiting_newVelId == 1:
                    if self.is_idVelZero > 0:
                        # if self.flag_inEvevator == 1: # neu diem dung la diem cho thang may:
                        #     index = self.check_codeInList(self.code_inElevator, self.list_code)

                        # else:
                        print("dang cho id = ", self.is_idVelZero)
                        index = self.check_codeInList(self.is_idVelZero, self.list_id)

                        if index != -1 and self.list_speed[index] > 0 and self.list_dir[index] > 0:
                            self.just_stop = 1
                            print("free diem cho binh thuong")

                            # thay doi huong di chuyen
                            self.infoAGV.dir_follow = self.list_dir[index]

                            # cap nhat van toc
                            self.target_velNow = (self.list_speed[index]/100.)*self.vel_linear_max

                            next_index = index + 1
                            if next_index <= 4 and self.list_id[next_index] != 0 and (self.list_dir[next_index] != self.list_dir[index] or self.list_speed[next_index] == 0):
                                # self.target_velNow = self.target_velNow/3.
                                # -- 
                                self.target_velNow = self.vel_movePointStop

                            if self.target_velNow <= self.vel_linear_min:
                                self.target_velNow = self.vel_linear_min
                                print("select vel min")

                                
                            print("co tin hieu di chuyen lai. huong di chuyen moi la: ", self.infoAGV.dir_follow)
                            print("van toc update: ", self.target_velNow)

                            # -- free
                            self.ID_vel0_notTarget = 0

                            # reset bien
                            self.waiting_newVelId = 0
                            self.is_idVelZero = 0

                    self.process = 1

                else:
                    self.process = 36

            else:
                if self.infoAGV.status_move == 0:
                    print("list_id = 0 when AGV is STOP, process update list")
                    self.process = 1
                    self.job_doing = -3 # lỗi list id khi agv đang dừng

                else:
                    self.stop()
                    print("list_id = 0 when AGV is MOVING, process update list")
                    # self.process = 36
                    self.process = 1
                    self.job_doing = -4 # lỗi list id khi agv đang di chuyển

                self.end_of_list = True

        elif self.process == 36: # chay tu dong
            # kiem tra traffic da cap nhat huong chua
            if self.flag_checkDir == 1:
                self.job_doing = 2 # update hướng
                if self.infoAGV.RFID_code == self.list_code[0] and self.infoAGV.RFID_lastCode != 0 and self.infoAGV.dir_now > 0:
                    print("update dir move, dir AGV now: ", self.infoAGV.dir_now)
                    self.flag_checkDir = 0
                    self.flag_findDirectMove = 1
                    self.process = 37

                else:
                    if self.flag_runningFindDirect:
                        print("AGV chay tu dong cap nhat huong")
                        self.target_velNow = self.vel_findDirFirst
                        if self.target_velNow <= self.vel_linear_min:
                            self.target_velNow = self.vel_linear_min
                        
                        # khi tim huong mac dinh agv di chuyen bang dau
                        self.dir_move = self.moveByHead
                        self.followLine(self.dir_move, False)

                    elif self.codeNoInList == 1:
                        if self.infoAGV.RFID_code == self.list_code[0]:
                            if self.req_move.moving_dir > 0:
                                if self.dir_move == self.moveByHead:
                                    self.infoAGV.dir_now = self.req_move.moving_dir

                                elif self.dir_move == self.moveByTail:
                                    dir_now = (2 + self.req_move.moving_dir)%4
                                    if dir_now == 0:
                                        dir_now = 4

                                    self.infoAGV.dir_now = dir_now
                                print("update dir AGV special", self.infoAGV.dir_now)

                            else:
                                self.infoAGV.dir_now = self.save_oldDirAGV
                                print("update dir AGV old = %s, because dir traffic now = %s" %(self.infoAGV.dir_now, self.req_move.moving_dir))

                    self.process = 1
                    
            else:
                self.process = 37

        elif self.process == 37: # kiem tra chieu di chuyen cua agv
            # kiem tra last code va code now co trong list khong
            if self.flag_findDirectMove == 1:
                print("kiem tra chieu di chuyen agv")
                print("huong di chuyen: ",self.infoAGV.dir_follow)
                self.job_doing = 3 # update chiều di chuyển
                # indexPreCode = self.check_codeInList(self.infoAGV.RFID_lastCode, self.list_code) 
                if self.infoAGV.dir_follow <= 0: # chua co du lieu huong di chuyen:
                    print("chua co du lieu di chuyen")
                    if self.flag_runningFindDirect:
                        print("AGV chay tu dong cap nhat huong")
                        self.target_velNow = self.vel_findDirFirst
                        if self.target_velNow <= self.vel_linear_min:
                            self.target_velNow = self.vel_linear_min

                        self.followLine(self.dir_move, False)

                    index_nowCode = self.check_codeInList(self.infoAGV.RFID_code, self.list_code) 
                    if index_nowCode >= 0 and self.list_dir[index_nowCode] > 0:
                        dir_nowCode = self.list_dir[index_nowCode]

                        error_dir = dir_nowCode - self.infoAGV.dir_now
                        print("dir now code %s, dir agv now %s" %(dir_nowCode, self.infoAGV.dir_now))

                        if abs(error_dir) % 2 == 0 or self.codeNoInList:
                            self.infoAGV.dir_follow = dir_nowCode

                            # kiem tra van toc
                            if self.target_velNow == 0:
                                print("chon van toc nho nhat")
                                self.target_velNow = 0.1

                            # reset flag
                            self.flag_runningFindDirect = 0
                            self.flag_findDirectMove = 0

                            # -- 
                            self.codeNoInList = 0
                            self.just_move = 0
                            print("da cap nhat huong di chuyen :%s va huong lam viec: %s" %( self.infoAGV.dir_now, self.infoAGV.dir_follow))

                        else:
                            if self.flag_runningFindDirect == 0:
                                self.infoAGV.dir_follow = dir_nowCode
                                print("Cap nhat Dir cua 90 do")

                    elif index_nowCode >= 0 and self.list_dir[index_nowCode] == 0 and self.list_id[index_nowCode] == self.target_id:
                        print("trung target")
                        self.infoAGV.dir_follow = self.target_dir

                else:
                    # -- Nếu nhiệm vụ trước là đứng yên thì update lại hướng mới
                    if self.flag_misson0 == 1:
                        index_nowCode = self.check_codeInList(self.infoAGV.RFID_code, self.list_code) 
                        if index_nowCode >= 0 and self.list_dir[index_nowCode] > 0:
                            dir_nowCode = self.list_dir[index_nowCode]

                            self.infoAGV.dir_follow = dir_nowCode
                            self.flag_findDirectMove = 0
                            self.codeNoInList = 0
                            self.just_move = 0

                            print("flag_mission0 = 1, dir_AGV %s, dir_follow %s" %(self.infoAGV.dir_now, self.infoAGV.dir_follow))

                    else:
                        error_dir = self.infoAGV.dir_follow - self.infoAGV.dir_now

                        if self.target_velNow == 0:
                            print("chon van toc nho nhat")
                            self.target_velNow = 0.1

                        if abs(error_dir) % 2 == 0:
                            self.flag_findDirectMove = 0
                            self.just_move = 0

                        else:
                            if self.status_cross == self.before_cross:
                                self.dir_move = self.moveByTail
                                print("lui lai")

                            elif self.status_cross == self.after_cross:
                                self.dir_move = self.moveByHead
                                print("tien len")
                            
                            else:
                                self.dir_move = self.moveByHead

                            self.just_move = 1
                            self.flag_findDirectMove = 0

                        self.flag_runningFindDirect = 0        

                self.process = 1

            else:
                self.process = 38

        elif self.process == 38: # di chuyen theo dir_follow
            if self.just_move == 1:
                self.process = 39

            else:
                error_dir = (self.infoAGV.dir_follow - self.infoAGV.dir_now)%4
                if self.is_idTarget:
                    stt = self.doNextDirection(error_dir)
                    self.job_doing = 4 # quay hướng target target

                    if stt == 1:
                        self.infoAGV.dir_now = self.infoAGV.dir_follow
                        self.completed_all = 1
                        self.process = 1

                        # -- reset cross
                        self.status_cross = 0

                elif self.is_idVelZero > 0:
                    print("di chuyen vao diem ID vel = 0")
                    if self.waiting_newVelId == 0:
                        # stt = self.doNextDirection(error_dir)
                        stt = 1
                        self.stop()
                        self.job_doing = 5 # quay huong điểm vận tốc = 0
                        if stt == 1:
                            self.waiting_newVelId = 1
                            self.process = 1

                            # -- reset cross
                            # self.status_cross = 0

                else:
                    if error_dir == 0 or error_dir == 2:
                        if error_dir == 0:
                            self.dir_move = self.moveByHead

                        else:
                            self.dir_move = self.moveByTail

                        if self.infoAGV.status_move != 0 and self.infoAGV.status_move != self.dir_move:
                            self.stop()

                        # reset flag
                        self.flag_misson0 = 0

                        self.process = 39
                        self.just_stop = 0
                        
                    else:
                        stt = self.doNextDirection(error_dir)
                        if stt == 1:
                            self.infoAGV.dir_now = self.infoAGV.dir_follow

                        self.process = 1
                        # reset flag
                        self.flag_misson0 = 0

                        # -- reset cross
                        self.status_cross = 0
            
        elif self.process == 39:
            self.job_doing = 7 # di chuyển qua các điểm
            # --
            code_now = 0
            if self.dir_move == self.moveByHead:
                _code = self.allRFID.getRfidFront(self.data_RFID_front)
                if _code:
                    code_now = _code
                    self.just_move = 0

            elif self.dir_move == self.moveByTail:
                _code = self.allRFID.getRfidBehind(self.data_RFID_behind)
                if _code:
                    code_now = _code
                    self.just_move = 0

            # co tin hieu rfid
            if code_now > 0 and code_now != self.oldCodeRFID:
                # -- free
                self.ID_vel0_notTarget = 0
                # -- tim vi tri code trong list
                has_code = 0
                for index, c in enumerate(self.list_code):
                    if c == code_now:
                        has_code = 1
                        id_code = self.list_id[index]

                        self.id_pointNow = id_code

                        # -- reset cross
                        # self.status_cross = 0

                        if self.list_id[index] == self.target_id:
                            self.is_idTarget = 1
                            self.infoAGV.dir_follow = self.target_dir
                            print("da toi dich roi")

                        else:
                            # -- update van toc
                            if self.list_speed[index] == 0:
                                self.is_idVelZero = self.list_id[index]
                                self.waiting_newVelId = 0
                                print("Da gap diem Vel = 0, ID = ", self.is_idVelZero)

                            _vel = (self.list_speed[index]/100.)*self.vel_linear_max
                            next_id = index + 1

                            if next_id < 4 and self.list_id[next_id] != 0 and (self.list_dir[next_id] != self.list_dir[index] or self.list_id[next_id] == self.target_id or self.list_speed[next_id] == 0):
                                print("gap diem truoc chuyen huong, id next = ", self.list_id[next_id])
                                if self.list_speed[next_id] == 0 and self.list_id[next_id] != self.target_id:
                                    self.ID_vel0_notTarget = self.list_id[next_id]
                                    self.save_vel0_notTarget = _vel

                                # -- 
                                _vel = self.vel_movePointStop

                            self.target_velNow = _vel
                            if self.target_velNow <= self.vel_linear_min:
                                self.target_velNow = self.vel_linear_min
                                print("select vel min")

                            self.infoAGV.dir_follow = self.list_dir[index]
                            print("update new param at id: %s, code: %s, direction: %s" %(self.list_id[index], code_now, self.infoAGV.dir_follow))
                            # -- 
                            self.oldCodeRFID = code_now

                        # break

                if has_code == 0:
                    self.codeNoInList = 1
                    self.save_oldDirAGV = self.infoAGV.dir_now
                    self.infoAGV.dir_now = -1
                    self.infoAGV.dir_follow = 0
                    print("gap the ko co trong list diem, kiem tra lai huong di chuyen, AGV dang di chuyen, ID the: ", code_now)
                    print("list diem: ", self.list_code)
            
            # tu fong free rfid
            if code_now == self.oldCodeRFID:
                self.saveTimeFreeRfid = time.time()

            else:
                if time.time() - self.saveTimeFreeRfid > 3. and self.oldCodeRFID > 0:
                    self.oldCodeRFID = 0

            # -- Lưu ID hiện tại và ID follow
            # -- id now
            index_idNow = self.check_codeInList(self.infoAGV.RFID_code, self.list_code)
            self.id_agvNow = self.list_id[index_idNow]

            # -- id next
            index_idNext = index_idNow + 1
            if index_idNext <= 4 and self.list_id[index_idNext] > 0:
                self.id_agvNextFollow = self.list_id[index_idNext] 
            else:
                self.id_agvNextFollow = -1

            # nếu di chuyển sai hướng thì dừng lai
            if self.codeNoInList == 1:
                self.flag_checkDir = 1
                self.stop()

            else:
                self.followLine(self.dir_move)

            self.process = 1

        elif self.process == -3:
            if self.completed_reset == 0:
                self.stop()
                self.resetAll()
                self.completed_reset = 1
                print("Reset all")

            self.process = 1

        elif self.process == 43: # tien/lui gap vach ngang
            if self.completed_backward == 0:
                if self.req_move.mission == 0:
                    self.completed_backward = 1

                else:
                    if self.req_move.mission < 0:
                        self.status_go_horizontal_line = self.backward_horizontal_line

                    elif self.req_move.mission > 0:
                        self.status_go_horizontal_line = self.forward_horizontal_line

                    # - di chuyen
                    self.target_velNow = self.vel_moveHorizontalLine
                    if self.target_velNow <= self.vel_linear_min:
                        self.target_velNow = self.vel_linear_min

                    value_magline = -1
                    if self.status_go_horizontal_line == self.backward_horizontal_line: # lui gap vach ngang
                        value_magline = self.data_mangline_behind.value
                        self.dir_move = self.moveByTail

                    elif self.status_go_horizontal_line == self.forward_horizontal_line: # tien gap vach ngang
                        value_magline = self.data_mangline_front.value
                        self.dir_move = self.moveByHead

                    if value_magline == 20: # gap vach ngang
                        self.stop()
                        # -- ghi nho
                        # if self.dir_move == self.moveByTail:
                        #     self.dir_move = self.moveByHead

                        # else:
                        #     self.dir_move = self.moveByTail

                        self.completed_backward = 1
                        print("Gap vach ngang diem thao tac, dung lai! Dir follow = %s, Dir AGV now = %s" %(self.infoAGV.dir_follow, self.infoAGV.dir_now))

                    else:
                        self.followLine(self.dir_move, True)
                
            self.process = 1
            # 

        # update rfid code
        if self.mode_move != self.mode_moveStopHorizontaLine:
            rfid = self.allRFID.getInfoRFIDByDirectMoving(self.infoAGV.status_move)
            if rfid:
                self.infoAGV.RFID_code = rfid[0]
                self.infoAGV.RFID_lastCode = rfid[1]
        
        complete = 0
        if self.mode_move == self.mode_moveAuto:
            complete = self.completed_all
        elif self.mode_move == self.mode_moveStopHorizontaLine:
            complete = self.completed_backward

        # -- status AGV
        status_agv = 1 # agv di chuyển bt
        if self.waiting_newVelId == 1:
            status_agv = 3 # AGV dừng chờ thang máy hoặc dừng tránh
        if self.infoAGV.war_agv == 1:
            status_agv = 2 # AGv lỗi vật cản

        self.pub_respondMove(self.mode_move, self.target_id, self.target_dir, status_agv, complete, self.infoAGV.err_agv, self.process, self.infoAGV.RFID_lastCode, self.infoAGV.RFID_code, self.infoAGV.dir_now)

        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)

# Hàm xử lý khi nhấn Ctrl+C
def signal_handler(node):
    node.killnode = 1

def main():
    # -- Khoi tao ROS
    rclpy.init()
    node = MoveControl()

    # Đăng ký Ctrl+C, truyền hàm `publish_final_message` vào `signal_handler`
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(node))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')

if __name__ == '__main__':
    main()
