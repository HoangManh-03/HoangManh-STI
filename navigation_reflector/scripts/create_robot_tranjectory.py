#!/usr/bin/env python3
"""
*** infomation

*** Task Description: 
    Phase 1: Nhận điều khiển từ bàn phím, cho phép robot chạy các dạng lộ trình khác nhau để build chế độ navigation
        - Phím '1': Chạy đoạn A -> B với khoảng cách 5m
        - Phím '2': Chạy đoạn B -> A với khoảng cách 5m
        - Phím '3': Quay cùng chiều
        - Phím '4': Quay ngược chiều
        - Phím '5': Đi theo quy đạo có bán kính R cùng chiều kim đồng hồ
        - Phím '6': Đi theo quy đạo có bán kính R ngược chiều kim đồng hồ

        - Phím 'A': Cho phép remote robot tới vị trí start
        - Phím 'S': Dừng robot và reset

    Phase 2: Thay đổi tốc độ của robot theo các trường hợp quỹ đạo trên

*** Need to do
    ...
"""

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Int8, String
 
from math import atan2, sin, cos, sqrt, fabs, degrees, isnan, radians, log, exp, asin, acos, tan
from math import pi as PI
import rospy
import time
import copy

import json

import numpy as np
from scipy.optimize import minimize

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped, TwistWithCovarianceStamped, Twist
from navigation_reflector.msg import Raw_reflector, Init_mode
from nav_msgs.msg import Odometry

from itertools import combinations

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Tranjectory_mode():
    def __init__(self):
        rospy.init_node('initialization_node', anonymous = True)
        self.rate = rospy.Rate(50)

        # -- ros pub && sub
        rospy.Subscriber("/Keyboard_cmd", String, self.callback_KeyboardCmd)
        self.keyboad_cmd = String()        

        rospy.Subscriber('/odom', Odometry, self.callback_odom, queue_size = 10)
        self.data_odom = Odometry()
        self.is_recv_odom = False

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)

        # -- constant variables
        self.V_XMAX = 0.5     # m/s
        self.V_YMAX = 0.0     # m/s
        self.OMEGA_VMAX = 0.1  # rad/s

        self.THRESHOLD_DISTANCE = 1 #m 
        self.rate_cmdvel = 25
        self.tolerance_rot_step1 = 0.02
        self.vel_rot_step1 = 0.45
        self.angle_giam_toc = 45.0*PI/180.0

        # -- global variables
        self.mode = 0
        self.step = 0

        self.pre_mode = self.mode
        self.flag_mode = 0
        self.pre_flag_mode = self.flag_mode
        self.is_still_running = False

        self.x_start = 0.0
        self.y_start = 0.0
        self.z_start = 0.0

        self.pre_mess = ''
        self.time_tr = rospy.get_time()

        self.angle_target = 0.0
        self.theta_robot = 0.0

        self.direction = 1

        # # -- Khai báo tf -- 
        # self.br = tf.TransformBroadcaster()
        # self.ref_frame = "map"
        # self.origin_frame = "ref1"

        # self.translation = (0,0,0)
        # self.quanternion = quaternion_from_euler(0, 0, 0)

    def callback_KeyboardCmd(self, data):
        self.keyboad_cmd = data

        if self.keyboad_cmd.data == 's':     #reset
            print("################################## STOP && RESET ALL !!!######################################")
            # self.flag_mode = 7
            self.stop()
            self.mode = -1
            self.step = 0
            self.is_still_running = False

        # elif self.keyboad_cmd.data == 'A':     # use teleop
        #     print("################################## ALLOW USE TELEOP !!!######################################")
        #     self.flag_mode = 8

        elif self.keyboad_cmd.data == '1':
            self.flag_mode = 1
            print("Chế độ chạy tiến và chạy lùi start. Mời chọn hướng")

        elif self.keyboad_cmd.data == '2':
            self.flag_mode = 2
        
        elif self.keyboad_cmd.data == '3':
            self.flag_mode = 3
            print("Chế độ quay start. Hãy nhâp góc quay")
        
        elif self.keyboad_cmd.data == '4':
            self.flag_mode = 4
        
        elif self.keyboad_cmd.data == '5':
            self.flag_mode = 5
        
        elif self.keyboad_cmd.data == '6':
            self.flag_mode = 6

    def callback_odom(self, data):
        self.data_odom = data
        self.is_recv_odom = True

        quaternion1 = (data.pose.pose.orientation.x, data.pose.pose.orientation.y,\
                    data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion1)

        self.theta_robot = round(euler[2], 3)

    def euler_to_quaternion(self, euler):
        quat = Quaternion()
        odom_quat = quaternion_from_euler(0, 0, euler)
        quat.x = odom_quat[0]
        quat.y = odom_quat[1]
        quat.z = odom_quat[2]
        quat.w = odom_quat[3]
        return quat

    def log_mess(self, typ, mess, val):
        if self.pre_mess != mess:
            if typ == "info":
                rospy.loginfo (mess + ": %s", val)
            elif typ == "warn":
                rospy.logwarn (mess + ": %s", val)
            else:
                rospy.logerr (mess + ": %s", val)
        self.pre_mess = mess

    def stop(self):
        for i in range(2):
            self.pub_cmd_vel.publish(Twist())

    def pub_cmdVel(self, twist , rate):
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass

    def turn_ar(self, theta, tol_theta, vel_rot):
        if fabs(theta) > tol_theta: # +- 10 do
            if theta > 0: #quay trai
                # print "b"
                if fabs(theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhh')
                    vel_th = (fabs(theta)/self.angle_giam_toc)*vel_rot
                else:
                    vel_th = vel_rot

                if vel_th < 0.1:
                    vel_th = 0.1

                # vel_th = fabs(theta) + 0.1
                # if vel_th > vel_rot : vel_th = vel_rot
                return vel_th

            if theta < 0: #quay phai , vel_z < 0
                # print "a"
                if fabs(theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhhh')
                    vel_th = (fabs(theta)/self.angle_giam_toc)*(-vel_rot)
                else:
                    vel_th = -vel_rot

                if vel_th > -0.1:
                    vel_th = -0.1

                # vel_th = -fabs(theta) - 0.1
                # if vel_th < -vel_rot : vel_th = -vel_rot
                return vel_th
                # buoc = 1

        else : 
            return -10 
          
    def run(self):
        while not rospy.is_shutdown():
            # -- Chờ nhận được full các dữ liệu
            if self.mode == 0:
                if self.is_recv_odom == True:
                    self.mode = -1
                else:
                    self.log_mess("warn","Wait data odom from kinematic node", 0)
            
            # -- Stop and reset variables
            elif self.mode == -1:
                # - wait keyboard cmd and program in wait mode
                if self.pre_flag_mode != self.flag_mode and self.is_still_running == False:
                    self.mode = self.flag_mode
                    self.pre_flag_mode = self.flag_mode
                    self.step = 0
                
                self.log_mess("info","Can use teleop keyboard", 0)

            # -- robot run for 5m and stop
            elif self.mode == 1:
                if self.step == 0:       # record start pose of robot
                    if self.keyboad_cmd.data == 'i':
                        self.direction = 1
                        print("Chiều tiến được chọn")
                    elif self.keyboad_cmd.data == 'm':
                        self.direction = -1
                        print("Chiều lùi được chọn")
                    elif self.keyboad_cmd.data == 'k':
                        print("Bắt đầu chaỵ")
                        self.step = 1
                        self.x_start = self.data_odom.pose.pose.position.x
                        self.y_start = self.data_odom.pose.pose.position.y

                elif self.step == 1:
                    # send vel to robot
                    twist = Twist()
                    twist.linear.x = self.direction*self.V_XMAX
                    twist.angular.z = 0

                    self.pub_cmdVel(twist, self.rate_cmdvel)

                    # -- update state variables
                    self.is_still_running = True

                    # -- tính toán khoảng cách
                    dx = self.data_odom.pose.pose.position.x - self.x_start
                    dy = self.data_odom.pose.pose.position.y - self.y_start

                    d = sqrt(dx*dx + dy*dy)
                    print(f"Khoảng cách hiện tại là: {d}")

                    if d >= self.THRESHOLD_DISTANCE:
                        self.stop()
                        self.is_still_running = False
                        self.step = 0
            
            # -- điêù khiển robot quay tới góc mong muốn   
            elif self.mode == 3:
                if self.step == 0:
                    if self.keyboad_cmd.data == 'j':
                        self.angle_target = 0*PI/180
                    elif self.keyboad_cmd.data == 'i':
                        self.angle_target = 90*PI/180
                    elif self.keyboad_cmd.data == 'l':
                        self.angle_target = PI
                    elif self.keyboad_cmd.data == 'm':
                        self.angle_target = 270*PI/180   
                    
                    if self.keyboad_cmd.data == 'k':    # run
                        self.step = 1
                        print("Góc mục tiêu là:", self.angle_target)

                elif self.step == 1:
                    theta = self.angle_target - self.theta_robot
                    print(f"Góc còn lại là: {theta}")
                    gt = self.turn_ar(theta, self.tolerance_rot_step1, self.vel_rot_step1)
                    if gt == -10:
                        self.stop()
                        self.is_still_running = False
                        self.step = 0
                    else:
                        twist = Twist()
                        twist.angular.z = gt
                        self.is_still_running = True
                        self.pub_cmdVel(twist,self.rate_cmdvel)

            
            self.rate.sleep()

def main():
    print ("--- Run create tranjectory mode ---")
    program = Tranjectory_mode()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




