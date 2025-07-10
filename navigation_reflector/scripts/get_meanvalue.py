#!/usr/bin/env python3
"""
    Tìm trung bình và sai số của list kết quả
"""

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Int8, String
 
from math import atan2, sin, cos, sqrt, fabs, degrees, isnan, radians
from math import pi as PI
import rospy
import time
import copy

import json

import numpy as np
from scipy.optimize import minimize

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from navigation_reflector.msg import *
from openpyxl import Workbook
import matplotlib.pyplot as plt

class Average_data():
    def __init__(self):
        rospy.init_node('average_node', anonymous = True)
        self.rate = rospy.Rate(50)
        
        # -- param
        self.radius_reflector = rospy.get_param('~radius_reflector', 0.03)
        self.tolerance_radiusRelector = 0.01

        self.diameter_reflector = 2.0*self.radius_reflector
        self.circumference_reflector = 2.0*self.radius_reflector*PI
        self.half_circumference_reflector = self.radius_reflector*PI

        self.min_scanningRadius = rospy.get_param('~min_scanningRadius', 0.1)
        self.max_scanningRadius = rospy.get_param('~max_scanningRadius', 30.)

        self.min_reflectionIntensity = rospy.get_param('~min_reflectionIntensity', 1000)  # 970
        self.max_reflectionIntensity = rospy.get_param('~max_reflectionIntensity', 5000)

        # -- 
        self.x_base_to_lidar = rospy.get_param('~x_base_to_lidar', 0.361)
        self.y_base_to_lidar = rospy.get_param('~y_base_to_lidar', 0.261)
        self.r_base_to_lidar = rospy.get_param('~r_base_to_lidar', radians(45.))

        # -------- Topic Sub -------- #

        rospy.Subscriber("/r2000_data", R2000_data, self.callback_lidar)
        self.data_lidar = R2000_data()
        self.is_lidar = False

        rospy.Subscriber("/r2000_reflectors", R2000_reflectors, self.callback_reflector)
        self.data_ref = R2000_reflectors()
        self.is_ref = False

        rospy.Subscriber("/Keyboard_cmd", String, self.callback_KeyboardCmd)
        self.keyboad_cmd = String()

        # -------- Topic Pub -------- #
        # -- 
        self.process = 0
        self.flag_mode = 0
        self.pre_flag_mode = 0
        self.is_still_running = False

        # Danh sách để lưu trữ dữ liệu quét
        self.lidar_pose = []
        self.count = 0

        self.list_pose_x = []
        self.list_pose_y = []
        self.list_pose_phi = []

        self.ref1_x = []
        self.ref1_y = []
    def callback_KeyboardCmd(self, data):
        self.keyboad_cmd = data

        if self.keyboad_cmd.data == 's':     #reset
            print("################################## STOP && RESET ALL !!!######################################")
            # self.flag_mode = 7
            # self.stop()
            self.mode = -1
            # self.step = 0
            self.is_still_running = False

        elif self.keyboad_cmd.data == '1':
            self.flag_mode = 1
            print("Bắt đầu thu thập dữ liệu r2000_data")

        elif self.keyboad_cmd.data == '2':
            self.flag_mode = 2
            print("Bắt đầu thu thập dữ liệu r2000_ref")
        
        elif self.keyboad_cmd.data == '3':
            self.flag_mode = 3
            # print("Chế độ quay start. Hãy nhâp góc quay")
        
        elif self.keyboad_cmd.data == '4':
            self.flag_mode = 4
        
        elif self.keyboad_cmd.data == '5':
            self.flag_mode = 5
        
        elif self.keyboad_cmd.data == '6':
            self.flag_mode = 6        

    def callback_lidar(self, data):
        self.data_lidar = data
        self.is_lidar = True

    def callback_reflector(self, data):
        self.data_ref = data
        self.is_ref = True

    def run(self):
        while not rospy.is_shutdown():
            if self.process == 0:
                # if self.is_lidar == True and self.is_ref == True:
                self.process = -1

            # -- Stop and reset variables
            elif self.process == -1:
                # - wait keyboard cmd and program in wait mode
                if self.pre_flag_mode != self.flag_mode and self.is_still_running == False:
                    self.process = self.flag_mode
                    self.pre_flag_mode = self.flag_mode
                    self.step = 0
                    self.is_still_running = True
                # self.log_mess("info","Can use teleop keyboard", 0)

            elif self.process == 1:       
                if self.is_lidar == False:
                    self.rate.sleep()
                    continue
                    
                self.is_lidar = False
                
                # --
                self.list_pose_x.append(self.data_lidar.x)
                self.list_pose_y.append(self.data_lidar.y)
                self.list_pose_phi.append(self.data_lidar.phi)
                self.count += 1
                print("Đang lấy dữ liệu lần: ", self.count)

                if self.count == 10:
                    A = np.array(self.list_pose_x) # map
                    B = np.array(self.list_pose_y) # gương
                    C = np.array(self.list_pose_phi) # gương
                    # Tính trung bình của A và B
                    mu_A = np.mean(A, axis=0)
                    mu_B = np.mean(B, axis=0)
                    mu_C = np.mean(C, axis=0)

                    # Tái căn giữa các điểm (A_n và B_n)
                    A_n = A - mu_A
                    B_n = B - mu_B
                    C_n = C - mu_C

                    max_An = np.max(A_n)
                    min_An = np.min(A_n)

                    max_Bn = np.max(B_n)
                    min_Bn = np.min(B_n)

                    max_Cn = np.max(C_n)
                    min_Cn = np.min(C_n)

                    print(f"Toạ dộ trung bình của lidar là: x = {mu_A}, y = {mu_B}, phi = {degrees(mu_C)}")
                    print(f"Sai số max cuả x là: {max_An}, min là: {min_An}")
                    print(f"Sai số max cuả y là: {max_Bn}, min là: {min_Bn}")
                    print(f"Sai số max cuả phi là: {max_Cn}, min là: {min_Cn}")

                    self.process = -1
                    self.list_pose_x = []
                    self.list_pose_y = []
                    self.list_pose_phi = []
                    self.pre_flag_mode = 0
                    self.count = 0
                
                # step back 
                # self.process = 0

            elif self.process == 2:
                """
                    Toạ dộ trung bình của gương 1 là: x = -5.389937229715766, y = 3.6974799947522685
                    Sai số max cuả x là: 0.004617852749348117, min là: -0.005552380738190088
                    Sai số max cuả y là: 0.0038089157308767163, min là: -0.003167832468492726
                    ----
                    Toạ dộ trung bình của gương 2 là: x = -5.606431522964341, y = -3.4558475518632235
                    Sai số max cuả x là: 0.004511684668570659, min là: -0.0040010683251301415
                    Sai số max cuả y là: 0.0027810371629071184, min là: -0.002466289317831105
                    ---
                    => Tính ra sai số khi tính khoảng cách giưã 2 gương d tại giá trị min và max: 0.0025 m
                """
                if self.is_ref == False:
                    self.rate.sleep()
                    continue
                    
                self.is_ref = False

                # -- tìm list gương detected
                ls_detected_ref_x = []
                ls_detected_ref_y = []
                for index, ref in enumerate(self.data_ref.reflectors):
                    ls_detected_ref_x.append(ref.Cart_X)
                    ls_detected_ref_y.append(ref.Cart_Y)

                # -- trung bình 10 lần của 1 gương
                self.ref1_x.append(ls_detected_ref_x[1])
                self.ref1_y.append(ls_detected_ref_y[1])
                self.count += 1
                print("Đang lấy dữ liệu lần: ", self.count)

                if self.count == 10:
                    A = np.array(self.ref1_x) 
                    B = np.array(self.ref1_y) 
                    # Tính trung bình của A và B
                    mu_A = np.mean(A, axis=0)
                    mu_B = np.mean(B, axis=0)

                    # Tái căn giữa các điểm (A_n và B_n)
                    A_n = A - mu_A
                    B_n = B - mu_B

                    max_An = np.max(A_n)
                    min_An = np.min(A_n)

                    max_Bn = np.max(B_n)
                    min_Bn = np.min(B_n)

                    print(f"Toạ dộ trung bình của gương 1 là: x = {mu_A}, y = {mu_B}")
                    print(f"Sai số max cuả x là: {max_An}, min là: {min_An}")
                    print(f"Sai số max cuả y là: {max_Bn}, min là: {min_Bn}")

                    self.process = -1
                    self.ref1_x = []
                    self.ref1_y = []
                    self.pre_flag_mode = 0
                    self.count = 0

            self.rate.sleep()

def main():
    print ("--- Run Average Scan data node ---")
    program = Average_data()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




