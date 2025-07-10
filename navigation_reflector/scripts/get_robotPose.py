#!/usr/bin/env python3
"""
    -> vẽ lộ trình di chuyển theo file stiClient, và lộ trình thực tế của AGV
    -> Vẽ biểu đồ số lần báo khởi tạo lại vị trí của AGV.
"""

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Int8, String
 
from math import atan2, sin, cos, sqrt, fabs, degrees, isnan, radians
from math import pi as PI
import rospy
import time
import copy

import json
import os
import numpy as np
from scipy.optimize import minimize

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, TwistWithCovarianceStamped
from navigation_reflector.msg import *
from sti_msgs.msg import *
from openpyxl import Workbook
import matplotlib.pyplot as plt

class Average_data():
    def __init__(self):
        rospy.init_node('average_node', anonymous = True)
        self.rate = rospy.Rate(50)
        
        self.dir_pose = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/'

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
        rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback_robotpose)
        self.data_robot_pose = PoseStamped()
        self.is_recv_robotpose = False

        rospy.Subscriber("/Keyboard_cmd", String, self.callback_KeyboardCmd)
        self.keyboad_cmd = String()

        # -------- Topic Pub -------- #
        # -- 
        self.process = 0
        self.flag_mode = 0
        self.pre_flag_mode = 0
        self.is_still_running = False

        # Danh sách để lưu trữ dữ liệu quét
        # Danh sách tọa độ (x, y) của ba điểm
        self.tranjectory_points = [(-0.531, 0.184),(0.669, 0.184), (0.991, 5.771), (2.191, 5.769)]
        # Tách x và y từ danh sách điểm
        self.ctime_get_pose = time.time()
        self.time_getpose = 1

        self.ls_pose_x = []
        self.ls_pose_y = []
        self.ls_pose_z = []
        self.ls_pose_w = []
        self.ls_pose_exec = []

        self.pre_taskstatus = 0
        self.count = 0

        # -

    def callback_KeyboardCmd(self, data):
        self.keyboad_cmd = data

        if self.keyboad_cmd.data == 's':     #reset
            print("################################## STOP && RESET ALL !!!######################################")
            # self.flag_mode = 7
            # self.stop()
            self.process = -1
            # self.pre_flag_mode = 0
            # self.step = 0
            self.is_still_running = False

        elif self.keyboad_cmd.data == '1':
            self.flag_mode = 1
            print("Bắt đầu thu thập dữ liệu lidar")

        elif self.keyboad_cmd.data == '2':
            self.flag_mode = 2
            print("Bắt đầu vẽ bản đồ")
        
        elif self.keyboad_cmd.data == '3':
            self.flag_mode = 3
            # print("Chế độ quay start. Hãy nhâp góc quay")
        
        elif self.keyboad_cmd.data == '4':
            self.flag_mode = 4
        
        elif self.keyboad_cmd.data == '5':
            self.flag_mode = 5
        
        elif self.keyboad_cmd.data == '6':
            self.flag_mode = 6        

    def callback_robotpose(self, data):
        self.data_robot_pose = data
        self.is_recv_robotpose = True

    def add_pose(self, filename, x, y, z, w):
        # Dữ liệu cần lưu
        data = {
            "position": [x, y, 0],
            "orientation": [0, 0, z, w]
        }        

        # temp_file_path = filename + '.tmp' 
        # try:
        #     with open(temp_file_path, 'w', encoding='utf-8') as temp_file:
        #         json.dump(data, temp_file, ensure_ascii=False, indent=4)
        #         temp_file.flush()
        #         os.fsync(temp_file.fileno())
        #     os.rename(temp_file_path, filename)

        # except Exception as e:
            # print("Write json file backup error", e)
        with open(filename, "w", encoding="utf-8") as file:
            json.dump(data, file, indent=4, ensure_ascii=False)

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
                    print("change mode to", self.process)
                # self.log_mess("info","Can use teleop keyboard", 0)

            # vẽ lộ trình từ NN_cmdRequest
            elif self.process == 1:       
                if self.is_recv_robotpose == False:
                    self.rate.sleep()
                    continue
                    
                self.is_recv_robotpose = False

                numave = 20 
                if self.count < 20:
                    print("Start get data")
                    self.ctime_get_pose = time.time()
                    x = self.data_robot_pose.pose.position.x
                    y = self.data_robot_pose.pose.position.y
                    z = self.data_robot_pose.pose.orientation.z
                    w = self.data_robot_pose.pose.orientation.w

                    self.ls_pose_x.append(x)
                    self.ls_pose_y.append(y)
                    self.ls_pose_z.append(z)
                    self.ls_pose_w.append(w)

                    self.count += 1
                else:
            
                    # ghi vào file json
                    pose_x = np.array(self.ls_pose_x)
                    pose_y = np.array(self.ls_pose_y)
                    pose_z = np.array(self.ls_pose_z)
                    pose_w = np.array(self.ls_pose_w)

                    mean_x = np.mean(pose_x)
                    mean_y = np.mean(pose_y)
                    mean_z = np.mean(pose_z)
                    mean_w = np.mean(pose_w)

                    file_name = input("Nhập tên file JSON (không cần .json): ") + ".json"
                    
                    file_path = os.path.join(self.dir_pose, file_name)
                    self.add_pose(file_path, mean_x, mean_y, mean_z, mean_w)
                    print(f"Đã tạo và lưu file {file_path} thành công!")

                    self.process = -1
                    self.ls_pose_x = []
                    self.ls_pose_y = []
                    self.ls_pose_z = []
                    self.ls_pose_w = []

                    self.count = 0

            self.rate.sleep()

def main():
    print ("--- Run Average Scan data node ---")
    program = Average_data()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




