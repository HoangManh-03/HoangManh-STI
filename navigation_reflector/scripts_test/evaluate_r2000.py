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
        rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.callback_NNcmdRequest)
        self.data_NN = NN_cmdRequest()
        self.is_recvNN = False

        rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.callback_NNinfoRespond)
        self.data_NNinfo = NN_infoRespond()
        self.is_recvNNinfo = False

        rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback_robotpose)
        self.data_robot_pose = PoseStamped()
        self.is_recv_robotpose = False

        rospy.Subscriber("/Keyboard_cmd", String, self.callback_KeyboardCmd)
        self.keyboad_cmd = String()

        rospy.Subscriber('/raw_vel', TwistWithCovarianceStamped, self.callback_rawvel, queue_size = 10)
        self.data_rawvel = TwistWithCovarianceStamped()
        self.is_recv_rawvel = False

        # -------- Topic Pub -------- #
        # -- 
        self.process = 0
        self.flag_mode = 0
        self.pre_flag_mode = 0
        self.is_still_running = False

        # Danh sách để lưu trữ dữ liệu quét
        # Danh sách tọa độ (x, y) của ba điểm
        # self.tranjectory_points = [(3.696, 1.031),(-1.224, 1.804)]
        # self.tranjectory_points = [(-0.531, 0.184),(0.669, 0.184), (0.991, 5.771), (2.191, 5.769)]
        # self.tranjectory_points = [(-0.278, 8.262),(-0.478, 6.262), (-0.578, 3.262), (-0.651, 0.44), (1.46, 0.241), (1.521, 2.425), (1.676, 5.298), (-0.278, 8.262)]
        self.tranjectory_points = [(-0.737, -5.725),(1.463, 0.521), (2.454, 0.544)]    # for file fake 7
        
        # Tách x và y từ danh sách điểm
        self.ctime_get_pose = time.time()
        self.time_getpose = 2

        self.ls_pose = []
        self.ls_pose_exec_point1 = []
        self.ls_pose_exec_point2 = []
        self.ls_pose_exec_point3 = []
        self.ls_pose_exec_point4 = []
        self.ls_pose_exec_point5 = []
        self.ls_pose_exec_point6 = []

        self.pre_taskstatus = 0
        self.is_first_getpose1 = False
        self.is_first_getpose2 = False
        self.is_first_getpose3 = False
        self.is_first_getpose4 = False
        self.is_first_getpose5 = False
        self.is_first_getpose6 = False

        self.point1x = 0
        self.point1y = 0
        self.count1 = 0
        
        self.point2x = 0
        self.point2y = 0
        self.count2 = 0

        self.point3x = 0
        self.point3y = 0
        self.count3 = 0
        
        self.point4x = 0
        self.point4y = 0
        self.count4 = 0

        self.point5x = 0
        self.point5y = 0
        self.count5 = 0
        
        self.point6x = 0
        self.point6y = 0
        self.count6 = 0

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

    def callback_NNcmdRequest(self, data):
        self.data_NN = data
        self.is_recvNN = True

    def callback_NNinfoRespond(self, data):
        self.data_NNinfo = data
        self.is_recvNNinfo = True

    def callback_robotpose(self, data):
        self.data_robot_pose = data
        self.is_recv_robotpose = True

    def callback_rawvel(self, data):
        self.data_rawvel = data
        self.is_recv_rawvel = True

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

                if self.data_rawvel.twist.twist.linear.x == 0.0 and self.data_rawvel.twist.twist.angular.z == 0:
                    self.time_getpose = 4
                else:
                    self.time_getpose = 2

                if time.time() - self.ctime_get_pose > self.time_getpose:
                    # print("Start get data")
                    self.ctime_get_pose = time.time()
                    x = self.data_robot_pose.pose.position.x
                    y = self.data_robot_pose.pose.position.y

                    self.ls_pose.append((x,y))
                
                # -- for lộ trình hcn 
                if self.data_NN.target_x == -0.651 and self.data_NN.target_y == 0.44:
                    if self.data_NNinfo.task_status == 66 and self.data_NNinfo.process == 8:
                        print("Start get pose point 1 exec")
                        x = self.data_robot_pose.pose.position.x
                        y = self.data_robot_pose.pose.position.y

                        self.point1x += x
                        self.point1y += y
                        self.count1 += 1
                        self.is_first_getpose1 = True
                
                else:
                    if self.is_first_getpose1 == True:
                        self.is_first_getpose1 = False
                        self.ls_pose_exec_point1.append((self.point1x/ self.count1, self.point1y/self.count1))

                        print(f"Vị trí gương trung bình là: {self.point1x/ self.count1} và {self.point1y/self.count1}")
                        self.count1 = 0
                        self.point1x = 0
                        self.point1y = 0

                if self.data_NN.target_x == -0.278 and self.data_NN.target_y == 8.262:
                    if self.data_NNinfo.task_status == 65 and self.data_NNinfo.process == 8:
                        # if self.is_first_getpose == True:
                        print("Start get pose point 2 exec")
                        x = self.data_robot_pose.pose.position.x
                        y = self.data_robot_pose.pose.position.y

                        self.point2x += x
                        self.point2y += y
                        self.count2 += 1
                        self.is_first_getpose2 = True

                else:
                    if self.is_first_getpose2 == True:
                        self.is_first_getpose2 = False
                        self.ls_pose_exec_point2.append((self.point2x/self.count2, self.point2y/self.count2))

                        print(f"Vị trí số2  trung bình là: {self.point2x/ self.count2} và {self.point2y/self.count2}")
                        self.count2 = 0
                        self.point2x = 0
                        self.point2y = 0                                       

                # -- for lộ trình đươnfg thẳng tự quét
                if self.data_NN.target_x == -0.737 and self.data_NN.target_y == -5.725:
                    if self.data_NNinfo.task_status == 66 and self.data_NNinfo.process == 8:
                        # if self.is_first_getpose == True:
                        print("Start get pose point 3 exec")
                        x = self.data_robot_pose.pose.position.x
                        y = self.data_robot_pose.pose.position.y

                        self.point3x += x
                        self.point3y += y
                        self.count3 += 1
                        self.is_first_getpose3 = True

                else:
                    if self.is_first_getpose3 == True:
                        self.is_first_getpose3 = False
                        self.ls_pose_exec_point3.append((self.point3x/self.count3, self.point3y/self.count3))

                        print(f"Vị trí số 3 trung bình là: {self.point3x/ self.count3} và {self.point3y/self.count3}")
                        self.count3 = 0
                        self.point3x = 0
                        self.point3y = 0        

                if self.data_NN.target_x == 2.454 and self.data_NN.target_y == 0.544:
                    if self.data_NNinfo.task_status == 66 and self.data_NNinfo.process == 8:
                        print("Start get pose point 4 exec")
                        x = self.data_robot_pose.pose.position.x
                        y = self.data_robot_pose.pose.position.y

                        self.point4x += x
                        self.point4y += y
                        self.count4 += 1
                        self.is_first_getpose4 = True
                
                else:
                    if self.is_first_getpose4 == True:
                        self.is_first_getpose4 = False
                        self.ls_pose_exec_point4.append((self.point4x/ self.count4, self.point4y/self.count4))

                        print(f"Vị trí gương trung bình là: {self.point4x/ self.count4} và {self.point4y/self.count4}")
                        self.count4 = 0
                        self.point4x = 0
                        self.point4y = 0

                # -- for lộ trình chaỵ trên map gương nav350
                if self.data_NN.target_x == -0.736 and self.data_NN.target_y == -7.257:
                    if self.data_NNinfo.task_status == 66 and self.data_NNinfo.process == 8:
                        # if self.is_first_getpose == True:
                        print("Start get pose point 5 exec")
                        x = self.data_robot_pose.pose.position.x
                        y = self.data_robot_pose.pose.position.y

                        self.point5x += x
                        self.point5y += y
                        self.count5 += 1
                        self.is_first_getpose5 = True

                else:
                    if self.is_first_getpose5 == True:
                        self.is_first_getpose5 = False
                        self.ls_pose_exec_point5.append((self.point5x/self.count5, self.point5y/self.count5))

                        print(f"Vị trí số 5 trung bình là: {self.point5x/ self.count5} và {self.point5y/self.count5}")
                        self.count5 = 0
                        self.point5x = 0
                        self.point5y = 0        

                if self.data_NN.target_x == -0.813 and self.data_NN.target_y == -1.416:
                    if self.data_NNinfo.task_status == 66 and self.data_NNinfo.process == 8:
                        print("Start get pose point 6 exec")
                        x = self.data_robot_pose.pose.position.x
                        y = self.data_robot_pose.pose.position.y

                        self.point6x += x
                        self.point6y += y
                        self.count6 += 1
                        self.is_first_getpose6 = True
                
                else:
                    if self.is_first_getpose6 == True:
                        self.is_first_getpose6 = False
                        self.ls_pose_exec_point6.append((self.point6x/ self.count6, self.point6y/self.count6))

                        print(f"Vị trí gương 6 trung bình là: {self.point6x/ self.count6} và {self.point6y/self.count6}")
                        self.count6 = 0
                        self.point6x = 0
                        self.point6y = 0

            # - vẽ đồ thị lộ trình
            elif self.process == 2:
                print("Start map result graph", len(self.ls_pose))
                # print("List các điểm vị trí point 1 cuả AGV là:", len(self.ls_pose_exec_point1))
                # print("List các điểm vị trí point 2 cuả AGV là:", len(self.ls_pose_exec_point2))

                print("List các điểm vị trí point 3 cuả AGV là:", len(self.ls_pose_exec_point3))
                print("List các điểm vị trí point 4 cuả AGV là:", len(self.ls_pose_exec_point4))

                # - vẽ đường tham chiếu
                x_values, y_values = zip(*self.tranjectory_points)

                # Vẽ đồ thị
                # plt.plot(x_values, y_values, marker='o', linestyle='-', color='b', label="Lộ trình của AGV")

                x2, y2 = zip(*self.ls_pose)
                # Vẽ đường thứ hai (màu đỏ)
                # plt.plot(x2, y2, marker='s', linestyle='--', color='r', label="y = Vị trí thực tế cuả AGV")

                # Định dạng đồ thị
                # Tạo figure với 2 subplot (1 hàng, 2 cột)
                fig, axs = plt.subplots(1, 3, figsize=(15, 8))  # 1 hàng, 2 cột

                # Vẽ lộ trình
                axs[0].plot(x_values, y_values, marker='o', linestyle='-', color='b', label="Lộ trình của AGV")
                axs[0].plot(x2, y2, marker='s', linestyle='--', color='r', label="y = Vị trí thực tế cuả AGV")
                axs[0].set_title("Lộ trình AGV di chuyển")
                axs[0].set_xlabel("X")
                axs[0].set_ylabel("Y")
                axs[0].legend()
                axs[0].grid()

                # Vẽ sai số tại điểm

                # if len(self.ls_pose_exec_point1) > 0:
                pointx2, pointy2 = zip(*self.ls_pose_exec_point3)

                # Vẽ điểm nhóm 1
                axs[1].scatter(pointx2, pointy2, color='red', marker='o', label='ID 4')

                # Vẽ điểm nhóm 2

                # axs[1].plot(x2, y2, color='r', linestyle="--", label="Sai số vị trí cuả AGV tại ID1")
                # axs[1].plot(x3, y3, color='b', linestyle="-", label="Sai số vị trí cuả AGV tại ID2")

                axs[1].set_title("Vị trí cuả AGV tại điểm thao tác")
                axs[1].set_xlabel("X")
                axs[1].set_ylabel("Y")
                axs[1].legend()
                axs[1].grid()

                # if len(self.ls_pose_exec_point2) > 0:
                pointx3, pointy3 = zip(*self.ls_pose_exec_point4)

                # Vẽ điểm nhóm 2
                axs[2].scatter(pointx3, pointy3, color='blue', marker='s', label='ID 1')

                # axs[1].plot(x2, y2, color='r', linestyle="--", label="Sai số vị trí cuả AGV tại ID1")
                # axs[1].plot(x3, y3, color='b', linestyle="-", label="Sai số vị trí cuả AGV tại ID2")

                axs[2].set_title("Vị trí cuả AGV tại điểm thao tác")
                axs[2].set_xlabel("X")
                axs[2].set_ylabel("Y")
                axs[2].legend()
                axs[2].grid()

                # Hiển thị
                plt.tight_layout()  # Điều chỉnh khoảng cách giữa các subplot
                plt.show()

                self.process = -1
                self.ls_pose = []
                self.ls_pose_exec = []

            self.rate.sleep()

def main():
    print ("--- Run Average Scan data node ---")
    program = Average_data()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




