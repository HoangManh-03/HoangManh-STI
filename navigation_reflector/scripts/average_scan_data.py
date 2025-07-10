#!/usr/bin/env python3
"""
Msg description
    - sensor_msgs/LaserScan
      + angle_min: -3.1415927410125732 rad
      + angle_max: 3.1415927410125732 rad 
      + angle_increment: 0.004363323096185923 rad
      + time_increment: 3.4722223062999547e-05 seconds
      + scan_time: 0.05000000074505806 seconds
      + range_min: 0.0 m
      + range_max: 30.0 m
      + lenght of ranges: 1440
      + lenght of intensities: 1440
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
from navigation_reflector.msg import Raw_reflector
from openpyxl import Workbook
import matplotlib.pyplot as plt

'''
- Lấy trung bình dữ liệu 100 lần và gửi ra

'''

class Average_data():
    def __init__(self):
        rospy.init_node('graph_node', anonymous = True)
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

        rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        self.dataScan_1 = LaserScan()
        self.dataScan = LaserScan()
        self.is_scan = False

        rospy.Subscriber("/Keyboard_cmd", String, self.callback_KeyboardCmd)
        self.keyboad_cmd = String()
        
        # Đăng ký publisher cho topic dữ liệu trung bình
        self.avg_pub = rospy.Publisher('/scan_filter', LaserScan, queue_size=10)

        # -------- Topic Pub -------- #
        # -- 
        self.process = 0

        # Danh sách để lưu trữ dữ liệu quét
        self.scan_dataRanges_list = []
        self.scan_dataIntensities_list = []

        self.scan_dataRanges_filter = []
        self.scan_dataIntensities_filter = []

        self.max_scans = 10  # Số lần quét cần nhận trước khi công bố 1 | 6
        self.count_scan_filter = 0

    def callback_KeyboardCmd(self, data):
        self.keyboad_cmd = data

    def callback_scan(self, data):
        self.dataScan = data
        # print(len(self.dataScan.ranges))

        # clear all ranges which have value under 500m 
        dataRanges = list(data.ranges)

        for index in range(0, len(dataRanges) - 1):
            if dataRanges[index] > 500:
                dataRanges[index] = 0
        
        dataRanges_tuple = tuple(dataRanges)

        self.dataScan.ranges = dataRanges_tuple

        # print(len(self.dataScan.intensities))
        self.is_scan = True

    def run(self):
        while not rospy.is_shutdown():
            # if self.process == 0:
            #     if self.is_scan and self.keyboad_cmd.data == '1':
            #         self.process = 1
            #     else:
            #         self.is_scan = False

            if self.process == 0:
                if self.is_scan:
                    self.process = 1

            elif self.process == 1:       
                if self.is_scan == False:
                    self.rate.sleep()
                    continue
                
                # reset 
                self.is_scan = False

                print("Start filter scan data with time: ", self.count_scan_filter)
                # Chuyển đổi dữ liệu scan thành numpy array
                current_ranges = np.array(self.dataScan.ranges)
                current_intensities = np.array(self.dataScan.intensities)

                # Lọc các giá trị vô hạn (infinity)
                current_ranges = np.where(current_ranges == float('Inf'), np.nan, current_ranges)
                current_intensities = np.where(current_intensities == float('Inf'), np.nan, current_intensities)

                # Thêm dữ liệu quét vào danh sách
                self.scan_dataRanges_list.append(current_ranges)
                self.scan_dataIntensities_list.append(current_intensities)

                self.count_scan_filter = self.count_scan_filter + 1

                # Nếu đã nhận đủ max_scans dữ liệu, tính trung bình và công bố
                if len(self.scan_dataRanges_list) >= self.max_scans:
                    # Tính trung bình cho tất cả dữ liệu quét
                    avg_ranges = np.nanmean(np.array(self.scan_dataRanges_list), axis=0)
                    avg_intensities = np.nanmean(np.array(self.scan_dataIntensities_list), axis=0)
                
                    # Xuất bản toàn bộ dữ liệu quét
                    self.scan_dataRanges_filter = avg_ranges.tolist()  
                    self.scan_dataIntensities_filter = avg_intensities.tolist()  

                    # Tạo message LaserScan để công bố
                    avg_scan = LaserScan()
                    avg_scan.header.stamp = rospy.Time.now()
                    avg_scan.header.frame_id = self.dataScan.header.frame_id
                    avg_scan.angle_min = self.dataScan.angle_min
                    avg_scan.angle_max = self.dataScan.angle_max
                    avg_scan.angle_increment = self.dataScan.angle_increment
                    avg_scan.time_increment = self.dataScan.time_increment
                    avg_scan.range_min = self.dataScan.range_min
                    avg_scan.range_max = self.dataScan.range_max
                    
                    # Xuất bản toàn bộ dữ liệu quét
                    avg_scan.ranges = self.scan_dataRanges_filter  # Gửi toàn bộ điểm trung bình
                    avg_scan.intensities = self.scan_dataIntensities_filter  # Giữ nguyên giá trị intensities

                    # Công bố dữ liệu trung bình ra topic
                    self.avg_pub.publish(avg_scan)

                    # Xóa danh sách sau khi công bố
                    self.scan_dataRanges_list.clear()
                    self.scan_dataIntensities_list.clear()
                    self.count_scan_filter = 0

                    # step back 
                    self.process = 0

            self.rate.sleep()

def main():
    print ("--- Run Average Scan data node ---")
    program = Average_data()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




