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
from std_msgs.msg import Int8
 
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
- Vẽ đồ thị thông tin ranges và intensities của gương sau bộ lọc
    + trung bình cộng
    + trung bình nhân

- Check intensity of reflector in following distance
'''

class Graph_node():
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

        self.min_reflectionIntensity = rospy.get_param('~min_reflectionIntensity', 700)  # 970
        self.max_reflectionIntensity = rospy.get_param('~max_reflectionIntensity', 5000)

        # -- 
        self.x_base_to_lidar = rospy.get_param('~x_base_to_lidar', 0.361)
        self.y_base_to_lidar = rospy.get_param('~y_base_to_lidar', 0.261)
        self.r_base_to_lidar = rospy.get_param('~r_base_to_lidar', radians(45.))

        # -------- Topic Sub -------- #

        rospy.Subscriber("/scan_lms100", LaserScan, self.callback_scan)
        self.dataScan_1 = LaserScan()
        self.dataScan = LaserScan()
        self.is_scan = False

        # -------- Topic Pub -------- #
        # -- 
        self.process = 0
        self.record_status = 0

        self.wb = Workbook()
        self.ws = self.wb.active
        self.ws.title = 'Data scanning from R2000'

        # Ghi tiêu đề cột
        self.ws.cell(row=1, column=1, value="Ranges")
        self.ws.cell(row=1, column=2, value="Intensity")

        # Danh sách để lưu trữ dữ liệu quét
        self.scan_dataRanges_list = []
        self.scan_dataIntensities_list = []

        self.scan_dataRanges_filter = []
        self.scan_dataIntensities_filter = []

        self.reflector_dataRanges_list = []
        self.reflector_dataIntensities_list = []
        self.reflector_dataIndex_list = []

        self.max_scans = 1000  # Số lần quét cần nhận trước khi công bố 1 | 6
        self.count_scan_filter = 0

    def callback_scan(self, data):
        self.dataScan = data
        # print(self.dataScan.ranges)

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
            if self.process == 0:
                if self.is_scan:
                    self.process = 1

            elif self.process == 1:       
                if self.is_scan == False:
                    self.rate.sleep()
                    continue
                
                # reset 
                self.is_scan = False

                print("Start filter scan data wiht time: ", self.count_scan_filter)
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

                # Nếu đã nhận đủ 5 dữ liệu, tính trung bình và công bố
                if len(self.scan_dataRanges_list) >= self.max_scans:
                    # Tính trung bình cho tất cả dữ liệu quét
                    avg_ranges = np.nanmean(np.array(self.scan_dataRanges_list), axis=0)
                    avg_intensities = np.nanmean(np.array(self.scan_dataIntensities_list), axis=0)
                
                    # Xuất bản toàn bộ dữ liệu quét
                    self.scan_dataRanges_filter = avg_ranges.tolist()  
                    self.scan_dataIntensities_filter = avg_intensities.tolist()  

                    # Xóa danh sách sau khi công bố
                    self.scan_dataRanges_list.clear()
                    self.scan_dataIntensities_list.clear()

                    self.process = 2

                    print(len(self.scan_dataRanges_filter))
                    print(len(self.scan_dataIntensities_filter))
                    print("Stop fiter scan data")
            
            elif self.process == 2:           ### Reflector extraction
                # -- lọc các điểm có cường độ cao 
                print("Step 2 - Reflector filter starting")
                ls_highReflectionIntensity = []
                for index, hri in enumerate(self.scan_dataIntensities_filter):
                    condition_1 = self.scan_dataRanges_filter[index] >= self.min_scanningRadius
                    condition_2 = hri >= self.min_reflectionIntensity and hri <= self.max_reflectionIntensity
                    
                    if condition_1 and condition_2:
                        # ls_highReflectionIntensity.append((index, self.scan_dataRanges_filter[index], hri))
                        self.reflector_dataIndex_list.append(index)
                        self.reflector_dataRanges_list.append(self.scan_dataRanges_filter[index])
                        self.reflector_dataIntensities_list.append(hri)

                print("Step 2 done - reflector filter done")

                self.process = 3
                # print(ls_highReflectionIntensity) # (index, dis, intensity)
                # print('---+---')                

            elif self.process == 3:   ### Draw graph
                # Tạo trục x
                # x = np.arange(len(self.scan_dataRanges_filter))  # Dữ liệu trục x (index)
                x = self.reflector_dataIndex_list
                # Tạo đồ thị
                fig, ax1 = plt.subplots(figsize=(12, 6))

                # Trục y đầu tiên (ranges)
                ax1.plot(x, self.reflector_dataRanges_list, label="Ranges", color="blue", linewidth=2)
                ax1.set_xlabel("Scanning Angle", fontsize=14)
                ax1.set_ylabel("Ranges", fontsize=14, color="blue")
                ax1.tick_params(axis='y', labelcolor="blue")
                ax1.grid(True, linestyle="--", alpha=0.7)

                # Trục y thứ hai (intensities)
                ax2 = ax1.twinx()
                ax2.plot(x, self.reflector_dataIntensities_list, label="Intensities", color="orange", linestyle="--", linewidth=2)
                ax2.set_ylabel("Intensities", fontsize=14, color="orange")
                ax2.tick_params(axis='y', labelcolor="orange")

                # Thêm tiêu đề
                plt.title("Filter reflector in 1000 times mean in 18cm", fontsize=16)

                # Hiển thị đồ thị
                fig.tight_layout()
                plt.show()

                self.process = 4

            elif self.process == 4:
                pass

            self.rate.sleep()

def main():
    print ("--- Run Detect Reflector ---")
    program = Graph_node()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




