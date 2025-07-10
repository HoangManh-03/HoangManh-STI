#!/usr/bin/env python3
"""
    - Author: Archie
    - Date: 2025-04-05
    - Description: Get info of reflector from laser scan data like x, y, theta with parameter:
        + protocol: tcp
        + filter_type: remission
        + filter_width: 16
        + fitler_error_matching: tolerance
        + remission_filter_threshold: reflector_low
    
    - Note:
        + Các gương sẽ được sắp xếp theo góc quét tăng dần của lidar : -180 -> 180, CCW
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
from geometry_msgs.msg import Point
from navigation_reflector.msg import *

class Reflector_dataPoint():
    def __init__(self):
        self.ranges = 0
        self.theta = 0

class Estimate_reflector_center():
    def __init__(self):
        rospy.init_node('evaluate_reflector_center_node', anonymous = True)
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
        self.Reflector_dataPoint = Reflector_dataPoint()
        self.reflector_data = []

        # -- 
        self.process = 0

        # Danh sách để lưu trữ dữ liệu quét

        self.reflector = []
        self.id_ref = 0

        self.RANGE_MAX = 30.0
        self.RANGE_MIN = 0.35   # from lidar to reflector center
        self.ANGLE_RESOLUTION = 0.05*PI/180.0     #degree

        self.POINT_REF = 7200              # tương ứng vơí resolution: 0.05
        self.CONVERT_RATE_ACTUAL_ANGLE = self.POINT_REF/360

        # khoang cach toi da cua guong de so luong diem guong tra ve > 1
        self.LIMIT_RANGE = (self.radius_reflector - self.radius_reflector*sin(self.ANGLE_RESOLUTION))/sin(self.ANGLE_RESOLUTION) # 6.84 m
        self.is_cluster = 1

        self.ANGLE_TOL = 6.0    # góc tối thiểu giữa 2 gương

        # self.reflector_unit = []
        self.ANGLE_TRANSLATE = 180

        self.ANGLE_OFFSET_POINT_REF = 10

        # --
        self.RANGE_THRESHOLD = 1000 

        self.ls_poseReflector = []

        self.r_thres = 0
        self.a_thres = 0
        self.step = 0
        self.DISTANCE_TOLERANCE = 0.03
        self.ANGLE_TOLERANCE = 0
        self.ls_ranges = []
        self.ls_theta = []
        self.count_r = 0
        
        self.stop_flag = 0     # for debug: stop process when get data from laser scan

        # rospy.Subscriber("/scan_filter", LaserScan, self.callback_scan)
        rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        self.dataScan = LaserScan()
        self.is_scan = False

        # -------- Topic Pub -------- #
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        
        self.pub_infoReflector = rospy.Publisher('/r2000_reflectors', R2000_reflectors, queue_size= 10)
        self.msg_raw_reflector = R2000_reflectors()

    def sort_list_with_indices(self, lst):
        # Bước 1: Tạo danh sách (giá trị, chỉ số ban đầu)
        indexed_list = list(enumerate(lst))

        # Bước 2: Sắp xếp danh sách theo giá trị
        sorted_list = sorted(indexed_list, key=lambda x: x[1])

        # Bước 3: Tạo danh sách kết quả
        sorted_values = [val for idx, val in sorted_list]  # Danh sách đã sắp xếp
        sorted_indices = [idx for idx, val in sorted_list]  # Chỉ số gốc của từng phần tử

        return sorted_values, sorted_indices

    def callback_scan(self, data):
        if self.stop_flag == 0:
            self.dataScan = data
            self.is_scan = True

            self.ANGLE_TOLERANCE = data.angle_increment*5
            self.reflector_data = []

            self.msg_raw_reflector.header.frame_id = "scanner_link"
            self.msg_raw_reflector.header.stamp = rospy.Time.now()

            
            for i, r in enumerate(data.ranges):
                Reflector_info = Reflector_data()
                ref = Reflector_dataPoint()
                if r < self.RANGE_THRESHOLD:
                    ref.ranges = r  
                    ref.theta = data.angle_min + i * data.angle_increment

                    self.reflector_data.append(ref)

                    if self.step == 0:
                        # -- update nguong
                        self.r_thres = r
                        self.a_thres = i * data.angle_increment
                        self.step = 1

                        self.ls_ranges.append(r)
                        self.ls_theta.append(data.angle_min + i * data.angle_increment)
                        self.count_r += 1
                        # print(f"Ngưỡng lần {self.count_r} là: {self.r_thres}")

                    else:
                        delta_r = r - self.r_thres

                        delta_a = i * data.angle_increment - self.a_thres

                        # print(f"Giá trị delta_r: {delta_r} / delta_a: {delta_a}")

                        if abs(delta_r) <= self.DISTANCE_TOLERANCE and abs(delta_a) <= self.ANGLE_TOLERANCE:
                            self.ls_ranges.append(r)
                            self.ls_theta.append(data.angle_min + i * data.angle_increment)
                        
                        else:
                            # self.step = 0
                            mean_r = np.mean(self.ls_ranges)
                            mean_theta = np.mean(self.ls_theta)
                            # print(f"mean_r: {mean_r} / mean_theta: {mean_theta}")

                            # # - update id
                            self.id_ref += 1
                            # print(f"Gương cụm {self.id_ref} có thông tin là: {self.ls_ranges} / {self.ls_theta}")
                            # # -
                        
                            x_final = mean_r*cos(mean_theta)
                            y_final = mean_r*sin(mean_theta)

                            # - for rviz display
                            # self.ls_poseReflector.append([x_final, y_final, 0.0])

                            # for ros
                            Reflector_info.Cart_X = x_final
                            Reflector_info.Cart_Y = y_final
                            Reflector_info.Polar_Dist = mean_r
                            Reflector_info.Polar_Phi = mean_theta
                            Reflector_info.LocalID = self.id_ref
                            Reflector_info.Size = 0
                            Reflector_info.Index_Begin = 0
                            Reflector_info.Index_End = 0

                            self.msg_raw_reflector.reflectors.append(Reflector_info)

                            self.ls_ranges = []
                            self.ls_theta = []

                            # update new ref
                            self.r_thres = r
                            self.a_thres = i * data.angle_increment
                            self.ls_ranges.append(r)
                            self.ls_theta.append(data.angle_min + i * data.angle_increment)
                            self.count_r += 1
                            # print(f"Ngưỡng lần {self.count_r} là: {self.r_thres}")
            
            # -- update last reflector
            mean_r = np.mean(self.ls_ranges)
            mean_theta = np.mean(self.ls_theta)
            # print(f"mean_r: {mean_r} / mean_theta: {mean_theta}")

            # # - update id
            self.id_ref += 1
            # print(f"Gương cụm {self.id_ref} có thông tin là: {self.ls_ranges} / {self.ls_theta}")
            # # -
        
            x_final = mean_r*cos(mean_theta)
            y_final = mean_r*sin(mean_theta)

            # - for rviz display
            # self.ls_poseReflector.append([x_final, y_final, 0.0])

            # for ros
            Reflector_info.Cart_X = x_final
            Reflector_info.Cart_Y = y_final
            Reflector_info.Polar_Dist = mean_r
            Reflector_info.Polar_Phi = mean_theta
            Reflector_info.LocalID = self.id_ref
            Reflector_info.Size = 0
            Reflector_info.Index_Begin = 0
            Reflector_info.Index_End = 0

            self.msg_raw_reflector.reflectors.append(Reflector_info)

            # -- arrange data following the order of distance
            ls_d = []
            for ref in self.msg_raw_reflector.reflectors:
                ls_d.append(ref.Polar_Dist)
            
            sorted_values, sorted_indices = self.sort_list_with_indices(ls_d)

            lsReflector_arrange = []
            for j, indices in enumerate(sorted_indices):
                Reflector_info = Reflector_data()
                for i, ref in enumerate(self.msg_raw_reflector.reflectors):
                    if i == indices:
                        Reflector_info.LocalID = j+1
                        Reflector_info.Cart_X = ref.Cart_X
                        Reflector_info.Cart_Y = ref.Cart_Y
                        Reflector_info.Polar_Dist = ref.Polar_Dist
                        Reflector_info.Polar_Phi = ref.Polar_Phi
                        Reflector_info.Size = 0
                        Reflector_info.Index_Begin = 0
                        Reflector_info.Index_End = 0
                        
                        lsReflector_arrange.append(Reflector_info)
                        self.ls_poseReflector.append([ref.Cart_X, ref.Cart_Y, 0.0])

            # - update reflector info
            self.msg_raw_reflector.reflectors = lsReflector_arrange

            # -- screen data
            # for i, ref in enumerate(self.reflector_data):
            #     print(f"reflector {i+1}: {ref.ranges:.2f} / {degrees(ref.theta):.2f}")

            for ref in self.msg_raw_reflector.reflectors:
                print(f"Thông tin các gương: {ref.LocalID} / {ref.Cart_X:.2f} / {ref.Cart_Y:.2f} / {ref.Polar_Dist:.2f} / {degrees(ref.Polar_Phi):.2f}")

            # for ref in self.msg_raw_reflector.reflectors:
            #     print(f"Thông tin các gương: {ref.LocalID} / {ref.Cart_X:.2f} / {ref.Cart_Y:.2f} / {ref.Polar_Dist:.2f} / {ref.Polar_Phi:.2f}")

            self.msg_raw_reflector.num_reflector = self.id_ref
            print(f"Số lượng gương detect là: {self.id_ref}")
            
            # -- publish
            self.pub_marker(self.ls_poseReflector)
            self.pub_infoReflector.publish(self.msg_raw_reflector)

            # -- reset var
            self.ls_poseReflector = []
            self.msg_raw_reflector = R2000_reflectors()
            self.id_ref = 0
            self.count_r = 0
            self.stop_flag = 0
            self.ls_ranges = []
            self.ls_theta = []

            self.step = 0

            print("-------------------------- \n")
    
    def pub_marker(self, list_point):
        marker_a = Marker()
        for center in list_point:
            p = Point()
            p.x = center[0]
            p.y = center[1]
            p.z = center[2]
            marker_a.points.append(p)

        marker_array = []  # Lưu danh sách các marker

        for i, p in enumerate(marker_a.points):
            # Marker hình cầu
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "scanner_link"
            sphere_marker.header.stamp = rospy.Time.now()
            sphere_marker.ns = "spheres"
            sphere_marker.id = i  # ID duy nhất
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = p.x
            sphere_marker.pose.position.y = p.y
            sphere_marker.pose.position.z = p.z
            sphere_marker.scale.x = 0.2
            sphere_marker.scale.y = 0.2
            sphere_marker.scale.z = 0.2
            sphere_marker.color.a = 1.0  # Độ trong suốt
            sphere_marker.color.r = 0.0
            sphere_marker.color.g = 1.0
            sphere_marker.color.b = 0.0
            # Pose
            sphere_marker.pose.orientation.x = 0.0
            sphere_marker.pose.orientation.y = 0.0
            sphere_marker.pose.orientation.z = 0.0
            sphere_marker.pose.orientation.w = 1.0

            marker_array.append(sphere_marker)

            # Marker hiển thị ID
            text_marker = Marker()
            text_marker.header.frame_id = "scanner_link"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "text"
            text_marker.id = i + 100  # Đảm bảo ID không trùng với hình cầu
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = p.x
            text_marker.pose.position.y = p.y
            text_marker.pose.position.z = p.z + 0.6  # Hiển thị phía trên hình cầu
            text_marker.scale.z = 3  # Kích thước chữ
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = str(i+1)  # Hiển thị ID

            # Pose
            text_marker.pose.orientation.x = 0.0
            text_marker.pose.orientation.y = 0.0
            text_marker.pose.orientation.z = 0.0
            text_marker.pose.orientation.w = 1.0

            marker_array.append(text_marker)

        # Xuất tất cả marker
        for marker in marker_array:
            self.marker_pub.publish(marker)   

    def run(self):
        while not rospy.is_shutdown():
            pass

            self.rate.sleep()

def main():
    print ("--- Run estimate_reflector_center ---")
    program = Estimate_reflector_center()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




