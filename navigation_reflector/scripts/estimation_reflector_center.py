#!/usr/bin/env python3
"""
*** infomation
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

Khoảng cách lidar - gương tối thiểu là 350 mm
khoảng cách giữa các gương là 300 mm
Góc giữa các gương > 6 độ

*** Task Description: 
    + Lọc điểm gương
    + phân cụm gương
    + Trả về vị trí tâm các gương ( có hiển thị trên rviz qua visualizeMarker)

*** need to do
   + Trường hợp: gương ở góc chuyển tiếp từ 1420 -> 20 thì sao ??? ko sao vì theta_f có 1 giá trị max thôi mà
   + Mình đã tăng góc theta quá ngưỡng tối đã để đảm bảo bắt hết được điểm gương, vì có trường hợp chưa chắc tia nhỏ nhất đã nằm tài điểm giữa của gương
   + Cần cấu hình gương ở angle resolution nhỏ hơn, để tìm được gương ở khoảng cách xa hơn nữa. (đang 0.25)
   + Sử dụng công thức toán cho hàm polyfitting - dòng 445

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
from navigation_reflector.msg import Raw_reflector
from openpyxl import Workbook
import matplotlib.pyplot as plt

class Reflector_dataPoint():
    def __init__(self):
        self.ranges = []
        self.theta = []
        self.intensities = []

class Estimate_reflector_center():
    def __init__(self):
        rospy.init_node('estimate_reflector_center_node', anonymous = True)
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

        # rospy.Subscriber("/scan_filter", LaserScan, self.callback_scan)
        rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        self.dataScan = LaserScan()
        self.is_scan = False

        rospy.Subscriber("/Keyboard_cmd", String, self.callback_KeyboardCmd)
        self.keyboad_cmd = String()

        # -------- Topic Pub -------- #
        self.pub_infoReflector = rospy.Publisher('/info_raw_reflector', Raw_reflector, queue_size= 10) 
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        # Đăng ký publisher cho topic dữ liệu trung bình
        self.ref_pub = rospy.Publisher('/reflector_filter', LaserScan, queue_size=10)

        # -- 
        self.process = 0

        # Danh sách để lưu trữ dữ liệu quét
        self.reflector_data = Reflector_dataPoint()

        self.reflector = []
        self.id_ref = 0

        self.reflector_dataIntensities_gauss_list = []
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

    def callback_scan(self, data):
        self.dataScan = data
        self.is_scan = True

    def callback_KeyboardCmd(self, data):
        self.keyboad_cmd = data

        if self.keyboad_cmd.data == '2':     #reset
            self.is_cluster = 1
            self.process = 0
            self.reflector = []
            print("############################################################################")

    def find_gaussCoef(self, inten_list):

        index_list = []
        for index in range(0, len(inten_list)):
            index_list.append(index)

        u = np.array(index_list)

        # Dữ liệu mẫu
        sigma_r = np.array(inten_list)

        # Lấy log(σ_r)
        log_sigma_r = np.log(sigma_r)

        # Fit đa thức bậc 2: log(σ_r) = p1*u^2 + p2*u + p3
        degree = 2
        coefficients = np.polyfit(u, log_sigma_r, degree)  # Hồi quy đa thức
        polynomial = np.poly1d(coefficients)  # Tạo hàm đa thức

        # Tạo dữ liệu dự đoán
        u_fit = np.linspace(min(u), max(u), 100)  # Các điểm u cho fitting
        log_sigma_r_fit = polynomial(u_fit)  # Giá trị log(σ_r) từ fitting
        sigma_r_fit = np.exp(log_sigma_r_fit)  # Chuyển ngược từ log về σ_r

        #Vẽ dữ liệu gốc và đường fitting
        # plt.figure(figsize=(10, 6))
        # plt.plot(u, sigma_r, color='red', label='Dữ liệu gốc', zorder=3)
        # plt.plot(u_fit, sigma_r_fit, color='blue', label='Fitting: $\\log(\\sigma_r)$', linewidth=2)
        # plt.title('Polynomial Fitting for $\\log(\\sigma_r)$', fontsize=16)
        # plt.xlabel('u', fontsize=14)
        # plt.ylabel('$\\sigma_r$', fontsize=14)
        # plt.legend(fontsize=12)
        # plt.grid(True, linestyle='--', alpha=0.7)
        # plt.show()

        # Hiển thị các hệ số đa thức
        # print("Hệ số đa thức (p1, p2, p3):", coefficients)

        return coefficients

    # def pub_marker(self, list_point):
    #     # Tạo Marker kiểu SPHERE_LIST để hiển thị danh sách các điểm tâm
    #     marker = Marker()

    #     # Đặt các thuộc tính cơ bản cho Marker
    #     marker.header.frame_id = 'scanner_link'  # Frame tham chiếu, ví dụ: "world" hoặc "map"
    #     marker.header.stamp = rospy.Time.now()

    #     marker.ns = "circle_centers"
    #     marker.id = 0  # ID của marker
    #     marker.type = Marker.SPHERE_LIST  # Sử dụng SPHERE_LIST để hiển thị nhiều điểm
    #     marker.action = Marker.ADD  # Thao tác: thêm vào hiển thị

    #     # Kích thước của các hình cầu (tất cả các điểm tâm sẽ có cùng kích thước)
    #     marker.scale.x = 0.2  # Bán kính SPHERE trên trục x
    #     marker.scale.y = 0.2  # Bán kính SPHERE trên trục y
    #     marker.scale.z = 0.2  # Bán kính SPHERE trên trục z

    #     # Màu sắc của các hình cầu (RGBA)
    #     marker.color.r = 0.0  # Màu đỏ
    #     marker.color.g = 1.0  # Màu xanh lá
    #     marker.color.b = 0.0  # Màu xanh dương
    #     marker.color.a = 1.0  # Độ đậm của màu (1.0 là không trong suốt)

    #     # Pose
    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z = 0.0
    #     marker.pose.orientation.w = 1.0
    #     # Thêm danh sách các điểm vào Marker
    #     for center in list_point:
    #         p = Point()
    #         p.x = center[0]
    #         p.y = center[1]
    #         p.z = center[2]
    #         marker.points.append(p)

    #     self.marker_pub.publish(marker)

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
            text_marker.scale.z = 0.3  # Kích thước chữ
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
            # print(f"Đang chạy ở process {self.process}")
            # print("#---")
            if self.process == 0:
                if self.is_scan:
                    self.process = 1

            elif self.process == 1:            # - Step 1: Separate scan data point as reflector or not
                if self.is_scan == False:
                    self.rate.sleep()
                    continue
                
                self.is_scan = False
            
                # -- lọc các điểm có cường độ cao 
                # print("Step 1 - Reflector filter starting")
                self.reflector_data = Reflector_dataPoint()
                self.reflector = []

                data_scan = copy.copy(self.dataScan)

                # -- 
                dis_arr = list(data_scan.ranges)

                for theta, hri in enumerate(self.dataScan.intensities):
                    condition_1 = self.dataScan.ranges[theta] >= self.min_scanningRadius
                    condition_2 = hri >= self.min_reflectionIntensity and hri <= self.max_reflectionIntensity
                    
                    if condition_1 and condition_2:
                        self.reflector_data.ranges.append(self.dataScan.ranges[theta])
                        self.reflector_data.theta.append(theta)
                        self.reflector_data.intensities.append(hri)

                    else:
                        dis_arr[theta] = 9.

                data_scan.ranges = tuple(dis_arr)
                self.ref_pub.publish(data_scan)

                # print("Step 1 done - reflector filter done")

                if len(self.reflector_data.theta) > 0:
                    self.process = 2
                    print("Số lượng data set của gương là:" , len(self.reflector_data.ranges)) # (index, dis, intensity)
                    # print(f"Vị trí các điểm gương là: {self.reflector_data.theta}")
                    # print(f"Vị trí các điểm gương là: {self.reflector_data.theta} với khoảng cách tương ứng là: {self.reflector_data.ranges}")
                    print("---------------------------------")
                else:
                    print("Không tìm thấy gương trên map")
                    self.rate.sleep()
                    time.sleep(3)
                    # self.process = 4
            
            elif self.process == 2:   ### Step 2: Phân cụm gương, gán ID and data set for reflector
                ## Nếu gương nằm tại điểm giáp giữa điểm quay lại 360 thì xử lý sao?? 
                self.is_cluster = 0
                if len(self.reflector_data.theta) == 0:
                    print("Đã hết dữ liệu => next step")
                    print("Số lượng gương detect được là ", len(self.reflector))
                    print("---------------------------------")

                    # -- move to next step
                    self.process = 3
                    self.id_ref = 0
                    continue

                # -- Start with minimum distance
                d_k = min(self.reflector_data.ranges)
                if d_k < self.RANGE_MIN:         # clear hết dữ liệu gương mà có d < 0.35 m
                    # -- find index of d_k in list
                    I_k = self.reflector_data.ranges.index(d_k)
                    # print("Xoá vị trí gương ", self.reflector_data.theta[I_k])

                    del self.reflector_data.theta[I_k]
                    del self.reflector_data.ranges[I_k]
                    del self.reflector_data.intensities[I_k]

                else:
                    # -- Calculate delta max angle between 2 adject reflector data point
                    delta_inradian_max = asin(self.radius_reflector/(self.radius_reflector + d_k)) + self.ANGLE_OFFSET_POINT_REF*self.ANGLE_RESOLUTION # mo rong goc quet len 0.5 độ để đảm bảo lấy hết được reflector point
                    delta_indegree_max = degrees(delta_inradian_max)

                    # -- Calculate distance max angle between 2 adject reflector data point
                    D_k = tan(delta_inradian_max/2)*d_k*sin(delta_inradian_max)

                    # -- find index of d_k in list
                    I_k = self.reflector_data.ranges.index(d_k)

                    # print("Góc ngưỡng là: ", 4*delta_indegree_max)
                    # print("Khoảng cách ngưỡng là: ", D_k)
                    # print("chỉ số điểm ngưỡng là", I_k)
                    # print("---------------------------------")
                    
                    # -- cluster reflector data
                    self.id_ref = self.id_ref + 1
                    n = 0
                    i = 1
                    j = 1
                    cluster = []
                    cluster.append([self.id_ref, self.reflector_data.ranges[I_k], self.reflector_data.theta[I_k], self.reflector_data.intensities[I_k]])
                    # print("Check điểm gương tại vị trí", self.reflector_data.theta[I_k])
                    while n < len(self.reflector_data.theta) - 1:
                        if (I_k - j) >= 0:
                            # print("Check điểm gương tại vị trí", self.reflector_data.theta[I_k-j])
                            delta_theta = abs(self.reflector_data.theta[I_k-j]/self.CONVERT_RATE_ACTUAL_ANGLE - self.reflector_data.theta[I_k]/self.CONVERT_RATE_ACTUAL_ANGLE)
                            delta_theta = radians(delta_theta)
                            delta_d = tan(delta_theta/2)*d_k*sin(delta_theta)
                            # print("Delta_d", delta_d)
                            if (delta_d < D_k):
                                cluster.append([self.id_ref, self.reflector_data.ranges[I_k-j], self.reflector_data.theta[I_k-j], self.reflector_data.intensities[I_k - j]])
                            
                            j = j+1

                        else:      
                            # print("Check điẻm gương tại vị trí", self.reflector_data.theta[I_k+i])    
                            delta_theta = abs(self.reflector_data.theta[I_k+i]/self.CONVERT_RATE_ACTUAL_ANGLE - self.reflector_data.theta[I_k]/self.CONVERT_RATE_ACTUAL_ANGLE)
                            delta_theta = radians(delta_theta)
                            delta_d = tan(delta_theta/2)*d_k*sin(delta_theta)
                            # print("Delta_d", delta_d)

                            if (delta_d < D_k):
                                cluster.append([self.id_ref, self.reflector_data.ranges[I_k+i], self.reflector_data.theta[I_k+i], self.reflector_data.intensities[I_k + i]])
                            
                            i = i + 1

                        n = n + 1
                    
                    ## - Sắp xếp lại các điểm gương từ góc bé đến lớn
                    cluster.sort(key=lambda x: x[2])                                  

                    ## - Xóa phần tử đã phân loại trong data gốc
                    for item_cluster in cluster:
                        for item_theta in self.reflector_data.theta:
                            if item_cluster[2] == item_theta:
                                index = self.reflector_data.theta.index(item_theta)

                                self.reflector_data.theta.remove(item_theta)
                                del self.reflector_data.ranges[index]
                                del self.reflector_data.intensities[index]

                    # print("---------------------------------")
                    # print("Chiều dài List còn lại sau khi xóa phần tử đã phân loại là: ", len(self.reflector_data.theta))

                    ## -- Xoá các điểm gương có cùng hướng vơí điểm gương đã phân loại
                    theta_lower = cluster[0][2]
                    theta_upper = cluster[-1][2]

                    # print("Vị trí góc cận dưới là", theta_lower)
                    # print("Vị trí góc cận trên là", theta_upper)
                    delete_reflector = []
                    # - find index need to delete
                    n = len(self.reflector_data.theta)
                    for index in range(0, n):
                        if self.reflector_data.theta[index] > theta_upper:
                            delta_theta = self.reflector_data.theta[index] - theta_upper

                            if delta_theta < self.CONVERT_RATE_ACTUAL_ANGLE*self.ANGLE_TOL:
                                # print("--------------------")
                                # print("Xóa điẻm gương tại", self.reflector_data.theta[index])
                                delete_reflector.append(index)

                        elif self.reflector_data.theta[index] < theta_lower:
                            delta_theta = theta_lower - self.reflector_data.theta[index]

                            if delta_theta < self.CONVERT_RATE_ACTUAL_ANGLE*self.ANGLE_TOL:
                                # print("--------------------")
                                # print("Xóa điẻm gương tại", self.reflector_data.theta[index])
                                delete_reflector.append(index)

                    # print("Vị trí các điểm gương cần xóa là: ", delete_reflector)                                                              
                    # - clear reflector point has index in above list
                    indices_to_remove_set = set(delete_reflector)

                    # Duyệt qua danh sách và giữ lại các phần tử không nằm trong indices_to_remove
                    filtered_list = [element for index, element in enumerate(self.reflector_data.theta) if index not in indices_to_remove_set]
                    self.reflector_data.theta = filtered_list

                    filtered_list = [element for index, element in enumerate(self.reflector_data.ranges) if index not in indices_to_remove_set]
                    self.reflector_data.ranges = filtered_list

                    filtered_list = [element for index, element in enumerate(self.reflector_data.intensities) if index not in indices_to_remove_set]
                    self.reflector_data.intensities = filtered_list

                    # print("mảng output là: ")
                    # print(cluster)
                    # print("---------------------------------")
                    # print("chiều dài mảng output là: ")
                    # print(len(cluster))
                    # print("---------------------------------")
                    # print("Chiều dài List còn lại là: ")
                    # print(len(self.reflector_data.theta))

                    ## - Add to final reflector list
                    # if len(cluster) >= 1:                               # ít nhất ở khoảng cách này có 3 điểm gương
                    self.reflector.append(cluster)

                    print("Độ dài của các điểm gương số {x} là: {y}".format(x = self.id_ref, y = len(cluster)))
                    print("#########################################")
                
                # else:
                #     # -- find index of d_k in list
                #     self.id_ref = self.id_ref + 1
                #     I_k = self.reflector_data.ranges.index(d_k)
                #     theta_k = self.reflector_data.theta[I_k]
                #     cluster.append([self.id_ref, self.reflector_data.ranges[I_k], self.reflector_data.theta[I_k], self.reflector_data.intensities[I_k]])

                #     #-- delete reflector in the same direction 
                #     n = len(self.reflector_data.theta)
                #     delete_reflector = []
                #     for index in range(0, n):
                #         delta_theta = self.reflector_data.theta[index] - theta_k

                #         if (delta_theta < self.ANGLE_TOL*4 and delta_theta > -4*self.ANGLE_TOL):
                #             # print("--------------------")
                #             # print("Xóa điẻm gương tại", self.reflector_data.theta[index])
                #             delete_reflector.append(index)

                #     # - clear index in refletor raw data
                #     for index in range(0, len(delete_reflector)):
                #         self.reflector_data.theta.pop(delete_reflector[index])
                #         self.reflector_data.ranges.pop(delete_reflector[index])
                #         self.reflector_data.intensities.pop(delete_reflector[index])

                #     cluster.clear()

            elif self.process == 3:   ### figure center of reflector pos
                n = len(self.reflector)
                
                # self.reflector_unit = []
                ls_poseReflector = []

                msg_raw_reflector = Raw_reflector()
                msg_raw_reflector.header.frame_id = 'raw_reflector'
                msg_raw_reflector.header.stamp = rospy.Time.now()

                for indexx in range(0, n):

                    # - Create list again after clustering reflector
                    data = self.reflector[indexx]
                    self.reflector_data = Reflector_dataPoint()
                    pose_reflector = Point()

                    for indexxx in range(0, len(data)):
                        self.reflector_data.theta.append(data[indexxx][2])
                        self.reflector_data.ranges.append(data[indexxx][1])
                        self.reflector_data.intensities.append(data[indexxx][3])

                    len_ref = len(data)

                    if len_ref == 1:           # in case reflector cluster has 1 point
                        # - 
                        theta_f = self.reflector_data.theta[0]
                        theta_rad = radians(theta_f/self.CONVERT_RATE_ACTUAL_ANGLE - self.ANGLE_TRANSLATE)
                        d_f = self.reflector_data.ranges[0] + self.radius_reflector
                    
                        x_final = d_f*cos(theta_rad)
                        y_final = d_f*sin(theta_rad)

                        # x_cv = x_final * cos(self.ANGLE_TRANSLATE) + y_final * sin(self.ANGLE_TRANSLATE)
                        # y_cv = -x_final * sin(self.ANGLE_TRANSLATE) + y_final * cos(self.ANGLE_TRANSLATE)

                        # -
                        pose_reflector.x = x_final
                        pose_reflector.y = y_final

                        # print("Khoảnh cách tới tâm gương thứ {x} là: {y}, góc là: {z}".format(x = data[indexxx][0], y = d_f, z = theta_f))
                        # print("Vị trí x của gương thứ {x} là: {y}".format(x = data[indexxx][0], y = x_cv))
                        # print("Vị trí y của gương thứ {x} là: {y}".format(x = data[indexxx][0], y = y_cv))
                        # print("---------------------------------")

                        ls_poseReflector.append([x_final, y_final, 0.0])
                        msg_raw_reflector.points.append(pose_reflector)
                        # self.reflector_unit.append([d_f, theta_f])

                    elif len_ref == 2:         # in case reflector cluster has 2 point
                        d0 = self.reflector_data.ranges[0]
                        d1 = self.reflector_data.ranges[1]

                        if d0 <= d1: 
                            theta_f = self.reflector_data.theta[0]
                            theta_rad = radians(theta_f/self.CONVERT_RATE_ACTUAL_ANGLE - self.ANGLE_TRANSLATE)
                            d_f = self.reflector_data.ranges[0] + self.radius_reflector
                        else:
                            theta_f = self.reflector_data.theta[1]
                            theta_rad = radians(theta_f/self.CONVERT_RATE_ACTUAL_ANGLE - self.ANGLE_TRANSLATE)
                            d_f = self.reflector_data.ranges[1] + self.radius_reflector

                        x_final = d_f*cos(theta_rad)
                        y_final = d_f*sin(theta_rad)

                        # x_cv = x_final * cos(self.ANGLE_TRANSLATE) + y_final * sin(self.ANGLE_TRANSLATE)
                        # y_cv = -x_final * sin(self.ANGLE_TRANSLATE) + y_final * cos(self.ANGLE_TRANSLATE)

                        # -
                        pose_reflector.x = x_final
                        pose_reflector.y = y_final

                        # print("Khoảnh cách tới tâm gương thứ {x} là: {y}, góc là: {z}".format(x = data[indexxx][0], y = d_f, z = theta_f))
                        # print("Vị trí x của gương thứ {x} là: {y}".format(x = data[indexxx][0], y = x_cv))
                        # print("Vị trí y của gương thứ {x} là: {y}".format(x = data[indexxx][0], y = y_cv))
                        # print("---------------------------------")

                        ls_poseReflector.append([x_final, y_final, 0.0])
                        msg_raw_reflector.points.append(pose_reflector)
                        # self.reflector_unit.append([d_f, theta_f])

                    else:
                        # - find p1, p2, p3 for polynominal fitting 
                        p1 = p2 = p3 = 0
                        try:
                            p1, p2, p3 = self.find_gaussCoef(self.reflector_data.intensities)
                            # print("hệ số p1, p2, p3 là:", p1, p2, p3)
                            # print("---------------------------------")

                        except ValueError as e:
                            print(f"Lỗi: {e}")
                            print("Không tìm được hệ số polynominal cho gương cụm gương này => loaị bỏ gương")
                        
                        if p1 == 0 and p2 == 0 and p3 == 0:
                            self.rate.sleep()
                            continue

                            
                        else:
                            a = b = c = 0
                            try:
                                # - find coefficients of the Gaussian fitting model 
                                a = exp(p3 - p2*p2/4/p1)                 # biên độ
                                b = -p2/p1                               # giá trị trung bình
                                c = sqrt(-1/p1)                          # độ lệch chuẩn

                            except ValueError as e:
                                print(f"Lỗi: {e}")
                                # self.rate.sleep()
                                # continue
                            
                            if a == 0 and b == 0 and c == 0:
                                print("Không tìm được hệ số a, b, c cho gương cụm gương này => loaị bỏ gương")
                                self.rate.sleep()
                                continue
                            else:
                                                    
                                # print("hệ số a, b, c là:", a, b, c)
                                # print("---------------------------------")
                                
                                # - tạo gauss list
                                self.reflector_dataIntensities_gauss_list = []
                                for index in range(0, 4*len(self.reflector_data.intensities)):
                                    xichma_g = a*exp(-(index-b)*(index-b)/c)
                                    self.reflector_dataIntensities_gauss_list.append(xichma_g)

                                # print("Giá trị của list gaussian fitting là:", self.reflector_dataIntensities_gauss_list)
                                # print("---------------------------------------")

                                # Đưa ra giá trị max của list gaussian
                                xichma_f = max(self.reflector_dataIntensities_gauss_list)
                                index_f = self.reflector_dataIntensities_gauss_list.index(xichma_f)

                                # Đưa ra góc ngưỡng
                                theta_lamda = round((self.reflector_data.theta[-1] - self.reflector_data.theta[0])*index_f/4/len(self.reflector_data.theta))

                                theta_f = theta_lamda + self.reflector_data.theta[0]

                                # print(self.reflector_dataIndex_list)
                                # print("Chỉ số của giá trị max trong list gaussian là: ", index_f, theta_f)
                                # print("---------------------------------")
                                
                                # print("Góc của cụm gương là:", self.reflector_data.theta)
                                # Đưa ra thông tin của điểm gương có cường độ phân phối gauss max
                                # toggle_case = 1
                                # count_err = 0
                                # I_m = 0
                                # condition1 = 0
                                # condition2 = 0

                                # while(I_m == 0):     # hướng xử lý khi không tìm thấy theta_f trong list theta của gương  => Đưa ra vị trí gần nó nhất
                                #     print(f"Vị trí góc set là {theta_f}, tại lần lỗi thứ {count_err}")
                                #     for index in range(0, len(self.reflector_data.theta)):
                                #         if(theta_f == self.reflector_data.theta[index]):
                                #             I_m = index
                                #             break
                                    
                                #     if(I_m == 0):
                                #         count_err = count_err + 1
                                #         # print("Tăng lỗi lần:", count_err)

                                #         if (toggle_case):
                                #             theta_f = theta_f + count_err
                                #             toggle_case = 0

                                #             if theta_f > self.reflector_data.theta[-1]:
                                #                 condition1 = 1

                                #         else:
                                #             theta_f = theta_f - count_err - 1
                                #             toggle_case = 1

                                #             if theta_f < self.reflector_data.theta[0]:
                                #                 condition2 = 1
                                            
                                #         if condition1 == 1 and condition2 == 1:
                                #             print("Ko có góc theta_f nào thoả mãn")
                                #             break

                                I_m = 0
                                # print(f"Vị trí góc set là {theta_f}, tại lần lỗi thứ {count_err}")
                                for index in range(0, len(self.reflector_data.theta)):
                                    if(theta_f == self.reflector_data.theta[index]):
                                        I_m = index
                                        break

                                if I_m == 0:
                                    # self.rate.sleep()
                                    print("Ko tìm được vị trí điểm gương thoả mãn")
                                    continue

                                else:    
                                    try:
                                        d_m = self.reflector_data.ranges[I_m]
                                        theta_m = theta_f
                                        xichma_m = self.reflector_data.intensities[I_m]

                                        delta = min([len(self.reflector_data.theta) - I_m, I_m - 1])
                                        print(f"Vi tri ok la: {I_m}, Giá trị ngưỡng là: {delta}")
                                        print("---------------------------------")

                                        index_lower_bound = I_m - delta
                                        index_upper_bound = I_m + delta

                                        # - Khi chỉ số của point ở trên cùng vượt qua cả chiều dài cuả list
                                        if index_upper_bound >= len(self.reflector_data.ranges):
                                            index_upper_bound = len(self.reflector_data.ranges) - 1
                                        
                                        # - Số lượng phân tử của list
                                        numberOflist = index_upper_bound - index_lower_bound + 1

                                        print(f"Giá trị giới hạn dưới là :{index_lower_bound}, giơí hạn trên là: {index_upper_bound}")
                                        print(f"Chiều dài của list là: {len(self.reflector_data.ranges)}")
                                        print("---------------------------------")
                                        # find mean 
                                        sum = 0
                                        for index in range(index_lower_bound, index_upper_bound + 1):
                                            sum = sum + self.reflector_data.ranges[index]
                                            # print("Tổng dữ liệu lần thứ {x} là: {y}".format(x = index, y = sum))

                                        d_f = sum/numberOflist
                                        
                                        # - 
                                        theta_rad = radians(theta_f/self.CONVERT_RATE_ACTUAL_ANGLE - self.ANGLE_TRANSLATE)
                                    
                                        x_final = d_f*cos(theta_rad)
                                        y_final = d_f*sin(theta_rad)

                                        # x_cv = x_final * cos(self.ANGLE_TRANSLATE) + y_final * sin(self.ANGLE_TRANSLATE)
                                        # y_cv = -x_final * sin(self.ANGLE_TRANSLATE) + y_final * cos(self.ANGLE_TRANSLATE)

                                        # --
                                        # pose_reflector.x = x_cv
                                        # pose_reflector.y = y_cv
                                        pose_reflector.x = x_final
                                        pose_reflector.y = y_final
                                        pose_reflector.z = theta_rad

                                        print("Khoảnh cách tới tâm gương thứ {x} là: {y}, góc là: {z}".format(x = data[indexxx][0], y = d_f, z = theta_f))
                                        print("Vị trí x của gương thứ {x} là: {y}".format(x = data[indexxx][0], y = x_final))
                                        print("Vị trí y của gương thứ {x} là: {y}".format(x = data[indexxx][0], y = y_final))
                                        print("---------------------------------")

                                        ls_poseReflector.append([x_final, y_final, 0.0])
                                        msg_raw_reflector.points.append(pose_reflector)
                                        # self.reflector_unit.append([d_f, theta_f]) 

                                    except ValueError as e:
                                        print(f"Lỗi: {e}")
                                        print(f"Giá trị cận trên là: {index_lower_bound}, giá trị cận trên là: {index_upper_bound}")
                                        print("---------------------------------")

                self.pub_marker(ls_poseReflector)
                self.pub_infoReflector.publish(msg_raw_reflector)
                self.process = 0
            
            elif self.process == 15:   ### Draw graph
                # Tạo trục x
                # x = np.arange(len(self.scan_dataRanges_filter))  # Dữ liệu trục x (index)
                x = self.reflector_data.theta

                # Tạo đồ thị
                fig, ax1 = plt.subplots(figsize=(12, 6))

                # Trục y đầu tiên (ranges)
                ax1.plot(x, self.reflector_data.ranges, label="Ranges", color="blue", linewidth=2)
                ax1.set_xlabel("Scanning Angle", fontsize=14)
                ax1.set_ylabel("Ranges", fontsize=14, color="blue")
                ax1.tick_params(axis='y', labelcolor="blue")
                ax1.grid(True, linestyle="--", alpha=0.7)

                # Trục y thứ hai (intensities)
                ax2 = ax1.twinx()
                ax2.plot(x, self.reflector_data.intensities, label="Intensities", color="orange", linestyle="--", linewidth=2)
                ax2.set_ylabel("Intensities", fontsize=14, color="orange")
                ax2.tick_params(axis='y', labelcolor="orange")

                # Thêm tiêu đề
                plt.title("Filter reflector in 100 times mean in 90cm", fontsize=16)

                # Hiển thị đồ thị
                fig.tight_layout()
                plt.show()

                # x = self.reflector_dataIndex_list

                # index_list = []
                # for index in range(0, 4*len(self.reflector_dataIndex_list)):
                #     index_list.append(index)

                # x = index_list

                # Tạo biểu đồ
                # plt.figure(figsize=(10, 6))  # Kích thước biểu đồ

                # # Vẽ đường 1
                # # plt.plot(x, self.reflector_dataIntensities_list, label='Sin(x)', color='blue', linewidth=2)

                # # Vẽ đường 2
                # plt.plot(x, self.reflector_dataIntensities_gauss_list, label='Cos(x)', color='red', linestyle='--', linewidth=2)

                # # Thêm tiêu đề và nhãn trục
                # plt.title('Biểu đồ 2 đường: Sin và Cos', fontsize=16)
                # plt.xlabel('X', fontsize=14)
                # plt.ylabel('Y', fontsize=14)

                # # Thêm chú thích
                # plt.legend(fontsize=12)

                # # Hiển thị lưới
                # plt.grid(True, linestyle='--', alpha=0.7)

                # # Hiển thị biểu đồ
                # plt.show()

                # -- move to next step
                self.process = -1

            # elif self.process == 16:
            #     pass

            self.rate.sleep()

def main():
    print ("--- Run estimate_reflector_center ---")
    program = Estimate_reflector_center()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




