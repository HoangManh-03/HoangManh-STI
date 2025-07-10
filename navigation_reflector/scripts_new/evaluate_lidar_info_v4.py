#!/usr/bin/env python3

"""
    - Author: Archie
    - Date: 2025-04-08
    - Description: Get info of lidar like x, y, theta after matching:
    
    Version 4:        
        + Check lại thuật toán bù chuyển động theo các pha: tiến, lùi, quay : OK
            -> Áp dụng thuật toán bù cho dữ liệu gốc từ node detect gương

        + Thử cover trường hợp khi chỉ có 2 gương có thể khớp được.

        + Sử dụng thư viện multiprcocessing cho việc detect gương

        + fix lỗi:                                                            OK
            - Lỗi chia cho 0, khi xử lý dữ liệu từ gương detect
        
        + Thêm code ghi log file => cho các trường hợp mất matching, để debug lỗi code : OK
            - Các trường dữ liệu cần ghi vào log file

        + Ở chế độ navigation, khớp gương theo thứ tự quét trước sau của gương có ổn hơn ko ??
            => khả năng cao là ko, vì dữ liệu Scan sẽ hoàn thành rồi nó ms pub ra => có sắp xếp lại theo khoảng cách hay ko thì nó vẫn như vậy
        
        + Fix lỗi khớp điểm gương sai tại chế độ navigation
            - vị trí gương đang 14 (rmse < ngưỡng) -> vị trí gương số 12 (rmse > cao hơn ngưỡng)
"""

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Int8, String
from nav_msgs.msg import Odometry
from math import atan2, sin, cos, sqrt, fabs, degrees, isnan, radians, log, exp, asin, acos, tan
from math import pi as PI
import rospy
import time
import copy

import json

import numpy as np
from scipy.optimize import minimize

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped, TwistWithCovarianceStamped
from navigation_reflector.msg import *
from itertools import combinations, product

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import Counter
from sti_msgs.msg import *
import csv
import os

from uptime import uptime
from datetime import datetime

class reflectorMap():
    def __init__(self, _id = 0, _x = 0., _y = 0.):
        self.id = _id
        self.x = _x
        self.y = _y

class Navigation_mode():
    def __init__(self):
        rospy.init_node('navigation_node', anonymous = True)
        self.rate = rospy.Rate(50)

        # -- Constant varibles
        # self.SCANNING_FREQUENCY = 11 #Hz
        self.SCANNING_FREQUENCY = rospy.get_param('~scanning_frequency', 11)
        self.SCANNING_PERIOD = 1/self.SCANNING_FREQUENCY

        # self.DISTANCE_X_BETWEEN_LIDAR_RB = 0.404 #m
        # self.DISTANCE_Y_BETWEEN_LIDAR_RB = 0.0 #m
        self.DISTANCE_X_BETWEEN_LIDAR_RB = rospy.get_param('~distance_x', 0.404)
        self.DISTANCE_Y_BETWEEN_LIDAR_RB = rospy.get_param('~distance_y', 0.0)
        # self.ANGLE_BETWEEN_LIDAR_RB = 0  # rad

        # -- global variables
        self.process = 0
        self.pre_mess = ''
        self.ls_predict_ref = []

        # # -- Khai báo tf -- 
        self.br = tf.TransformBroadcaster()
        self.ref_frame = rospy.get_param('~ref_frame', 'world')
        self.origin_frame = rospy.get_param('~origin_frame', 'frame_map_nav350')
        # self.ref_frame = "world"
        # self.origin_frame = "frame_map_nav350"

        self.translation = (0,0,0)
        self.quanternion = quaternion_from_euler(0, 0, 0)

        self.list_M_distance = []   # khoảng cách giữa các điểm gương trên map tham chiếu
        self.list_M_angle = []      # Góc giữa các gương tham chiếu

        # -- 
        self.num_refStart = -1
        self.num_getPoseReflector = 0
        self.data_store = []
        self.ls_refFilter_average = []

        self.list_N_distance = []   # khoảng cách giữa các điểm gương phát hiện được
        self.list_N_angle = []      # Góc giữa các gương phát hiện được

        # -- 
        # self.DISTANCE_THRESHOLD_MAX = 0.4         # before 0.1
        # self.DISTANCE_THRESHOLD_ORIGIN = 0.02     # before 0.01
        # self.DISTANCE_THRESHOLD_INCREMENT = 0.1  # beforre 0.01

        self.DISTANCE_THRESHOLD_MAX = rospy.get_param('~distance_threshold_max', 0.1)
        self.DISTANCE_THRESHOLD_ORIGIN = rospy.get_param('~distance_threshold_origin', 0.01)
        self.DISTANCE_THRESHOLD_INCREMENT = rospy.get_param('~distance_threshold_increment', 0.01)

        self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_ORIGIN  #  (zf = 2 cm ), do sai số vị trí các gương là +- 0.005 m => 0,014 m

        # for map nav350
        # self.ANGLE_THRESHOLD_MAX = 0.02
        # self.ANGLE_THRESHOLD_ORIGIN = 0.001

        # self.ANGLE_THRESHOLD_MAX = 0.4             # 0.06
        # self.ANGLE_THRESHOLD_ORIGIN = 0.01              # before : 0.01

        # self.ANGLE_THRESHOLD_INCREMENT = 0.1              # 0.015

        self.ANGLE_THRESHOLD_MAX = rospy.get_param('~angle_threshold_max', 0.06)
        self.ANGLE_THRESHOLD_ORIGIN = rospy.get_param('~angle_threshold_origin', 0.01)
        self.ANGLE_THRESHOLD_INCREMENT = rospy.get_param('~angle_threshold_increment', 0.015)

        self.angle_matching_error_threshold = self.ANGLE_THRESHOLD_ORIGIN     # before: 0.02 # +- 0.005 m => 0.34 độ => 0.5 độ : 
        self.list_Z_distance = []
        self.list_Z_angle = []
        self.min_distance_values = []
        self.min_angle_values = []
        self.posOf_min_distance_values = []
        self.posOf_min_angle_values = []

        self.list_IDref_referential_satify = []
        self.list_IDref_detected_sastify = []

        self.displacement_dx = 0     # khoảng dịch chuyển x cuả robot so vơí vị trí bắt đâù
        self.displacement_dy = 0     # khoảng dịch chuyển y cuả robot so vơí vị trí bắt đâù

        self.ANGLE_TRANSLATE = PI

        # self.ANGLE_LIMIT_LOWER = 5
        # self.ANGLE_LIMIT_UPPER = 175

        self.ANGLE_LIMIT_LOWER = rospy.get_param('~angle_limit_lower', 5)
        self.ANGLE_LIMIT_UPPER = rospy.get_param('~angle_limit_upper', 175)

        self.pre_phiR = 0
        self.step = 0

        self.x_tf = 0.0
        self.y_tf = 0.0
        self.r_tf = 0.0

        self.ls_reference_ref_dr = []
        self.ls_reference_ref_d = []
        self.ls_reference_ref_theta = []
        self.ls_reference_ref_xy = []

        self.ls_reference_ref_dr_full = []
        self.ls_reference_ref_d_full = []
        self.ls_reference_ref_theta_full = []
        self.ls_reference_ref_xy_full = []

        # self.ls_dectected_ref = []
        self.count_cal_again_zeromatch = 0
        self.count_cal_again_samematch = 0
        self.count_cal_again_highrmse = 0
        self.rmse_xy = 0                  # chỉ số đánh giá
        self.rmse_dr = []
        self.w_start = 0.0

        # self.WEIGHT_COEF = 25      # before: 25
        # self.VEL_COEF = 0.02
        # self.OMEGA_COEF = 0.04        # before 0.01
        self.WEIGHT_COEF = rospy.get_param('~weight_coef', 1.5)   #25
        self.VEL_COEF = rospy.get_param('~vel_coef', 0.02)
        self.OMEGA_COEF = rospy.get_param('~omega_coef', 0.04)

        self.RMSE_COEF = rospy.get_param('~rmse_coef', 3)
        self.RMSE_VEL_COEF = rospy.get_param('~rmse_vel_coef', 3)
        self.RMSE_OMEGA_COEF = rospy.get_param('~rmse_omega_coef', 0.3)
        self.RMSE_FORWARD_THRESHOLD_MAX = rospy.get_param('~rmse_forward_threshold_max', 0.23)
        self.RMSE_BACKWARD_THRESHOLD_MAX = rospy.get_param('~rmse_backward_threshold_max', 0.05)
        self.RMSE_SPIN_THRESHOLD_MAX = rospy.get_param('~rmse_spin_threshold_max', 0.09)
        self.RMSE_STANDTILL_THRESHOLD_MAX = rospy.get_param('~rmse_standtill_threshold_max', 0.05)

        self.rmse_start = self.RMSE_STANDTILL_THRESHOLD_MAX

        self.pre_w = 0.0

        self.ls_rmse = [0, 0]
        self.ls_w = [0, 0]
        self.ls_nomatch = [0, 0]
        self.ls_samematch = [0, 0]
        self.ls_highrmse = [0, 0]
        self.ls_localID = []
        self.ls_globalID = []

        self.count_zeromatch = 0
        self.count_samematch = 0
        self.count_highrmse_navmode_when_spin = 0
        self.count_highrmse_navmode_when_forward = 0
        self.count_highrmse_navmode_when_backward = 0
        self.count_highrmse_navmode_when_standtill = 0

        # self.RMSE_THRESHOLD_INITMODE = 10.0              # before: 0.1
        # self.RMSE_THRESHOLD_NAVMODE = 0.1
        # self.DISTANCE_COEF = 0.0

        self.RMSE_THRESHOLD_INITMODE = rospy.get_param('~rmse_threshold_initmode', 0.03)
        self.RMSE_THRESHOLD_NAVMODE = rospy.get_param('~rmse_threshold_navmode', 0.1)
        self.DISTANCE_COEF_LOWER = rospy.get_param('~distance_coef_lower', 0.0)
        self.DISTANCE_COEF_UPPER = rospy.get_param('~distance_coef_upper', 0.0)
    
        self.collision_ref_case = 0
        self.collision_det_case = 0

        self.ls_IDransac = 0
        self.ls_info_detref = 0

        self.completed_initmode = 0
        self.completed_navmode = 1

        self.count_result = 0
        self.sum_tfx = 0
        self.sum_tfy = 0
        self.sum_tfr = 0
        self.mean_tfx = 0
        self.mean_tfy = 0
        self.mean_tfr = 0

        # self.NUMBER_COUNT_RESULT = 5
        self.NUMBER_COUNT_RESULT = rospy.get_param('~number_count_result', 5)
        self.NUMBER_REFLECTOR_HOLD = rospy.get_param('~number_reflector_hold', 3)
        self.RANGE_FINDDING = rospy.get_param('~range_finding', 30) # m
        self.finding_coef = 1 

        self.range_start = 0.0
        self.range_end = 0.0

        self.count_cal_initmode = 0
        self.is_finding_initpose = True
        self.finding_status = 0                    # 0: Đang chờ tìm vị trí, 1: Tìm vị trí hoàn thành, 2: tìm nhưng ko ra

        self.flag_stop_program = 0
        
        self.time_start = 0.0
        self.time_initmode_done = 0.0

        self.d_lower = 0.0                # nhằm cho việc khởi tạo lại vị trí của robot
        self.d_upper = 0.0                # nhằm cho việc khởi tạo lại vị trí của robot 

        self.dir_csv_file_xy = "/home/stivietnam/catkin_ws/src/navigation_reflector/debug/data_xy.csv"
        self.dir_csv_file_dr = "/home/stivietnam/catkin_ws/src/navigation_reflector/debug/data_dr.csv"
        self.path_log_r2000 = "/home/stivietnam/catkin_ws/src/navigation_reflector/debug/log_r2000_navigation.txt"

        self.NUMBER_INIT_ERROR = 100

        self.pre_detected_ref_xy = []
        self.pre_detected_ref_dr = []
        self.pre_dnear = 0.0
        self.pre_dfar = 0.0
        self.pre_reference_ref_xy = []
        self.pre_reference_ref_dr = []
        self.pre_vx = 0.0
        self.pre_thetaz = 0.0
        self.pre_wsigma = 0.0
        self.pre_min_values = []
        self.pre_valid_indices = []
        self.pre_rmse = 0.0
        self.pre_rmse_threshold = 0.0
        self.pre_rmse_start = 0.0
        
        self.count_check_rmse = 0
        
        ##############################################  ROS PUB & SUB ######################################################
        # -- ros pub && sub 
        rospy.Subscriber('/r2000_reflectors', R2000_reflectors, self.callback_infoRawReflector, queue_size = 10)
        self.data_raw_reflector = R2000_reflectors()
        self.is_rawReflector = False

        rospy.Subscriber('/map', R2000_reflectors, self.callback_map, queue_size = 10)
        self.data_map = R2000_reflectors()
        self.is_recv_mapdata = False

        rospy.Subscriber('/raw_vel', TwistWithCovarianceStamped, self.callback_rawvel, queue_size = 10)
        self.data_rawvel = TwistWithCovarianceStamped()
        self.is_recv_rawvel = False

        rospy.Subscriber('/odom_rf2o', Odometry, self.callback_odomRf2o, queue_size = 10)
        self.data_odomRf2o = Odometry()
        self.is_recv_odomRf2o = False

        self.lidar_pose_pub = rospy.Publisher('/r2000_data', R2000_data, queue_size=10)

        self.marker_predict = rospy.Publisher('/visualization_marker_predict', Marker, queue_size=10)
        self.marker_normal = rospy.Publisher('/visualization_marker_normal', Marker, queue_size=10)

        # -- node subcribe -- 
        # rospy.Subscriber('NN_infoRespond' , NN_infoRespond, self.callBack_NNinfoRespond)
        # self.data_NNinfoRespond = NN_infoRespond()
        # self.is_recv_NNinfo = False

    def callback_infoRawReflector(self, data):
        self.data_raw_reflector = data
        self.is_rawReflector = True

    def callback_map(self, data):
        self.data_map = data
        self.is_recv_mapdata = True

    def callback_rawvel(self, data):
        self.data_rawvel = data
        self.is_recv_rawvel = True

    def callback_odomRf2o(self, data):
        self.data_odomRf2o = data
        self.is_recv_odomRf2o = True

    # -- hàm nhận dữ liệu từ AGV-TF 
    # def callBack_NNinfoRespond(self, data):
    #     self.data_NNinfoRespond = data
    #     self.is_recv_NNinfo = True

    def euler_to_quaternion(self, euler):
        quat = Quaternion()
        odom_quat = quaternion_from_euler(0, 0, euler)
        quat.x = odom_quat[0]
        quat.y = odom_quat[1]
        quat.z = odom_quat[2]
        quat.w = odom_quat[3]
        return odom_quat

    def pub_marker(self, pub_name, list_point, color, scale, text_pos ):
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
            sphere_marker.color.r = color[0]
            sphere_marker.color.g = color[1]
            sphere_marker.color.b = color[2]
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
            text_marker.pose.position.z = p.z + text_pos  # Hiển thị phía trên hình cầu
            text_marker.scale.z = scale  # Kích thước chữ   #1
            text_marker.color.a = 1.0
            text_marker.color.r = color[0]
            text_marker.color.g = color[1]
            text_marker.color.b = color[2]
            text_marker.text = str(i+1)  # Hiển thị ID

            # Pose
            text_marker.pose.orientation.x = 0.0
            text_marker.pose.orientation.y = 0.0
            text_marker.pose.orientation.z = 0.0
            text_marker.pose.orientation.w = 1.0

            marker_array.append(text_marker)

        # Xuất tất cả marker
        for marker in marker_array:
            pub_name.publish(marker)

    def log_mess(self, typ, mess, val):
        if self.pre_mess != mess:
            if typ == "info":
                rospy.loginfo (mess + ": %s", val)
            elif typ == "warn":
                rospy.logwarn (mess + ": %s", val)
            else:
                rospy.logerr (mess + ": %s", val)
        self.pre_mess = mess

    def find_min_max(self, lst):
        if not lst:
            return None, None  # Trả về None nếu danh sách rỗng

        min_value = min(lst, key=lambda x: x[1])[1]
        max_value = max(lst, key=lambda x: x[1])[1]

        return min_value, max_value

    def common_elements(self, list1, list2):
        # Chuyển danh sách nhỏ thành set để tìm kiếm nhanh hơn
        if len(list1) > len(list2):
            list1, list2 = list2, list1  # Đảm bảo list1 luôn nhỏ hơn

        set_list2 = set(list2)
        return [x for x in list1 if x in set_list2]
    
    def multiply_matrices(self, A, B):
        # Kiểm tra nếu số cột của A khác số hàng của B thì không thể nhân
        if len(A[0]) != len(B):
            raise ValueError("Số cột của ma trận A phải bằng số hàng của ma trận B.")

        # Tạo ma trận kết quả với số hàng = số hàng của A, số cột = số cột của B
        result = [[0 for _ in range(len(B[0]))] for _ in range(len(A))]

        # Nhân ma trận A với B
        for i in range(len(A)):  # Duyệt từng hàng của A
            for j in range(len(B[0])):  # Duyệt từng cột của B
                for k in range(len(B)):  # Duyệt từng phần tử để nhân
                    result[i][j] += A[i][k] * B[k][j]

        return result

    def get_matrix_size(self, matrix):
        rows = len(matrix)  # Số hàng
        cols = len(matrix[0]) if matrix else 0  # Số cột (giả sử ma trận không rỗng)
        return rows, cols

    def find_min_in_columns(self, matrix):
        if not matrix or not matrix[0]:  # Kiểm tra ma trận rỗng
            return []

        rows = len(matrix)
        cols = len(matrix[0])
        
        min_values = []  # Lưu giá trị nhỏ nhất của mỗi cột
        positions = []   # Lưu vị trí (hàng, cột) của giá trị nhỏ nhất

        for j in range(cols):  # Duyệt từng cột
            min_value = matrix[0][j]  # Giả sử phần tử đầu tiên là nhỏ nhất
            min_row = 0  # Vị trí hàng của giá trị nhỏ nhất

            for i in range(1, rows):  # Duyệt từng hàng
                if matrix[i][j] < min_value:
                    min_value = matrix[i][j]
                    min_row = i

            min_values.append(min_value)
            positions.append((min_row, j))  # Lưu vị trí (hàng, cột)

        return min_values, positions
    
    def column_minimums(self, n, list2, matrix):
        min_values = []
        positions = []

        # Duyệt qua từng cột
        if n == 2:
            for col in range(len(list2)):
                column_values = [row[col][4] for row in matrix]
                min_value = min(column_values)
                min_position = column_values.index(min_value)
                
                min_values.append(min_value)
                positions.append([min_position, col])

        elif n == 3:
            for col in range(len(list2)):
                column_values = [row[col][6] for row in matrix]
                min_value = min(column_values)
                min_position = column_values.index(min_value)
                
                min_values.append(min_value)
                positions.append([min_position, col])

        # print("\nGiá trị khoảng cách nhỏ nhất trong từng cột:")
        # for i in range(len(min_values)):
        #     print(f"Cột {i+1}: Giá trị khoảng cách nhỏ nhất = {min_values[i]}, Vị trí = {positions[i]}")

        return min_values, positions

    def SVD_algorithm(self, ls_pointMap, ls_pointRef):
        # Tập hợp A và B (ví dụ)
        A = np.array(ls_pointMap) # map
        B = np.array(ls_pointRef) # gương

        # Tính trung bình của A và B
        mu_A = np.mean(A, axis=0)
        mu_B = np.mean(B, axis=0)

        # Tái căn giữa các điểm (A_n và B_n)
        A_n = A - mu_A
        B_n = B - mu_B

        # Tính ma trận hiệp phương sai H
        H = np.zeros((2, 2))

        for i in range(len(A)):
            H += np.outer(A_n[i], B_n[i])

        # Phân tích SVD trên ma trận H
        U, S, Vt = np.linalg.svd(H)

        # Tính ma trận quay R
        R = Vt.T @ U.T

        # Nếu cần điều chỉnh (det(R) = -1), sửa đổi Vt hoặc R để giữ R là ma trận quay hợp lệ
        if np.linalg.det(R) < 0:
            Vt[1,:] *= -1
            R = Vt.T @ U.T

        # Tính vector tịnh tiến t
        t = -R @ mu_A + mu_B

        # Tọa độ của robot trong hệ tọa độ robot (0, 0)
        # x_l, y_l = np.array([0, 0])

        # Tính tọa độ của robot trong hệ tọa độ toàn cục (với chuyển vị)
        robot_position_global = (np.linalg.inv(R) @ (-t)).T
        
        # Tính góc quay theta_g
        theta_g = -np.arctan2(R[1, 0], R[0, 0])

        # print(f"Ma trận quay R:\n{R}")
        # print(f"Vector tịnh tiến t: {t}")
        # print(f"Tọa độ trong hệ toàn cục: {robot_position_global}")
        # print(f"Góc quay trong hệ toàn cục: {theta_g} độ")

        return robot_position_global[0], robot_position_global[1], theta_g

    def count_duplicate_pairs(self, lst):
        # Chuyển danh sách con thành tuple để có thể đếm được
        tuple_list = [tuple(sublist) for sublist in lst]

        # Đếm số lần xuất hiện của từng cặp
        count_dict = Counter(tuple_list)

        return dict(count_dict)

    def find_max_element(self, x, data):
        # Lọc các phần tử có giá trị thứ 2 trong tuple bằng x
        filtered = {key: value for key, value in data.items() if (key[1] == x)}
        
        # print(filtered)
        if not filtered:
            return None  # Nếu không có phần tử nào thỏa mãn, trả về None
        
        # Tìm phần tử có giá trị lớn nhất trong dictionary đã lọc
        max_element = max(filtered, key=filtered.get)
        
        return max_element

    def pub_lidar_pose(self, pub, x, y, phi, ls_localID, ls_globalID, no_ref):
        lidar_data = R2000_data()
        lidar_data.header.frame_id = "scanner_link"
        lidar_data.header.stamp = rospy.Time.now()
        lidar_data.x = x
        lidar_data.y = y
        lidar_data.phi = phi
        lidar_data.localID = ls_localID
        lidar_data.globalID = ls_globalID
        lidar_data.number_reflectors = no_ref
        pub.publish(lidar_data)

    def pub_lidar_pose_full(self, pub, ls_rms, ls_w, ls_nomatch, ls_samematch, ls_highrmse, x, y, phi, ls_localID, ls_globalID, w, process, no_ref):
        lidar_data = R2000_data()
        lidar_data.header.frame_id = "scanner_link"
        lidar_data.header.stamp = rospy.Time.now()
        lidar_data.rmse = ls_rms
        lidar_data.weight = ls_w
        lidar_data.calagain_momatch = ls_nomatch
        lidar_data.calagain_samematch = ls_samematch
        lidar_data.calagain_highrmse = ls_highrmse
        lidar_data.x = x
        lidar_data.y = y
        lidar_data.phi = phi
        lidar_data.w = w
        lidar_data.localID = ls_localID
        lidar_data.globalID = ls_globalID
        lidar_data.process = process
        lidar_data.number_reflectors = no_ref
        pub.publish(lidar_data)

    def calculate_reflector_position_in_map(self, x_ss, y_ss, r_ss, x_ref, y_ref):
        # Tính toán ma trận quay thủ công (cos(R), sin(R))
        cos_R = cos(r_ss)
        sin_R = sin(r_ss)

        x_m = x_ss + (cos_R*x_ref - sin_R*y_ref)
        y_m = y_ss + (sin_R*x_ref + cos_R*y_ref)

        return x_m, y_m

    def cal_info_idmatch_in_initmode(self, idmatch, ls_detected_ref, ls_reference_refxy, ls_reference_refdr):
        # Tồn tại id tuyệt đôí -> Các id khớp là [(3, 1), (4, 5), (8, 6)]
        # # - Step 0: Lọc map tham chiếu khớp trong map gốc
        n = len(ls_reference_refxy)
        ls_reference_refFilter = []
        ls_reference_refFilter_dr = []
        for id_ref in idmatch:
            for i, ref in enumerate(ls_reference_refxy):
                if i+1 == id_ref[0]:                               # ref[0] -> i+1: change here
                    ls_reference_refFilter.append([ref[1], ref[2]])
                    ls_reference_refFilter_dr.append([ls_reference_refdr[i][1], ls_reference_refdr[i][2]])
        
        # print(" Độ dài thông số các gương tham chiếu khớp là: ", len(ls_reference_refFilter))

        n = len(ls_detected_ref)
        # print("Thông tin các gương phát hiện là ", ls_detected_ref)
        ls_detected_refFilter = []
        for id_ref in idmatch:
            for ref in ls_detected_ref:
                if ref[0] == id_ref[1]:
                    # print(f"Phần tử mới được thêm vào list là: x = {ref[1]}, y = {ref[2]}")
                    ls_detected_refFilter.append([ref[1], ref[2]])
        
        # print("Độ dài thông số các gương phát hiện khớp là: ", len(ls_detected_refFilter))

        # -- Step 1: find pose of robot
        xr, yr, phiR = self.SVD_algorithm(ls_reference_refFilter, ls_detected_refFilter)
        # print("num ref matching: ", len(ls_detected_refFilter), "| Pose: ", xr, yr, degrees(phiR))

        # -- Step 2: Tính toán RMS để đánh giá kết quả thu được
        detected_mirrors = []
        reference_mirrors = []

        detected_mirrors_dr = []
        reference_mirrors_dr = []

        for i, ref in enumerate(ls_detected_refFilter):
            x_cv, y_cv = self.calculate_reflector_position_in_map(xr, yr, phiR, ref[0], ref[1])

            detected_mirrors.append((x_cv, y_cv))
            reference_mirrors.append((ls_reference_refFilter[i][0], ls_reference_refFilter[i][1]))

            d_det = sqrt(x_cv * x_cv + y_cv * y_cv)
            theta_det = atan2(y_cv, x_cv)

            detected_mirrors_dr.append((d_det, theta_det))
            reference_mirrors_dr.append((ls_reference_refFilter_dr[i][0], ls_reference_refFilter_dr[i][1]))

        # - Tính rmse cho khoảng cách xy
        detected_arr = np.array(detected_mirrors)
        reference_arr = np.array(reference_mirrors)
        
        # Tính RMSE for xy
        error = detected_arr - reference_arr
        rmse_xy = np.sqrt(np.mean(np.sum(error**2, axis=1)))

        # - Tính RMSE for dr => trọng số w cho navigation mode
        detected_arr_dr = np.array(detected_mirrors_dr)
        reference_arr_dr = np.array(reference_mirrors_dr)
        
        rmse_dr = [abs((x - a)*(y - b)) for (x, y), (a, b) in zip(detected_arr_dr, reference_arr_dr)]

        w_start = self.WEIGHT_COEF * max(rmse_dr)
        # print("Trọng số khởi tạo là: ", self.w_start)

        # -- publish data
        ls_localID = []
        ls_globalID = []
        for id_ref in idmatch:
            ls_globalID.append(id_ref[0])
            ls_localID.append(id_ref[1])

        return xr, yr, phiR, rmse_xy, max(rmse_dr), w_start, ls_globalID, ls_localID

    def cal_info_idmatch_in_navmode(self, idmatch, ls_detected_ref, ls_reference_refxy, ls_reference_refdr):
        # # - Step 0: Lấy các điểm thoả mãn trong map tham chiếu
        n = len(ls_reference_refxy)
        ls_reference_refFilter = []
        ls_reference_refFilter_dr = []
        for id_ref in idmatch:
            for i, ref in enumerate(ls_reference_refxy):
                if id_ref != -1 and i == id_ref:
                    ls_reference_refFilter.append([ref[1], ref[2]])
                    ls_reference_refFilter_dr.append([ls_reference_refdr[i][1], ls_reference_refdr[i][2]])

        # print(f"List tham chiếu khớp là: {ls_reference_refFilter}")
        # print("-----")
        
        # -- Step 1: Lấy các điểm thoả mãn trong map phát hiện
        ls_detected_refFilter = []
        for i, id_ref in enumerate(idmatch):
            if id_ref != -1:
                ls_detected_refFilter.append([ls_detected_ref[i][1], ls_detected_ref[i][2]])

        # -- Step 2: find pose of robot
        xr, yr, phiR = self.SVD_algorithm(ls_reference_refFilter, ls_detected_refFilter)
        # print("num ref matching: ", len(ls_detected_refFilter), "| Pose: ", xr, yr, degrees(phiR))

        # -- Step 3: Tính toán RMS để đánh giá kết quả thu được
        detected_mirrors = []
        reference_mirrors = []

        detected_mirrors_dr = []
        reference_mirrors_dr = []

        for i, ref in enumerate(ls_detected_refFilter):
            x_cv, y_cv = self.calculate_reflector_position_in_map(xr, yr, phiR, ref[0], ref[1])
            detected_mirrors.append((x_cv, y_cv))
            reference_mirrors.append((ls_reference_refFilter[i][0], ls_reference_refFilter[i][1]))

            d_det = sqrt(x_cv * x_cv + y_cv * y_cv)
            theta_det = atan2(y_cv, x_cv)

            detected_mirrors_dr.append((d_det, theta_det))
            reference_mirrors_dr.append((ls_reference_refFilter_dr[i][0], ls_reference_refFilter_dr[i][1]))

        detected_arr = np.array(detected_mirrors)
        reference_arr = np.array(reference_mirrors)
        
        # Tính RMSE
        error = detected_arr - reference_arr
        rmse_xy = np.sqrt(np.mean(np.sum(error**2, axis=1)))
        # print(f" Hệ số rms là: {self.rmse_xy}")

        # - Tính RMSE for dr => cập nhật trọng số w cho navigation mode
        detected_arr_dr = np.array(detected_mirrors_dr)
        reference_arr_dr = np.array(reference_mirrors_dr)
        
        rmse_dr = [abs((x - a)*(y - b)) for (x, y), (a, b) in zip(detected_arr_dr, reference_arr_dr)]

        w_start = self.WEIGHT_COEF * max(rmse_dr)

        return xr, yr, phiR, rmse_xy, max(rmse_dr), w_start

    def save_csv_file(self, vx, vy, vz, rmse_xy, rmse_threshold, filename):
        file_exists = os.path.isfile(filename)

        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)

            # Ghi header nếu file chưa tồn tại
            # if not file_exists:
            #     writer.writerow(['vx', 'vy', 'vz', 'rmse_dr', 'rmse_threshold'])

            # Ghi dữ liệu
            writer.writerow([vx, vy, vz, rmse_xy, rmse_threshold])

    def sort_list_with_indices(self, lst):
        # Bước 1: Tạo danh sách (giá trị, chỉ số ban đầu)
        indexed_list = list(enumerate(lst))

        # Bước 2: Sắp xếp danh sách theo giá trị
        sorted_list = sorted(indexed_list, key=lambda x: x[1])

        # Bước 3: Tạo danh sách kết quả
        sorted_values = [val for idx, val in sorted_list]  # Danh sách đã sắp xếp
        sorted_indices = [idx for idx, val in sorted_list]  # Chỉ số gốc của từng phần tử

        return sorted_values, sorted_indices

    def log_r2000_data(self, moving_case, case, ls_detected_ref, convert_posReflector, d_near, d_far, ls_reference_ref_xy_filter, ls_reference_ref_dr_filter, vx, thetaz, w_sigma, min_indices, valid_indices, rmse_xy, rmse_threshold, rmse_start):
        self.file_log = open(self.path_log_r2000, "a+")

        # if moving_case == 0:
        #     self.file_log.write("Trường hợp robot đang đứng yên!!!\n")
        #     self.file_log.write("-----\n")
        # elif moving_case == 1:
        #     self.file_log.write("Trường hợp robot đang chạy tiến!!!\n")
        #     self.file_log.write("-----\n")
        # elif moving_case == 2:
        #     self.file_log.write("Trường hợp robot đang chạy lùi!!!\n")
        #     self.file_log.write("-----\n")

        self.file_log.write("\nList gương detect theo xy là: ")
        self.file_log.write(str(ls_detected_ref) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("List gương detect theo dr là: ")
        self.file_log.write(str(convert_posReflector) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("Khoảng tìm kiếm gương tham chiếu trong bán kính từ " + str(d_near) + " tới " + str(d_far) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("List gương tham chiếu theo xy là: ")
        self.file_log.write(str(ls_reference_ref_xy_filter) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("List gương tham chiếu theo dr là: ")
        self.file_log.write(str(ls_reference_ref_dr_filter) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("Vận tốc dài là: " + str(vx) + ",vận tốc góc là: " + str(thetaz) + ",w_sigma là: " + str(w_sigma) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("Giá trị nhỏ nhất tại mỗi cột: ")
        self.file_log.write(str(min_indices) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("Chỉ số hàng tương ứng sau khi loại bỏ id trùng: ")
        self.file_log.write(str(valid_indices) + "\n")
        self.file_log.write("-----\n")

        if case == 2:
            self.file_log.write("Hệ số rmse hiện tại là: " + str(rmse_xy) + ",rmse khởi tạo là: " + str(rmse_start) + ",rmse ngưỡng là: " + str(rmse_threshold) + "\n")
            self.file_log.write("-----\n")     

        self.file_log.write("\n#########################################\n")
        self.file_log.close()

    def log_r2000_data_before(self, moving_case, case, ls_detected_ref, convert_posReflector, d_near, d_far, ls_reference_ref_xy_filter, ls_reference_ref_dr_filter, vx, thetaz, w_sigma, min_indices, valid_indices, rmse_xy, rmse_threshold, rmse_start):
        # -- write data to log file
        now = datetime.now()
        current_time = now.strftime("%B/%d|%H:%M:%S")

        self.file_log = open(self.path_log_r2000, "a+")
        self.file_log.write("\nR2000 log file| ")
        self.file_log.write(str(current_time) + "\n")
        if moving_case == 0:
            self.file_log.write("Trường hợp robot đang đứng yên!!!\n")
            self.file_log.write("-----\n")
        elif moving_case == 1:
            self.file_log.write("Trường hợp robot đang chạy tiến!!!\n")
            self.file_log.write("-----\n")
        elif moving_case == 2:
            self.file_log.write("Trường hợp robot đang chạy lùi!!!\n")
            self.file_log.write("-----\n")

        if case == 1:
            self.file_log.write("Số lượng gương khớp < 3!!!\n")
            self.file_log.write("-----\n")
        
        elif case == 2:
            self.file_log.write("Trường hợp RMSE cao hơn ngưỡng!!!\n")
            self.file_log.write("-----\n")

        self.file_log.write("List gương detect theo xy trước đó là: ")
        self.file_log.write(str(ls_detected_ref) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("List gương detect theo dr trước đó là: ")
        self.file_log.write(str(convert_posReflector) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("Khoảng tìm kiếm gương tham chiếu trong bán kính trước đó từ: " + str(d_near) + " tới " + str(d_far) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("List gương tham chiếu theo xy trước đó là: ")
        self.file_log.write(str(ls_reference_ref_xy_filter) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("List gương tham chiếu theo dr trước đó là: ")
        self.file_log.write(str(ls_reference_ref_dr_filter) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("Vận tốc dài trước là: " + str(vx) + ",vận tốc góc trước là: " + str(thetaz) + ",w_sigma trước là: " + str(w_sigma) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("Giá trị nhỏ nhất tại mỗi cột trước đó: ")
        self.file_log.write(str(min_indices) + "\n")
        self.file_log.write("-----\n")

        self.file_log.write("Chỉ số hàng trước đó tương ứng sau khi loại bỏ id trùng: ")
        self.file_log.write(str(valid_indices) + "\n")
        self.file_log.write("-----\n")

        if case == 2:
            self.file_log.write("Hệ số rmse trước đó là: " + str(rmse_xy) + ",rmse khởi tạo trước đó là: " + str(rmse_start) + ",rmse ngưỡng trước đó là: " + str(rmse_threshold) + "\n")
            self.file_log.write("-----\n")     

        self.file_log.write("\n---------------------------\n")
        self.file_log.close()

    def run(self):
        while not rospy.is_shutdown():
            # -- wait for recv full data
            if self.process == 0:
                c_k = 0
                if self.is_recv_mapdata == True:
                    c_k = c_k + 1
                else:
                    self.log_mess("warn","Wait data from Map node", c_k)

                if self.is_rawReflector == True:
                    c_k = c_k + 1
                else:
                    self.log_mess("warn","Wait data from Estimate reflector center node", c_k)

                if self.is_recv_rawvel == True or self.is_recv_odomRf2o == True:
                    c_k = c_k + 1
                else:
                    self.log_mess("warn","Wait odom data from Kinematic node/ Rf2o", c_k)

                if c_k == 3:
                    rospy.loginfo("Completed wakeup ('_')")
                    self.process = 1             

            # -- Check xem đã nhận được dữ liêu mới không ???
            elif self.process == 1:
                if self.is_rawReflector == False:
                    self.rate.sleep()
                    continue
                    
                self.is_rawReflector = False
                
                num_reflector = self.data_raw_reflector.num_reflector
                if num_reflector < 3:
                    print("Số lượng gương phát hiện nhỏ hơn 3")  # báo lỗi ko đủ gương
                    self.ls_localID = []
                
                else:
                    if self.completed_initmode == 0:
                        self.process = 2
                        self.time_start = time.time()
                    else:
                        self.process = 3
                
                if self.flag_stop_program == 1:
                    print("Dừng chương trình để xử lý lôĩ")
                    return
                
            # -- Tìm ra vị trí khởi tạo cuả robot trên bản đồ và những gương nào đang được khớp với nhau
            elif self.process == 2:
                if self.completed_initmode == 0:
                    
                    if self.step == 0:                    # tính toán dữ liệu k/c và góc từ dữ liệu đâù vào

                        # -- Sắp xếp lại dữ liệu đầu vào theo khoảng cách
                        ls_d = []
                        for ref in self.data_raw_reflector.reflectors:
                            ls_d.append(ref.Polar_Dist)
                        
                        sorted_values, sorted_indices = self.sort_list_with_indices(ls_d)

                        lsReflector_arrange = []
                        ls_poseReflector = []

                        for j, indices in enumerate(sorted_indices):
                            Reflector_info = Reflector_data()
                            for i, ref in enumerate(self.data_raw_reflector.reflectors):
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

                                    ls_poseReflector.append([ref.Cart_X, ref.Cart_Y, 0.0])

                        # - publish to ros
                        # self.pub_marker(self.marker_normal, ls_poseReflector, [0,1,0], 0.5, 0.0)

                        # - update reflector info
                        self.data_raw_reflector.reflectors = lsReflector_arrange

                        # -- tìm list gương detected
                        ls_detected_ref = []
                        for index, ref in enumerate(self.data_raw_reflector.reflectors):
                            ls_detected_ref.append([ref.LocalID, ref.Cart_X, ref.Cart_Y])
                        
                        # print("Các điểm gương phát hiện tức thời là", ls_detected_ref)
                        # - Xác định khoảng cách và góc giưã các gương
                        self.list_N_distance = []
                        n = len(ls_detected_ref)
                        for i in range(0, n):
                            for j in range(i, n):
                                if j != i:
                                    dx = ls_detected_ref[j][1] - ls_detected_ref[i][1]
                                    dy = ls_detected_ref[j][2] - ls_detected_ref[i][2]
                                    d = sqrt(dx*dx + dy*dy)
                                    self.list_N_distance.append(d)

                        # print("Mảng thông số khoảng cách giưã các gương phát hiện là: ", self.list_N_distance)        ## số lượng là nC2 giá trị

                        # -- Tìm góc liên kết giữa các gương phát hiện
                        n = len(ls_detected_ref)
                        combs = list(combinations(ls_detected_ref, 3))
                        # Tìm góc của các tổ hợp đó
                        self.list_N_angle = []
                        for comb in combs:
                            list_N_comb_final = []
                            for i in range(0, 3):
                                for j in range(i, 3):             
                                    if j != i:
                                        dx = comb[j][1] - comb[i][1]
                                        dy = comb[j][2] - comb[i][2]
                                        d = sqrt(dx*dx + dy*dy)
                                        list_N_comb_final.append(d)

                            a = list_N_comb_final[0]
                            b = list_N_comb_final[1]
                            c = list_N_comb_final[2]

                            if a == 0.0 or b == 0.0:
                                self.list_N_angle.append(2*PI)

                            else:
                                theta_r = acos((a*a + b*b - c*c)/(2*a*b))

                                if degrees(theta_r) < self.ANGLE_LIMIT_LOWER or degrees(theta_r) > self.ANGLE_LIMIT_UPPER:
                                    self.list_N_angle.append(2*PI)
                                else:
                                    self.list_N_angle.append(theta_r)

                        # print(f'Mảng thông số góc giữa các gương phát hiện với {len(self.list_N_angle)} tổ hợp là: {self.list_N_angle}')

                        # -- update first fidding range
                        # self.range_start = self.data_map.reflectors[0].Polar_Dist
                        # self.range_end = self.range_start + self.RANGE_FINDDING

                        if self.completed_navmode == 1:
                            self.range_end = self.data_map.reflectors[-1].Polar_Dist
                            self.range_start = self.range_end - self.RANGE_FINDDING
                            self.RMSE_THRESHOLD_INITMODE = 0.03
                        else:
                            self.range_start = 0.9*self.d_lower
                            self.range_end = self.d_lower + self.RANGE_FINDDING
                            self.RMSE_THRESHOLD_INITMODE = self.RMSE_THRESHOLD_NAVMODE * 1.2

                        print(f"Khoảng cách tìm kiếm là: {self.range_start} to {self.range_end}, trọng số là {self.RMSE_THRESHOLD_INITMODE}")
                        
                        # - move to next step
                        self.step = 1
                    
                    elif self.step == 1:                  # tìm thông tin về gương tham chiếu tại các vùng khác nhau theo quy luật
                        # -- tìm list gương tham chiếu trong 30m đầu tiên
                        self.ls_reference_ref_xy = []
                        self.ls_reference_ref_d = []
                        self.ls_reference_ref_theta = []
                        self.ls_reference_ref_dr = []

                        self.ls_reference_ref_xy_full = []
                        self.ls_reference_ref_d_full = []
                        self.ls_reference_ref_theta_full = []
                        self.ls_reference_ref_dr_full = []
                        
                        for index, ref in enumerate(self.data_map.reflectors):
                            self.ls_reference_ref_dr_full.append([ref.GlobalID, ref.Polar_Dist, ref.Polar_Phi])
                            self.ls_reference_ref_d_full.append(ref.Polar_Dist)
                            self.ls_reference_ref_theta_full.append(ref.Polar_Phi)
                            self.ls_reference_ref_xy_full.append([ref.GlobalID, ref.Cart_X, ref.Cart_Y])

                            if self.range_start <= ref.Polar_Dist <= self.range_end:
                                self.ls_reference_ref_dr.append([ref.GlobalID, ref.Polar_Dist, ref.Polar_Phi])
                                self.ls_reference_ref_d.append(ref.Polar_Dist)
                                self.ls_reference_ref_theta.append(ref.Polar_Phi)
                                self.ls_reference_ref_xy.append([ref.GlobalID, ref.Cart_X, ref.Cart_Y])
                        
                        # print("Các điểm gương tham chiếu là", self.ls_reference_ref_xy)
                        rospy.logwarn(f"Số lượng điểm gương tham chiếu là: {len(self.ls_reference_ref_xy)}, from {self.range_start} to {self.range_end}, id from {self.ls_reference_ref_xy[0][0]} to {self.ls_reference_ref_xy[-1][0]}")

                        if len(self.ls_reference_ref_xy) < 3:     # lượng gương còn lại < 3
                            rospy.logwarn("Số lượng gương tham chiếu còn lại ko đủ để tiếp tục check")

                            # -- check thử vs dữ liệu khác
                            self.process = 1
                            self.step = 0

                            self.count_cal_initmode += 1

                            if self.count_cal_initmode > self.NUMBER_INIT_ERROR:
                                self.count_cal_initmode = 0
                                rospy.logerr("Ko có đủ gương cho AGV di chuyển => lắp thêm")
                                self.flag_stop_program = 1

                        else:
                                    
                            # - Step 1: Tìm khoảng cách giữa các gương tham chiếu 
                            n = len(self.ls_reference_ref_xy)
                            self.list_M_distance = []

                            for i in range(0, n):
                                for j in range(i, n):
                                    if j != i:
                                        dx = self.ls_reference_ref_xy[j][1] - self.ls_reference_ref_xy[i][1]
                                        dy = self.ls_reference_ref_xy[j][2] - self.ls_reference_ref_xy[i][2]
                                        d = sqrt(dx*dx + dy*dy)
                                        self.list_M_distance.append(d)
                        
                            # print("Mảng thông số khoảng cách các gương tham chiếu là: ", self.list_M_distance)    ## số lượng là nC2 giá trị

                            # - Tìm góc giữa các danh sách gương tham chiếu
                            n = len(self.ls_reference_ref_xy)
                            combs = list(combinations(self.ls_reference_ref_xy, 3))

                            # Tìm góc của các tổ hợp đó
                            self.list_M_angle = []
                            for comb in combs:
                                list_M_comb_final = []
                                for i in range(0, 3):
                                    for j in range(i, 3):             
                                        if j != i:
                                            dx = comb[j][1] - comb[i][1]
                                            dy = comb[j][2] - comb[i][2]
                                            d = sqrt(dx*dx + dy*dy)
                                            list_M_comb_final.append(d)
                                
                                a = list_M_comb_final[0]
                                b = list_M_comb_final[1]
                                c = list_M_comb_final[2]

                                if a == 0 or b == 0:
                                   self.list_M_angle.append(2*PI)

                                else: 
                                    theta_r = acos((a*a + b*b - c*c)/(2*a*b))
                                    if degrees(theta_r) < self.ANGLE_LIMIT_LOWER or degrees(theta_r) > self.ANGLE_LIMIT_UPPER:
                                        self.list_M_angle.append(2*PI)
                                    else:
                                        self.list_M_angle.append(theta_r)

                            # print(f'Mảng thông số góc giữa các gương tham chiếu với {len(self.list_M_angle)} tổ hợp là: {self.list_M_angle}')  # số lượng là n!/(3! * (n-3)!)

                            # - move to next step
                            self.step = 2

                    elif self.step == 2:                   # tìm giá trị khớp gương từ list trên
                        # - Tìm sự khác nhau giữa 2 map gương tham chiếu và map gương detect được
                        # self.list_Z_distance = [[[x[0],x[1], y[0], y[1], abs(x[2] - y[2])] for y in self.list_N_distance] for x in self.list_M_distance]
                        N_distance = np.array(self.list_N_distance)
                        M_distance = np.array(self.list_M_distance)

                        # Tạo ma trận Z_distance với |M_distance[i] - N_distance[j]|
                        Z_distance = np.abs(M_distance[:, np.newaxis] - N_distance)
                        # In kích thước của ma trận
                        # print("Kích thước ma trận Z_distance:", Z_distance.shape)

                        # Tìm giá trị nhỏ nhất của từng cột
                        min_values = np.min(Z_distance, axis=0)

                        # Tìm chỉ số hàng (vị trí) của giá trị nhỏ nhất trong từng cột
                        min_indices = np.argmin(Z_distance, axis=0)

                        # In kết quả
                        # print("Giá trị nhỏ nhất của từng cột:", min_values)
                        # print("Vị trí hàng tương ứng:", min_indices)

                        # So sánh với ngưỡng distance 
                        valid_indices = np.where(min_values < self.distance_matching_error_threshold, min_indices, -1)

                        # In kết quả
                        # print("Giá trị nhỏ nhất thoả mãn tại mỗi cột:", valid_values)
                        # print("Chỉ số các hàng thoả mãn là          :", valid_indices)

                        # - Quy đổi vị trí nhỏ nhất ra các gương tham chiếu và gương phát hiện
                        M_pairs = list(combinations(range(1, len(self.ls_reference_ref_xy)+1), 2))
                        N_pairs = list(combinations(range(1, len(ls_detected_ref)+1), 2))

                        list_IDref_satify_fromd = []
                        for index, val in enumerate(valid_indices):
                            if val != -1:
                                list_IDref_satify_fromd.append([M_pairs[val], N_pairs[index]])

                        print("Các vị trí gương khớp cua xet khoang cach là:", list_IDref_satify_fromd)
                        print("---")

                        # - Tạo các khả năng gương có thể khớp dựa vào khoảng cách
                        # expanded_results = [pair for group in list_IDref_satify_fromd for pair in product(*group)]

                        N_angle = np.array(self.list_N_angle)
                        M_angle = np.array(self.list_M_angle)

                        # Tạo ma trận G_angle với |M_angle[i] - N_angle[j]|
                        G_angle = np.abs(M_angle[:, np.newaxis] - N_angle)
                        # In kích thước của ma trận
                        # print("Kích thước ma trận G_angle:", G_angle.shape)

                        # --- Step 2: Tìm giá trị nhỏ nhất của từng cột
                        # Thay giá trị 0 hoặc giá trị lớn hơn threshold bằng np.inf
                        G_filtered = np.where((G_angle == 0) | (G_angle >= self.angle_matching_error_threshold), np.inf, G_angle)

                        # Tìm giá trị nhỏ nhất trong từng cột
                        min_values = np.min(G_filtered, axis=0)

                        # Tìm chỉ số của giá trị nhỏ nhất trong từng cột
                        min_indices = np.argmin(G_filtered, axis=0)

                        # Nếu tất cả giá trị trong cột đều >= threshold hoặc bằng 0, đặt kết quả là -1
                        min_indices[min_values == np.inf] = -1

                        # So sánh với ngưỡng distance 
                        valid_indices = min_indices

                        # In kết quả
                        # print("Chỉ số các hàng thoả mãn là          :", valid_indices)

                        # Quy đổi các vị trí nhỏ nhất ra vị trí phát hiện
                        M_numbers = list(range(1, len(self.ls_reference_ref_xy) + 1))
                        N_numbers = list(range(1, len(ls_detected_ref) + 1))

                        # Tìm tất cả các tổ hợp chập 3
                        M_combinations = list(combinations(M_numbers, 3))
                        N_combinations = list(combinations(N_numbers, 3))

                        list_IDref_satify_fromtheta = []
                        for index, val in enumerate(valid_indices):
                            if val != -1:
                                list_IDref_satify_fromtheta.append([M_combinations[val], N_combinations[index]])

                        print("Các ID trên map thỏa mãn góc là :", list_IDref_satify_fromtheta)
                        print("-------")

                        # - find ID match between reference map and detected map
                        # - Step 1: Tìm trong list khoảng cách có phần từ nào giống trong list góc ko -> đưa ra Id trùng
                        ls_IDmatch = []
                        ls_IDmatch_notsure = []

                        for item_theta_new in list_IDref_satify_fromtheta:
                            # id_theta = item_theta_new[0]
                            id0_theta_ref = item_theta_new[0][0]        # 2
                            id1_theta_ref = item_theta_new[0][1]        # 3
                            id2_theta_ref = item_theta_new[0][2]        # 5

                            id0_theta_det = item_theta_new[1][0]        # 1
                            id1_theta_det = item_theta_new[1][1]        # 2
                            id2_theta_det = item_theta_new[1][2]        # 4

                            condition1 = False
                            condition11 = False
                            condition12 = False
                            condition2 = False
                            condition21 = False
                            condition22 = False

                            for item_d in list_IDref_satify_fromd:
                                id0_dis_ref = item_d[0][0]                 # 2
                                id1_dis_ref = item_d[0][1]                 # 3
                                id0_dis_det = item_d[1][0]                 # 1
                                id1_dis_det = item_d[1][1]                 # 4

                                # - case 1: tìm được cạnh 01 của góc tham chiếu trong list d tham chiếu 
                                if id0_theta_ref == id0_dis_ref and id1_theta_ref == id1_dis_ref:
                                    # subcase 1: tìm được cạnh 01 của góc detect trong list d phát hiện => tìm được 1 cạnh ok
                                    if id0_theta_det == id0_dis_det and id1_theta_det == id1_dis_det:
                                        condition1 = True
                                        condition11 = True 

                                    # subcase 2: tìm được cạnh 02 của góc detect trong list d phát hiện => tìm được 1 cạnh ok
                                    elif id0_theta_det == id0_dis_det and id2_theta_det == id1_dis_det:
                                        condition1 = True
                                        condition12 = True
                                    
                                # -- case 2: tìm được cạnh 02 cuả góc tham chiếu trong list d tham chiếu
                                if id0_theta_ref == id0_dis_ref and id2_theta_ref == id1_dis_ref:
                                    # subcase 1: tìm được cạnh 01 của góc detect trong list d phát hiện => tìm được 1 cạnh ok
                                    if id0_theta_det == id0_dis_det and id1_theta_det == id1_dis_det:
                                        condition2 = True
                                        condition21 = True

                                    # subcase 2: tìm được cạnh 02 của góc detect trong list d phát hiện => tìm được 1 cạnh ok
                                    elif id0_theta_det == id0_dis_det and id2_theta_det == id1_dis_det:
                                        condition2 = True
                                        condition22 = True

                            # Th1: các id này thoả mãn cả góc và 2 cạnh
                            if condition1 == True and condition2 == True:
                                if condition11 == True and condition12 == False and condition21 == False and condition22 == True:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id1_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id2_theta_det))
                                
                                elif condition11 == False and condition12 == True and condition21 == True and condition22 == False:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id2_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id1_theta_det))
                            
                            # th2: các id thoả mãn góc và 1 cạnh
                            elif condition1 == True and condition2 == False:
                                if condition11 == True and condition12 == False:
                                    ls_IDmatch_notsure.append([(id0_theta_ref, id0_theta_det), (id1_theta_ref, id1_theta_det), (id0_theta_ref, id1_theta_det), (id1_theta_ref, id0_theta_det)])
                                
                                elif condition11 == False and condition12 == True:
                                    ls_IDmatch_notsure.append([(id0_theta_ref, id0_theta_det), (id1_theta_ref, id2_theta_det), (id0_theta_ref, id2_theta_det), (id1_theta_ref, id0_theta_det)])

                            elif condition1 == False and condition2 == True:
                                if condition21 == True and condition22 == False:
                                    ls_IDmatch_notsure.append([(id0_theta_ref, id0_theta_det), (id2_theta_ref, id1_theta_det), (id0_theta_ref, id1_theta_det), (id2_theta_ref, id0_theta_det)])
                                
                                elif condition21 == False and condition22 == True:
                                    ls_IDmatch_notsure.append([(id0_theta_ref, id0_theta_det), (id2_theta_ref, id2_theta_det), (id0_theta_ref, id2_theta_det), (id2_theta_ref, id0_theta_det)])
                            
                        print("ID các điểm có thể khớp là", ls_IDmatch_notsure)
                        print("-------")
                        print("ID các chắc chắn khớp là", ls_IDmatch)
                        print("-------")

                        # Chuyển mỗi danh sách con thành tập hợp (set) để loại bỏ phần tử trùng trong cùng một danh sách con
                        # ls_IDmatch_notsure = [list(set(sublist)) for sublist in ls_IDmatch_notsure]

                        # Loại bỏ các danh sách con trùng nhau bằng cách dùng tập hợp
                        # ls_IDmatch_notsure = list(map(list, {frozenset(sublist) for sublist in ls_IDmatch_notsure}))
                        seen = set()
                        unique_lists = []
                        for sublist in ls_IDmatch_notsure:
                            frozen_sublist = frozenset(sublist)  # Chuyển danh sách con thành frozenset để kiểm tra trùng lặp
                            if frozen_sublist not in seen:
                                seen.add(frozen_sublist)
                                unique_lists.append(sublist)  # Giữ nguyên định dạng danh sách con
                        
                        ls_IDmatch_notsure = unique_lists

                        print("ID các điểm có thể khớp sau khi xoá chung là", ls_IDmatch_notsure)
                        print("-------")
                        # Trích xuất từng cặp phần tử con
                        sub_pairs = [item for sublist in ls_IDmatch_notsure for item in sublist]

                        # Đếm số lần xuất hiện của từng cặp phần tử con
                        sub_pair_counts = Counter(sub_pairs)

                        print("Số lượng các phần tử con là", sub_pair_counts)
                        print("-------")
                        # Tìm số lần xuất hiện lớn nhất
                        max_count = max([v for v in sub_pair_counts.values() if v > 1], default=0)

                        # Lấy danh sách tất cả các phần tử có số lần xuất hiện bằng `max_count`
                        most_common_elements = [key for key, value in sub_pair_counts.items() if value == max_count and max_count > 1]

                        print(f"Các phần tử xuất hiện nhiều nhất: {most_common_elements} - Số lần: {max_count}")
                        print("-------")
                        # -- announce status
                        self.finding_status = 0
                        
                        if len(ls_IDmatch) == 0:
                            n = len(most_common_elements)

                            if n == 0:
                                if len(ls_IDmatch_notsure) > 1: # -- when elements of sub_pair_counts equal 1
                                    # tạo các case từ dữ liệu ban đầu
                                    """
                                    Ví dụ: ID các điểm có thể khớp sau khi xoá chung là [[(1, 1), (5, 5), (1, 5), (5, 1)], [(2, 2), (3, 3), (2, 3), (3, 2)]]
                                    """
                                    ls_IDmatch_unit = []
                                    ls_IDmatch_temp = []

                                    for idi in range(0, len(ls_IDmatch_notsure)):
                                        element1 = ls_IDmatch_notsure[idi]
                                        for idj in range(idi, len(ls_IDmatch_notsure)):
                                            element2 = ls_IDmatch_notsure[idj]
                                            if idj != idi:
                                                # case 1
                                                ls_IDmatch_unit = [element1[0], element1[1], element2[0], element2[1]]
                                                # -- Xử lý lọc dữ liệu trùng trong từng phần tử con và sắp xếp theo phần tử số 2 của tupple
                                                id_shrink = sorted(list(set(ls_IDmatch_unit)), key=lambda x: x[1])
                                                ls_IDmatch_temp.append(id_shrink)  
                                                
                                                # -- case 2
                                                ls_IDmatch_unit = [element1[0], element1[1], element2[2], element2[3]]
                                                id_shrink = sorted(list(set(ls_IDmatch_unit)), key=lambda x: x[1])
                                                ls_IDmatch_temp.append(id_shrink)

                                                # -- case 3
                                                ls_IDmatch_unit = [element1[2], element1[3], element2[0], element2[1]]
                                                id_shrink = sorted(list(set(ls_IDmatch_unit)), key=lambda x: x[1])
                                                ls_IDmatch_temp.append(id_shrink)

                                                # -- case 4
                                                ls_IDmatch_unit = [element1[2], element1[3], element2[2], element2[3]]
                                                id_shrink = sorted(list(set(ls_IDmatch_unit)), key=lambda x: x[1])
                                                ls_IDmatch_temp.append(id_shrink)
                                    
                                    print("Tồn tại các id tương đối có quatity = 1 -> Các id khớp là", ls_IDmatch_temp)                            
                                    print("---------")
                                    # - tính các dữ liệu cần thiết cho từ cụm id khớp
                                    info_IDmatch_xr = []
                                    info_IDmatch_yr = []
                                    info_IDmatch_phir = []
                                    info_IDmatch_rmsexy = []
                                    info_IDmatch_rmsedr = []
                                    info_IDmatch_wstart = []
                                    info_IDmatch_globalID = []
                                    info_IDmatch_localID = []

                                    for idmatch_temp in ls_IDmatch_temp:
                                        xr, yr, phir, rmse_xy, rmse_dr, w_start, globalID, localID = self.cal_info_idmatch_in_initmode(idmatch_temp, ls_detected_ref, self.ls_reference_ref_xy, self.ls_reference_ref_dr)
                                        info_IDmatch_xr.append(xr)
                                        info_IDmatch_yr.append(yr)
                                        info_IDmatch_phir.append(phir)
                                        info_IDmatch_rmsexy.append(rmse_xy)
                                        info_IDmatch_rmsedr.append(rmse_dr)
                                        info_IDmatch_wstart.append(w_start)
                                        info_IDmatch_globalID.append(globalID)
                                        info_IDmatch_localID.append(localID)

                                    # find min of rmse_xy
                                    min_rmse_xy = min(info_IDmatch_rmsexy)
                                    print(f"Giá trị rmse tương đối ở case = 1, quantity = 2 là: {min_rmse_xy}")
                                    print("--------")
                                    self.rmse_xy = min_rmse_xy 
                                    if min_rmse_xy <= self.RMSE_THRESHOLD_INITMODE:
                                        ik = info_IDmatch_rmsexy.index(min_rmse_xy)
                                        
                                        self.x_tf = info_IDmatch_xr[ik]
                                        self.y_tf = info_IDmatch_yr[ik]
                                        self.r_tf = info_IDmatch_phir[ik]
                                        self.w_start = info_IDmatch_wstart[ik]
                                        self.ls_localID = info_IDmatch_localID[ik]

                                        # -- find globalID
                                        self.ls_globalID = []
                                        for id_ref in info_IDmatch_globalID[ik]:
                                            for i, ref in enumerate(self.ls_reference_ref_xy):
                                                if id_ref == i+1:
                                                    self.ls_globalID.append(ref[0])
                                        
                                        self.pre_w = self.w_start

                                        # -- move to next step
                                        self.completed_initmode = 1
                                        
                                        # -- reset varibles
                                        self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_ORIGIN
                                        self.angle_matching_error_threshold = self.ANGLE_THRESHOLD_ORIGIN 
                                        self.step = 0
                                        self.count_cal_initmode = 0

                                        # -- announce status
                                        self.finding_status = 1

                                        # --
                                        # self.rmse_start = self.rmse_xy * self.RMSE_COEF

                                        # -- 
                                        self.time_initmode_done = time.time() - self.time_start
                                        print(f"Thời gian khởi tạo là: {self.time_initmode_done}")

                                        print(f"Tìm được các giá trị: xr = {self.x_tf}, yr = {self.y_tf}, phir = {self.r_tf}, w_start = {self.w_start}")
                                        print("--------")
                                        # -- stop program for debug
                                        # self.flag_stop_program = 1
                                    
                                    else:
                                        self.count_cal_again_highrmse += 1
                                        # print(f"rms > {self.RMSE_THRESHOLD_INITMODE} => yêu cầu tính toán lại lần thứ {self.count_cal_again_highrmse} => sử dụng dữ liệu đầu vào khác")                                       
                                        # # -- receive new data
                                        # self.process = 1
                                        # self.step = 0

                                        if self.completed_navmode == 1:
                                            rospy.logwarn(f"Init fail -> Giá trị rmse khởi tạo > {self.RMSE_THRESHOLD_INITMODE} => chuyển sang cụm gương kế tiếp")
                                            
                                            self.range_end = self.ls_reference_ref_d[1]
                                            self.range_start = self.range_end - self.RANGE_FINDDING

                                            if self.range_start < 0:
                                                self.range_start = self.data_map.reflectors[0].Polar_Dist

                                            # -- change to next reference reflector cluster 
                                            # self.process = 1
                                            self.step = 1

                                        else:     # trường hợp fail ở navmode => update luôn new detected data
                                            self.process = 1
                                            self.step = 0
                                            self.count_cal_initmode += 1

                                            if self.count_cal_initmode > self.NUMBER_INIT_ERROR:
                                                self.count_cal_initmode = 0
                                                rospy.logerr("Ko có đủ gương cho AGV di chuyển => lắp thêm")
                                                self.flag_stop_program = 1

                                    """
                                        Result: Tồn tại id tuyệt đôí -> Các id khớp là ls_IDmatch_temp = [ [(1, 1), (3, 2), (2, 3)]  , [(2, 2), (3, 3), (1, 3)]  ]
                                    """        

                                else:
                                    self.distance_matching_error_threshold += self.DISTANCE_THRESHOLD_INCREMENT
                                    # self.count_cal_again_zeromatch += 1
                                    # print(f"Số gương khớp == 0 => yêu cầu tính toán lại lần thứ {self.count_cal_again_zeromatch} vơí hệ số khớp theo khoảng cách là {self.distance_matching_error_threshold} và hệ số theo góc là {self.angle_matching_error_threshold}")

                                    if self.distance_matching_error_threshold >= self.DISTANCE_THRESHOLD_MAX:
                                        self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_MAX 
                                        self.angle_matching_error_threshold += self.ANGLE_THRESHOLD_INCREMENT
                                    
                                    if self.angle_matching_error_threshold > self.ANGLE_THRESHOLD_MAX:
                                        # -- reset thres for new cluster
                                        self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_ORIGIN
                                        self.angle_matching_error_threshold = self.ANGLE_THRESHOLD_ORIGIN

                                        # -
                                        if self.completed_navmode == 1:
                                            rospy.logwarn("Init fail -> Ko tìm được gương khớp => chuyển sang cụm gương kế tiếp")                                            

                                            self.range_end = self.ls_reference_ref_d[1]
                                            self.range_start = self.range_end - self.RANGE_FINDDING

                                            if self.range_start < 0:
                                                self.range_start = self.data_map.reflectors[0].Polar_Dist

                                            # -- change to next reference reflector cluster 
                                            # self.process = 1
                                            self.step = 1

                                        else:     # trường hợp fail ở navmode => update luôn new detected data
                                            self.process = 1
                                            self.step = 0
                                            self.count_cal_initmode += 1

                                            if self.count_cal_initmode > self.NUMBER_INIT_ERROR:
                                                self.count_cal_initmode = 0
                                                rospy.logerr("Ko có đủ gương cho AGV di chuyển => lắp thêm")
                                                self.flag_stop_program = 1

                            else:
                                print("Có ít nhất 1 case có số lượng tối đa => check xem rmse có thoả mãn ko")
                                ls_IDmatch_temp = []
                                for most_common_element in most_common_elements:
                                    ls_IDmatch_unit = []
                                    for idmatch in ls_IDmatch_notsure:
                                        if (idmatch[0][0] == most_common_element[0] and idmatch[0][1] == most_common_element[1]) or (idmatch[1][0] == most_common_element[0] and idmatch[1][1] == most_common_element[1]):
                                            ls_IDmatch_unit.append((idmatch[0][0], idmatch[0][1]))
                                            ls_IDmatch_unit.append((idmatch[1][0], idmatch[1][1]))

                                        elif (idmatch[2][0] == most_common_element[0] and idmatch[2][1] == most_common_element[1]) or (idmatch[3][0] == most_common_element[0] and idmatch[3][1] == most_common_element[1]):
                                            ls_IDmatch_unit.append((idmatch[2][0], idmatch[2][1]))
                                            ls_IDmatch_unit.append((idmatch[3][0], idmatch[3][1]))

                                    # -- Xử lý lọc dữ liệu trùng trong từng phần tử con và sắp xếp theo phần tử số 2 của tupple
                                    id_shrink = sorted(list(set(ls_IDmatch_unit)), key=lambda x: x[1])
                                    print("Một trong nhưng cụm gương có thể là", id_shrink)

                                    ls_IDmatch_temp.append(id_shrink)

                                print("Tồn tại các id tương đối -> Các id khớp là", ls_IDmatch_temp)

                                # - tính các dữ liệu cần thiết cho từ cụm id khớp
                                info_IDmatch_xr = []
                                info_IDmatch_yr = []
                                info_IDmatch_phir = []
                                info_IDmatch_rmsexy = []
                                info_IDmatch_rmsedr = []
                                info_IDmatch_wstart = []
                                info_IDmatch_globalID = []
                                info_IDmatch_localID = []

                                for idmatch_temp in ls_IDmatch_temp:
                                    xr, yr, phir, rmse_xy, rmse_dr, w_start, globalID, localID = self.cal_info_idmatch_in_initmode(idmatch_temp, ls_detected_ref, self.ls_reference_ref_xy, self.ls_reference_ref_dr)
                                    info_IDmatch_xr.append(xr)
                                    info_IDmatch_yr.append(yr)
                                    info_IDmatch_phir.append(phir)
                                    info_IDmatch_rmsexy.append(rmse_xy)
                                    info_IDmatch_rmsedr.append(rmse_dr)
                                    info_IDmatch_wstart.append(w_start)
                                    info_IDmatch_globalID.append(globalID)
                                    info_IDmatch_localID.append(localID)

                                # find min of rmse_xy
                                min_rmse_xy = min(info_IDmatch_rmsexy)
                                print(f"Giá trị rmse tương đối ở case > 2 là: {min_rmse_xy}")
                                print("--------")

                                self.rmse_xy = min_rmse_xy 
                                if min_rmse_xy <= self.RMSE_THRESHOLD_INITMODE:
                                    ik = info_IDmatch_rmsexy.index(min_rmse_xy)
                                    
                                    self.x_tf = info_IDmatch_xr[ik]
                                    self.y_tf = info_IDmatch_yr[ik]
                                    self.r_tf = info_IDmatch_phir[ik]
                                    self.w_start = info_IDmatch_wstart[ik]
                                    self.ls_localID = info_IDmatch_localID[ik]
                                    
                                    # -- find globalID
                                    self.ls_globalID = []
                                    for id_ref in info_IDmatch_globalID[ik]:
                                        for i, ref in enumerate(self.ls_reference_ref_xy):
                                            if id_ref == i+1:
                                                self.ls_globalID.append(ref[0])
                                    
                                    self.pre_w = self.w_start

                                    # -- move to next step
                                    self.completed_initmode = 1
                                    
                                    # -- reset varibles
                                    self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_ORIGIN
                                    self.angle_matching_error_threshold = self.ANGLE_THRESHOLD_ORIGIN 
                                    self.step = 0
                                    self.count_cal_initmode = 0

                                    # -- announce status
                                    self.finding_status = 1

                                    # --
                                    # self.rmse_start = self.rmse_xy * self.RMSE_COEF

                                    # -- 
                                    self.time_initmode_done = time.time() - self.time_start
                                    print(f"Thời gian khởi tạo là: {self.time_initmode_done}")
                                    
                                    print(f"Tìm được các giá trị: xr = {self.x_tf}, yr = {self.y_tf}, phir = {self.r_tf}, w_start = {self.w_start}")

                                    # -- 
                                    # self.flag_stop_program = 1

                                else:
                                    self.count_cal_again_highrmse += 1
                                    # print(f"rms > {self.RMSE_THRESHOLD_INITMODE} => yêu cầu tính toán lại lần thứ {self.count_cal_again_highrmse} => sử dụng dữ liệu đầu vào khác")                                       
                                    # -- receive new data
                                    # self.process = 1
                                    # self.step = 0 

                                    if self.completed_navmode == 1:
                                        rospy.logwarn(f"Init fail -> Giá trị rmse khởi tạo > {self.RMSE_THRESHOLD_INITMODE} => chuyển sang cụm gương kế tiếp")
                                        
                                        self.range_end = self.ls_reference_ref_d[1]
                                        self.range_start = self.range_end - self.RANGE_FINDDING

                                        if self.range_start < 0:
                                            self.range_start = self.data_map.reflectors[0].Polar_Dist

                                        # -- change to next reference reflector cluster 
                                        # self.process = 1
                                        self.step = 1

                                    else:     # trường hợp fail ở navmode => update luôn new detected data
                                        self.process = 1
                                        self.step = 0
                                        self.count_cal_initmode += 1

                                        if self.count_cal_initmode > self.NUMBER_INIT_ERROR:
                                            self.count_cal_initmode = 0
                                            rospy.logerr("Ko có đủ gương cho AGV di chuyển => lắp thêm")
                                            self.flag_stop_program = 1

                                print("--------")
                        else:
                            # Loại bỏ phần tử trùng lặp và Sắp xếp theo phần tử thứ 2 của tuple
                            ls_IDresult = sorted(list(set(ls_IDmatch)), key=lambda x: x[1])
                            print("Tồn tại id tuyệt đôí -> Các id khớp là", ls_IDresult)

                            """
                            Tồn tại id tuyệt đôí -> Các id khớp là [(1, 1), (3, 2), (2, 3)]
                            """

                            # - tính các thông số
                            xr, yr, phir, rmse_xy, rmse_dr, w_start, globalID, localID = self.cal_info_idmatch_in_initmode(ls_IDresult, ls_detected_ref, self.ls_reference_ref_xy, self.ls_reference_ref_dr)

                            self.rmse_xy = rmse_xy 
                            print(f"Hệ số rmse cho case tuyệt đối là: {self.rmse_xy}")
                            print("--------")

                            if rmse_xy <= self.RMSE_THRESHOLD_INITMODE:
                                self.x_tf = xr
                                self.y_tf = yr
                                self.r_tf = phir
                                self.w_start = w_start
                                self.pre_w = w_start

                                self.ls_localID = localID

                                # -- find globalID
                                self.ls_globalID = []
                                for id_ref in globalID:
                                    for i, ref in enumerate(self.ls_reference_ref_xy):
                                        if id_ref == i+1:
                                            self.ls_globalID.append(ref[0])
                                
                                # -- move to next step
                                self.completed_initmode = 1
                                
                                # -- reset varibles
                                self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_ORIGIN
                                self.angle_matching_error_threshold = self.ANGLE_THRESHOLD_ORIGIN 
                                self.step = 0
                                self.count_cal_initmode = 0
                                print(f"Tìm được các giá trị: xr = {self.x_tf}, yr = {self.y_tf}, phir = {self.r_tf}, w_start = {self.w_start}")

                                # -- announce status
                                self.finding_status = 1

                                # - 
                                # self.rmse_start = self.rmse_xy * self.RMSE_COEF

                                # -- 
                                self.time_initmode_done = time.time() - self.time_start
                                print(f"Thời gian khởi tạo là: {self.time_initmode_done}")
                                
                                # -
                                # self.flag_stop_program = 1
                            
                            else:
                                self.count_cal_again_highrmse += 1
                                # print(f"rms > {self.RMSE_THRESHOLD_INITMODE} => yêu cầu tính toán lại lần thứ {self.count_cal_again_highrmse} => sử dụng dữ liệu đầu vào khác")
                                # # -- receive new data
                                # self.process = 1
                                # self.step = 0
                                if self.completed_navmode == 1:
                                    rospy.logwarn(f"Init fail -> Giá trị rmse khởi tạo > {self.RMSE_THRESHOLD_INITMODE} => chuyển sang cụm gương kế tiếp")
                                    
                                    self.range_end = self.ls_reference_ref_d[1]
                                    self.range_start = self.range_end - self.RANGE_FINDDING

                                    if self.range_start < 0:
                                        self.range_start = self.data_map.reflectors[0].Polar_Dist

                                    # -- change to next reference reflector cluster 
                                    # self.process = 1
                                    self.step = 1

                                else:     # trường hợp fail ở navmode => update luôn new detected data
                                    rospy.logwarn(f"Init fail -> Giá trị rmse tuyệt đối > {self.RMSE_THRESHOLD_INITMODE} => tìm kiếm dữ liệu khác ")
                                    self.process = 1
                                    self.step = 0
                                    self.count_cal_initmode += 1
                                    
                                    if self.count_cal_initmode > self.NUMBER_INIT_ERROR:
                                        self.count_cal_initmode = 0
                                        rospy.logerr("Ko có đủ gương cho AGV di chuyển => lắp thêm")
                                        self.flag_stop_program = 1
                            
                            print("--------")
                
                else:
                    self.process = 1

                # -- update data to ros
                # print(f"Hệ số rmse là: {self.rmse_xy}")
                self.ls_rmse[0] = self.rmse_xy
                self.ls_w[0] = self.distance_matching_error_threshold
                self.ls_w[1] = self.angle_matching_error_threshold
                self.ls_nomatch[0] = self.count_cal_again_zeromatch
                self.ls_samematch[0] = self.count_cal_again_samematch
                self.ls_highrmse[0] = self.count_cal_again_highrmse
                print("################# Process 2 ###################### ")

            # - Tìm ra vị trí robot sau khi đã tìm ra vị trí khởi tạo
            elif self.process == 3:
                if self.step == 0:        # tính toán các thông tin nhận được từ gương tham chiếu và gương phát hiện
                    # - Step 1: Biến đổi các gương phát hiện đươc trên map gốc + Áp dụng thuật toán bù chuyển động
                    # -- use vel from raw_vel
                    vx = self.data_rawvel.twist.twist.linear.x
                    vy = 0
                    thetaz = self.data_rawvel.twist.twist.angular.z

                    # vel lidar from robot vel
                    vlidar_x = vx - thetaz * self.DISTANCE_Y_BETWEEN_LIDAR_RB
                    vlidar_y = vy + thetaz * self.DISTANCE_X_BETWEEN_LIDAR_RB
                    vlidar_w = thetaz

                    # vel of lidar from rf2o laser odometry
                    # vlidar_x = self.data_odomRf2o.twist.twist.linear.x
                    # vlidar_y = self.data_odomRf2o.twist.twist.linear.y

                    # predict position after scanning period t s
                    x_predict = self.x_tf + vlidar_x*self.SCANNING_PERIOD
                    y_predict = self.y_tf + vlidar_y*self.SCANNING_PERIOD
                    r_predict = self.r_tf + vlidar_w*self.SCANNING_PERIOD

                    k = self.data_raw_reflector.num_reflector

                    ls_detected_ref = []
                    ls_detected_ref_normal = []
                    convert_posReflector = []
                    convert_posReflector_d = []
                    convert_posReflector_theta = []
                    convert_posReflector_d_predict = []

                    for i, ref in enumerate(self.data_raw_reflector.reflectors):

                        # -- pose of reflector after motion conpensation
                        # x_conpensate = ref.Cart_X - (fabs(vlidar_x))*(1 - (i+1)/k)
                        # y_conpensate = ref.Cart_Y - (fabs(vlidar_y))*(1 - (i+1)/k)

                        x_conpensate = ref.Cart_X - vlidar_x*(1 - (i+1)/k)
                        y_conpensate = ref.Cart_Y - vlidar_y*(1 - (i+1)/k)

                        # -- create ls compensation ref in x, y >> for find lidar from svd
                        ls_detected_ref.append([ref.LocalID, x_conpensate, y_conpensate])

                        # -- create ls compensation ref in x, y  >> send data to rviz
                        ls_detected_ref_normal.append([ref.LocalID, ref.Cart_X, ref.Cart_Y])

                        # -- create list compensation ref with d and theta in global map
                        x_cv, y_cv = self.calculate_reflector_position_in_map(self.x_tf, self.y_tf, self.r_tf, x_conpensate, y_conpensate)
                        d_raw = sqrt(x_cv*x_cv + y_cv*y_cv)
                        theta_raw = atan2(y_cv, x_cv)

                        convert_posReflector.append([ref.LocalID, d_raw, theta_raw])
                        convert_posReflector_d.append(d_raw)
                        convert_posReflector_theta.append(theta_raw) 

                        # -- create list predict ref with d and theta in global map
                        x_cv_predict, y_cv_predict = self.calculate_reflector_position_in_map(x_predict, y_predict, r_predict, x_conpensate, y_conpensate)
                        d_predict = sqrt(x_cv_predict*x_cv_predict + y_cv_predict*y_cv_predict)
                        convert_posReflector_d_predict.append(d_predict)
                    

                    # -- arrange data in order of distance for matching
                    ls_d = []
                    for d in convert_posReflector_d:
                        ls_d.append(d)
                    
                    sorted_values, sorted_indices = self.sort_list_with_indices(ls_d)

                    lsReflector_arrange = []
                    lsReflector_arrange_d = []
                    lsReflector_arrange_theta = []

                    ls_detected_ref_arrange = []

                    ls_poseReflector_predict = []
                    ls_poseReflector_normal = []
                    
                    for j, indice in enumerate(sorted_indices):
                        for i, d in enumerate(convert_posReflector_d):
                            if i == indice:

                                ls_detected_ref_arrange.append([j+1, ls_detected_ref[i][1], ls_detected_ref[i][2]])

                                lsReflector_arrange.append(convert_posReflector[i])
                                lsReflector_arrange_d.append(d)
                                lsReflector_arrange_theta.append(convert_posReflector_theta[i])

                                ls_poseReflector_predict.append([ls_detected_ref[i][1], ls_detected_ref[i][2], 0.0])
                                ls_poseReflector_normal.append([ls_detected_ref_normal[i][1], ls_detected_ref_normal[i][2], 0.0])

                    # - update reflector info
                    ls_detected_ref = ls_detected_ref_arrange
                    convert_posReflector = lsReflector_arrange
                    convert_posReflector_d = lsReflector_arrange_d
                    convert_posReflector_theta = lsReflector_arrange_theta

                    # for i in range(len(convert_posReflector)):
                    #     print(f"Gương phát hiện thứ {convert_posReflector[i][0]} có thông số là: d = {convert_posReflector[i][1]}, theta = {convert_posReflector[i][2]}")
                    # print("\n")
                    # for i in range(len(self.ls_reference_ref_xy)):
                    #     print(f"Gương tham chiếu thứ {self.ls_reference_ref_xy[i][0]} có thông số là: d = {self.ls_reference_ref_d[i]}, theta = {self.ls_reference_ref_theta[i]}")
                    # print("--------------")                    

                    # for i in range(len(ls_detected_ref)):
                    #     print(f"Gương phát hiện thứ {ls_detected_ref[i][0]} có thông số là: x = {ls_detected_ref[i][1]}, y = {ls_detected_ref[i][2]}")
                    # print("\n")
                    # for i in range(len(self.ls_reference_ref_xy)):
                    #     print(f"Gương tham chiếu thứ {self.ls_reference_ref_xy[i][0]} có thông số là: x = {self.ls_reference_ref_xy[i][1]}, y = {self.ls_reference_ref_xy[i][2]}")
                    # print("--------------")

                    # -  Lọc gương tham chiếu trong khoảng gương phát hiện được
                    # print(f"list d là: {convert_posReflector_d}")
                    # print("-----")

                    # -- find range for finđing pose again after failure mathching
                    self.d_lower = min(convert_posReflector_d)
                    self.d_upper = max(convert_posReflector_d)

                    # --
                    d_near = min(convert_posReflector_d_predict)
                    d_far = max(convert_posReflector_d_predict)

                    d_near = (1-self.DISTANCE_COEF_LOWER)*d_near
                    d_far = (1+self.DISTANCE_COEF_UPPER)*d_far

                    print(f"khoảng cách dnear = {d_near}, dfar = {d_far}")
                    print("-----")
                    ls_reference_ref_xy_filter = []
                    ls_reference_ref_d_filter = []
                    ls_reference_ref_theta_filter = []
                    ls_reference_ref_dr_filter = []

                    for i in range(0, len(self.ls_reference_ref_d_full)):
                        d_current = self.ls_reference_ref_d_full[i]

                        if d_near <= d_current <= d_far:
                            ls_reference_ref_dr_filter.append(self.ls_reference_ref_dr_full[i])
                            ls_reference_ref_d_filter.append(d_current)
                            ls_reference_ref_theta_filter.append(self.ls_reference_ref_theta_full[i])
                            ls_reference_ref_xy_filter.append(self.ls_reference_ref_xy_full[i])

                    # for i in range(len(convert_posReflector)):
                    #     print(f"Gương phát hiện thứ {convert_posReflector[i][0]} có thông số là: d = {convert_posReflector[i][1]}, theta = {convert_posReflector[i][2]}")
                    # print("\n")
                    # for i in range(len(self.ls_reference_ref_xy)):
                    #     print(f"Gương tham chiếu thứ {self.ls_reference_ref_xy[i][0]} có thông số là: d = {self.ls_reference_ref_d[i]}, theta = {self.ls_reference_ref_theta[i]}")
                    # print("--------------")                    

                    # for i in range(len(ls_detected_ref)):
                    #     print(f"Gương phát hiện thứ {ls_detected_ref[i][0]} có thông số là: x = {ls_detected_ref[i][1]}, y = {ls_detected_ref[i][2]}")
                    # print("\n")
                    # for i in range(len(ls_reference_ref_xy_filter)):
                    #     print(f"Gương tham chiếu thứ {ls_reference_ref_xy_filter[i][0]} có thông số là: x = {ls_reference_ref_xy_filter[i][1]}, y = {ls_reference_ref_xy_filter[i][2]}")
                    # print("--------------")

                    # - Step 2: Tìm ma trận trọng số w giữa map gương tham chiếu và map gương phát hiện
                    D_n = np.array(convert_posReflector_d)  # n = 3
                    D_m = np.array(ls_reference_ref_d_filter)  # m = 4

                    # Giả sử góc của các gương detected và tham chiếu (đơn vị: radian)
                    A_n = np.array(convert_posReflector_theta)  # n = 3
                    A_m = np.array(ls_reference_ref_theta_filter)  # m = 4

                    # Tạo ma trận sai số khoảng cách và góc bằng cách mở rộng (broadcasting)
                    sigma_d = D_n.reshape(1, -1) - D_m.reshape(-1, 1)  # Ma trận m x n
                    sigma_a = A_n.reshape(1, -1) - A_m.reshape(-1, 1)  # Ma trận m x n

                    # Tính trọng số w
                    w = np.abs(sigma_d * sigma_a)         

                    # In kết quả
                    # print("Ma trận sai số khoảng cách (sigma_d):\n", sigma_d)
                    # print("Ma trận sai số góc (sigma_a):\n", sigma_a)
                    # print("Ma trận trọng số (w):\n", w)

                    # Đặt ngưỡng w_sigma
                    # v_lidar = sqrt(vlidar_x*vlidar_x + vlidar_y*vlidar_y)
                    w_sigma = self.w_start + self.VEL_COEF*abs(vx) + self.OMEGA_COEF*abs(thetaz)     # sai số khoảng cách * sai số góc 
                    if vx > 0.013:    # robot đi tiến
                        if w_sigma > 0.2:
                            w_sigma = 0.2

                    elif vx < -0.013:
                        if w_sigma > 0.0075:
                            w_sigma = 0.0075
                    else:  
                        if abs(thetaz) > 0.05: # robot quay
                            if w_sigma > 0.08:
                                w_sigma = 0.08
                        
                        else:       # robot gần như đứng yên
                            if w_sigma > 0.02:
                                w_sigma = 0.02
                    
                    print(f"Vận tốc hiện tại của robot: {vx} và vt quay là {thetaz}. Trọng số: {w_sigma}")
                    print("-----")
                    # Tìm giá trị nhỏ nhất tại mỗi cột
                    min_values = np.min(w, axis=0)

                    # Tìm chỉ số hàng tương ứng với giá trị nhỏ nhất tại mỗi cột
                    min_indices = np.argmin(w, axis=0)

                    # So sánh với ngưỡng w_sigma
                    valid_indices = np.where(min_values < w_sigma, min_indices, -1)

                    # In kết quả
                    print("Giá trị nhỏ nhất tại mỗi cột:", min_values)
                    print("-----")
                    print("Chỉ số hàng tương ứng (hoặc -1 nếu không đạt ngưỡng):", valid_indices)
                    print("-----")

                    # -- xử lý khi bị có trường hợp các id trùng nhau 
                    is_not_uniqueID = False
                    for i in range(0, len(valid_indices)):
                        for j in range(i, len(valid_indices)):
                            if j != i:
                                idj_ref = valid_indices[j]
                                idi_ref = valid_indices[i]
                                minj_val = min_values[j]
                                mini_val = min_values[i]

                                if idj_ref == idi_ref:
                                    if minj_val < mini_val:
                                        valid_indices[i] = -1
                                    elif minj_val > mini_val:
                                        valid_indices[j] = -1

                    print("Chỉ số hàng tương ứng sau khi loại bỏ id trùng :", valid_indices)
                    print("-----")

                    self.step = 1
                    self.count_check_rmse = 0
                
                elif self.step == 1:

                    # -- publish data
                    ls_localID = []
                    ls_globalID = []
                    for i, id_ref in enumerate(valid_indices):
                        if id_ref != -1:
                            ls_globalID.append(id_ref+1)
                            ls_localID.append(i+1)

                    self.finding_status = 0

                    if (len(ls_localID) < 3):
                        self.count_zeromatch += 1
                        rospy.logwarn("Số gương khớp nhỏ hơn 3 => navigation mode fail => init again")
                        # -- Tạm thời dừng chương trình để đánh giá
                        self.completed_initmode = 0
                        self.completed_navmode = 0
                        # - 
                        self.ls_nomatch[1] = self.count_zeromatch

                        # -- write data to log file
                        self.log_r2000_data_before(-1, 1, self.pre_detected_ref_xy, self.pre_detected_ref_dr, self.pre_dnear, self.pre_dfar, self.pre_reference_ref_xy, self.pre_reference_ref_dr, self.pre_vx, self.pre_thetaz, self.pre_wsigma, self.pre_min_values, self.pre_valid_indices, 0, 0, 0)
                        self.log_r2000_data(-1, 1, ls_detected_ref, convert_posReflector, d_near, d_far, ls_reference_ref_xy_filter, ls_reference_ref_dr_filter, vx, thetaz, w_sigma, min_values, valid_indices, 0, 0, 0)

                    else:
                        xr, yr, phir, self.rmse_xy, rmse_dr, w_start = self.cal_info_idmatch_in_navmode(valid_indices, ls_detected_ref, ls_reference_ref_xy_filter, ls_reference_ref_dr_filter)
                        print("Hệ số rmse là: ", self.rmse_xy)
                        print("-------")

                        # -- 
                        self.RMSE_THRESHOLD_NAVMODE = self.rmse_start + self.RMSE_VEL_COEF*abs(vx) + self.RMSE_OMEGA_COEF*abs(thetaz)

                        if vx > 0.013:    # robot đi tiến
                            if self.RMSE_THRESHOLD_NAVMODE > self.RMSE_FORWARD_THRESHOLD_MAX:
                                self.RMSE_THRESHOLD_NAVMODE = self.RMSE_FORWARD_THRESHOLD_MAX 

                        elif vx < -0.013:
                            if self.RMSE_THRESHOLD_NAVMODE > self.RMSE_BACKWARD_THRESHOLD_MAX:
                                self.RMSE_THRESHOLD_NAVMODE = self.RMSE_BACKWARD_THRESHOLD_MAX
                        else:  
                            if abs(thetaz) > 0.05: # robot quay
                                if self.RMSE_THRESHOLD_NAVMODE > self.RMSE_SPIN_THRESHOLD_MAX:
                                    self.RMSE_THRESHOLD_NAVMODE = self.RMSE_SPIN_THRESHOLD_MAX
                            
                            else:       # robot gần như đứng yên
                                if self.RMSE_THRESHOLD_NAVMODE > 0.05:
                                    self.RMSE_THRESHOLD_NAVMODE = 0.05

                        print(f"Trọng số ngưỡng cho rmse hiện tại là: {self.RMSE_THRESHOLD_NAVMODE}")
                        print("-------")
                        
                        # -- save to csv file
                        # self.save_csv_file(vx, vy, thetaz, rmse_xy, self.RMSE_THRESHOLD_NAVMODE, self.dir_csv_file_xy)
                        # self.save_csv_file(vx, vy, thetaz, w_start, w_sigma, self.dir_csv_file_dr)

                        # -- 

                        if self.rmse_xy <= self.RMSE_THRESHOLD_NAVMODE:
                            print("-- Matching OK in navigation mode ")

                            # update weight data
                            self.w_start = w_start
                            self.ls_rmse[1] = self.rmse_xy

                            if self.w_start < self.pre_w:
                                self.w_start = self.pre_w
                                
                            else:
                                self.pre_w = self.w_start

                            # -- announce success
                            self.finding_status = 1
                            self.completed_navmode = 1

                            # -- update tranform data for next step
                            self.x_tf = xr
                            self.y_tf = yr
                            self.r_tf = phir

                            # -- 
                            self.ls_localID = ls_localID
                            self.ls_globalID = ls_globalID

                            # -- CREATE previous param which run ok
                            self.pre_detected_ref_xy = ls_detected_ref
                            self.pre_detected_ref_dr = convert_posReflector
                            self.pre_dnear = d_near 
                            self.pre_dfar = d_far
                            self.pre_reference_ref_xy = ls_reference_ref_xy_filter
                            self.pre_reference_ref_dr = ls_reference_ref_dr_filter
                            self.pre_vx = vx
                            self.pre_thetaz = thetaz
                            self.pre_wsigma = w_sigma
                            self.pre_min_values = min_values
                            self.pre_valid_indices = valid_indices

                            self.pre_rmse = self.rmse_xy
                            self.pre_rmse_threshold = self.RMSE_THRESHOLD_NAVMODE
                            self.pre_rmse_start = self.rmse_start

                            # -- reset for new data
                            self.step = 0
                            self.process = 1

                            # -- 
                            self.count_highrmse_navmode_when_spin = 0

                            # - publish pose of reflector after compensate
                            self.pub_marker(self.marker_normal, ls_poseReflector_normal, [0,1,0], 0.5, 0.0)
                            self.pub_marker(self.marker_predict, ls_poseReflector_predict, [1,0,0], 0.5, -0.6)

                        else:

                            if vx > 0.013:    # robot đi tiến
                                self.count_highrmse_navmode_when_forward += 1
                                rospy.logwarn(f"-- Navigation mode is fail when move forward -> back to init mode --, tính toán lại lần thứ {self.count_highrmse_navmode_when_forward}") 

                                self.completed_initmode = 0
                                self.completed_navmode = 0

                                # -- init again, update new data
                                self.step = 0
                                self.process = 1

                                self.log_r2000_data_before(1, 2, self.pre_detected_ref_xy, self.pre_detected_ref_dr, self.pre_dnear, self.pre_dfar, self.pre_reference_ref_xy, self.pre_reference_ref_dr, self.pre_vx, self.pre_thetaz, self.pre_wsigma, self.pre_min_values, self.pre_valid_indices, self.pre_rmse, self.pre_rmse_threshold, self.pre_rmse_start)
                                self.log_r2000_data(1, 2, ls_detected_ref, convert_posReflector, d_near, d_far, ls_reference_ref_xy_filter, ls_reference_ref_dr_filter, vx, thetaz, w_sigma, min_values, valid_indices, self.rmse_xy, self.RMSE_THRESHOLD_NAVMODE, self.rmse_start)  
                                    

                            elif vx < -0.013:   # robot đi lùi
                                self.count_highrmse_navmode_when_backward += 1
                                rospy.logwarn(f"-- Navigation mode is fail when move backward -> back to init mode --, tính toán lại lần thứ {self.count_highrmse_navmode_when_backward}") 

                                self.completed_initmode = 0
                                self.completed_navmode = 0

                                # -- init again, update new data
                                self.step = 0
                                self.process = 1


                                self.log_r2000_data_before(2, 2, self.pre_detected_ref_xy, self.pre_detected_ref_dr, self.pre_dnear, self.pre_dfar, self.pre_reference_ref_xy, self.pre_reference_ref_dr, self.pre_vx, self.pre_thetaz, self.pre_wsigma, self.pre_min_values, self.pre_valid_indices, self.pre_rmse, self.pre_rmse_threshold, self.pre_rmse_start)
                                self.log_r2000_data(2, 2, ls_detected_ref, convert_posReflector, d_near, d_far, ls_reference_ref_xy_filter, ls_reference_ref_dr_filter, vx, thetaz, w_sigma, min_values, valid_indices, self.rmse_xy, self.RMSE_THRESHOLD_NAVMODE, self.rmse_start)  
                                    

                            else:
                                if abs(thetaz) > 0.05: # robot quay
                                    if self.rmse_xy >= self.RMSE_SPIN_THRESHOLD_MAX:
                                        # -- Xử lý trường hợp high rmse do vị trí gương khớp bị sai khi robot đang ở mode quay nha
                                        """
                                            Ví dụ case đúng trước đó: [ 0  1  2  3  4  7  9 14], có rmse là 0.13093619779284926 < ngưỡng: 0.2
                                        case hiện tại sai: [ 0  1  2  3  4  7  9 12], có rmse là: 2.180536150500932 > ngưỡng: 0.2
                                            => chỉ lấy phần tử giống của case hiện tại 
                                        """
                                        print("Chỉ số hàng trước đó :", self.pre_valid_indices)
                                        print("-----")

                                        print("Chỉ số hàng hiện tại đó :", valid_indices)
                                        print("-----")

                                        B_modified = [b if b in self.pre_valid_indices else -1 for b in valid_indices]
                                        valid_indices = B_modified

                                        print("Chỉ số hàng sau chỉnh sửa lại :", valid_indices)
                                        print("-----") 

                                        self.count_highrmse_navmode_when_spin += 1
                                        rospy.logwarn(f"-- Navigation mode is fail when spin -> back to init mode --, tính toán lại lần thứ {self.count_highrmse_navmode_when_spin}") 

                                        if self.count_highrmse_navmode_when_spin >= 2:
                                            self.completed_initmode = 0
                                            self.completed_navmode = 0

                                            # -- init again, update new data
                                            self.step = 0
                                            self.process = 1
                                    else:
                                        print("-- Matching OK in navigation mode after clearing ref id error")

                                        # update weight data
                                        self.w_start = w_start
                                        self.ls_rmse[1] = self.rmse_xy

                                        if self.w_start < self.pre_w:
                                            self.w_start = self.pre_w
                                            
                                        else:
                                            self.pre_w = self.w_start

                                        # -- announce success
                                        self.finding_status = 1
                                        self.completed_navmode = 1

                                        # -- update tranform data for next step
                                        self.x_tf = xr
                                        self.y_tf = yr
                                        self.r_tf = phir

                                        # -- 
                                        self.ls_localID = ls_localID
                                        self.ls_globalID = ls_globalID

                                        # -- CREATE previous param which run ok
                                        self.pre_detected_ref_xy = ls_detected_ref
                                        self.pre_detected_ref_dr = convert_posReflector
                                        self.pre_dnear = d_near 
                                        self.pre_dfar = d_far
                                        self.pre_reference_ref_xy = ls_reference_ref_xy_filter
                                        self.pre_reference_ref_dr = ls_reference_ref_dr_filter
                                        self.pre_vx = vx
                                        self.pre_thetaz = thetaz
                                        self.pre_wsigma = w_sigma
                                        self.pre_min_values = min_values
                                        self.pre_valid_indices = valid_indices

                                        self.pre_rmse = self.rmse_xy
                                        self.pre_rmse_threshold = self.RMSE_THRESHOLD_NAVMODE
                                        self.pre_rmse_start = self.rmse_start

                                        # -- reset for new data
                                        self.step = 0
                                        self.process = 1

                                        # -- 
                                        self.count_highrmse_navmode_when_spin = 0                               

                                        # - publish pose of reflector after compensate
                                        self.pub_marker(self.marker_normal, ls_poseReflector_normal, [0,1,0], 0.5, 0.0)
                                        self.pub_marker(self.marker_predict, ls_poseReflector_predict, [1,0,0], 0.5, -0.6)

                                else:       # robot gần như đứng yên
                                    self.log_r2000_data_before(0, 2, self.pre_detected_ref_xy, self.pre_detected_ref_dr, self.pre_dnear, self.pre_dfar, self.pre_reference_ref_xy, self.pre_reference_ref_dr, self.pre_vx, self.pre_thetaz, self.pre_wsigma, self.pre_min_values, self.pre_valid_indices, self.pre_rmse, self.pre_rmse_threshold, self.pre_rmse_start)
                                    self.log_r2000_data(0, 2, ls_detected_ref, convert_posReflector, d_near, d_far, ls_reference_ref_xy_filter, ls_reference_ref_dr_filter, vx, thetaz, w_sigma, min_values, valid_indices, self.rmse_xy, self.RMSE_THRESHOLD_NAVMODE, self.rmse_start)  
                                    
                                    self.count_highrmse_navmode_when_standtill += 1
                                    rospy.logwarn(f"-- Navigation mode is fail when is standtill -> back to init mode --, tính toán lại lần thứ {self.count_highrmse_navmode_when_standtill}") 

                                    self.completed_initmode = 0
                                    self.completed_navmode = 0

                                    # -- init again, update new data
                                    self.step = 0
                                    self.process = 1                          

                            # # -- write data to log file
                            # self.log_r2000_data_before(2, self.pre_detected_ref_xy, self.pre_detected_ref_dr, self.pre_dnear, self.pre_dfar, self.pre_reference_ref_xy, self.pre_reference_ref_dr, self.pre_vx, self.pre_thetaz, self.pre_wsigma, self.pre_min_values, self.pre_valid_indices, self.pre_rmse, self.pre_rmse_threshold, self.pre_rmse_start)
                            # self.log_r2000_data(2, ls_detected_ref, convert_posReflector, d_near, d_far, ls_reference_ref_xy_filter, ls_reference_ref_dr_filter, vx, thetaz, w_sigma, min_values, valid_indices, self.rmse_xy, self.RMSE_THRESHOLD_NAVMODE, self.rmse_start)

                
                # - update new pose of reflector
                print("################# Process 3 ###################### ")

            # send transform from map to r2000_frame
            if self.process > 0:
                if self.finding_status == 1:                # đã tìm kiếm được vị trí
                    self.translation = (self.x_tf, self.y_tf, 0.0)
                    self.quanternion = self.euler_to_quaternion(self.r_tf)
                    self.br.sendTransform(self.translation, self.quanternion, rospy.Time.now(), self.ref_frame, self.origin_frame)

                    self.finding_status = 0

                # publish to ros
                self.pub_lidar_pose_full(self.lidar_pose_pub, self.ls_rmse, self.ls_w, self.ls_nomatch, self.ls_samematch, self.ls_highrmse, self.x_tf, self.y_tf, self.r_tf, self.ls_localID, self.ls_globalID, self.w_start, self.process, len(self.ls_localID))

            self.rate.sleep()

def main():
    print ("--- Run navigation mode---")
    program = Navigation_mode()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




