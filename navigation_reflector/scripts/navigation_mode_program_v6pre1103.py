#!/usr/bin/env python3

"""
*** infomation

*** Task Description: 
    + Tìm ra vị trí cuả robot trên map
    + Cải tiến thuật toán khớp gương: 
        -> sử dụng góc, khoảng cách theo tài liệu: X 
        -> thuật toán Ransac: 0
        -> Thuật toán Xác suất: 0
        -> Thuật toán Query Ball Point: 0

    + sử dụng dnear, dfar. Test full map : OK
    + Thêm chọn lọc góc tính toán trong chế độ init mode : từ 5-> 175 độ mới tính : OK
    + Sử dụng trọng số động cho chế độ init mode
        - Trường hợp nếu có ID bị trùng thì => Giảm ngưỡng xuống để tìm kiếm lại
        - Trường hợp không tìm ra ID nào thì tăng ngưỡng khoảng cách lên để tìm
        - Nêú 2 trọng số đều tăng tới tới hạn => báo lôĩ mất gương lên hệ thống
    
    + add 8/3/2025
        - Bỏ check rmse trong chế độ initmode
            + Tăng ngưỡng lên 10 
        - Tăng khoảng tìm kiếm trong kc
            + Nhân hệ số 1.2
    
    + Tại chế độ khởi động : ADD 8/3/2025
        - sử dụng 1 dữ liệu đồng nhâts khi tìm gương khớp => nếu ko detect được thì sưả hệ số khớp: OK
        - Khi tìm được vị trí thì sẽ setup lại các trọng số như ban đầu

        - Thêm thuật toán Ransac (ko hẳn) để khởi tạo vị trí của robot   ---> ko hiệu quả, xoá, nhưng cứ để tạm vì có thể sẽ dùng
            + Case 1: có 4 điểm gương khớp: 2 cặp trùng nhau: 1 cặp ref và 1 cặp detect

    - add 10/3/2025
        + Các điểm được xếp theo thứ tự từ bé tới lớn theo khoảng cách, tại map gương tham chiếu
            - sửa map gương gốc cuả nav350
            - sửa chương trình add gương vào map gốc

        + Lấy trung bình 5 lần toạ độ trước khi publísh ra ==> có smooth hơn nhưng sai số nhiều hơn==> fail,  skip
        + Tìm toạ độ tới đâu thì pub tới đó 
        + Thêm trường hợp khớp map tại chế độ init mode

    - add 11/3/2025
        + sửa khoảng giới hạn dựa vào predict position: 
            - x_predict = x + vx*t
            - y_predict = y + vy*t
            - r_predict = r + w*t 
        + Thêm hệ số cho trọng số w_sigma khi robot quay
        + Khắc phục lôĩ robot di chuyển ko mượt
            -> Khi di chuyển điểm thường => pub dữ liệu liên tục
            -> Khi di chuyển lùi => yêu cầu rmse thấp, khi dữ liệu ok ms cho robot chạy

*** Need to do
    + Vấn đề gương ảo ???? Lôĩ gương nhỏ hơn 3 thì sao ????
    + Mục tiêu vận tốc >= 0.8 m/s

*** Mô tả về chỉ số RMS; Trung bình sai số của các gương
    + 1 -> 10: Good, ví dụ
        RMSE cặp gương 1: 7.148799411056392
        RMSE cặp gương 2: 7.149230592798687
        RMSE cặp gương 3: 0.006011189494467315
        RMSE cặp gương 4: 0.0037423415317134274
    + 10 -> 100: medium
    + 100 -> 500: low
    + > 500: not OK
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

class reflectorMap():
    def __init__(self, _id = 0, _x = 0., _y = 0.):
        self.id = _id
        self.x = _x
        self.y = _y

class Navigation_mode():
    def __init__(self):
        rospy.init_node('navigation_node', anonymous = True)
        self.rate = rospy.Rate(50)

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

        # self.marker_pub_predict = rospy.Publisher('/visualization_marker_predict', Marker, queue_size=10)
        # self.marker_pub_matching = rospy.Publisher('/visualization_marker_matching', Marker, queue_size=10)

        self.lidar_pose_pub = rospy.Publisher('/r2000_data', R2000_data, queue_size=10)
        # self.lidar_pose_data = R2000_data()

        # -- node subcribe -- 
        rospy.Subscriber('NN_infoRespond' , NN_infoRespond, self.callBack_NNinfoRespond)
        self.data_NNinfoRespond = NN_infoRespond()
        self.is_recv_NNinfo = False

        # -- Constant varibles
        self.SCANNING_FREQUENCY = 11 #Hz
        self.SCANNING_PERIOD = 1/self.SCANNING_FREQUENCY

        self.DISTANCE_X_BETWEEN_LIDAR_RB = 0.404 #m
        self.DISTANCE_Y_BETWEEN_LIDAR_RB = 0.0 #m
        # self.ANGLE_BETWEEN_LIDAR_RB = 0  # rad

        # -- global variables
        self.process = 0
        self.pre_mess = ''
        self.ls_predict_ref = []

        # # -- Khai báo tf -- 
        self.br = tf.TransformBroadcaster()
        self.ref_frame = "world"
        self.origin_frame = "frame_map_nav350"

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
        self.DISTANCE_THRESHOLD_MAX = 0.06
        self.DISTANCE_THRESHOLD_ORIGIN = 0.02     # before 0.01
        self.DISTANCE_THRESHOLD_INCREMENT = 0.002
        self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_ORIGIN  #  (zf = 2 cm ), do sai số vị trí các gương là +- 0.005 m => 0,014 m

        self.ANGLE_THRESHOLD_MAX = 0.06
        self.ANGLE_THRESHOLD_ORIGIN = 0.02
        self.ANGLE_THRESHOLD_INCREMENT = 0.01
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

        self.pre_detected_ref = 0
        self.ANGLE_TRANSLATE = PI

        self.ANGLE_LIMIT_LOWER = 5
        self.ANGLE_LIMIT_UPPER = 175 
        self.pre_phiR = 0
        self.step = 0

        self.x_tf = 0.0
        self.y_tf = 0.0
        self.r_tf = 0.0

        self.ls_reference_ref_dr = []
        self.ls_reference_ref_d = []
        self.ls_reference_ref_theta = []
        self.ls_reference_ref_xy = []
        # self.ls_dectected_ref = []
        self.count_cal_again_zeromatch = 0
        self.count_cal_again_samematch = 0
        self.count_cal_again_highrmse = 0
        self.rmse_xy = 0                  # chỉ số đánh giá
        self.rmse_dr = []
        self.w_start = 0.0
        self.WEIGHT_COEF = 25.0
        self.VEL_COEF = 0.02
        self.OMEGA_COEF = 0.01
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
        self.count_highrmse = 0

        self.RMSE_THRESHOLD_INITMODE = 10              # before: 0.1
        self.RMSE_THRESHOLD_NAVMODE = 0.1
        self.DISTANCE_COEF = 0.0

        self.collision_ref_case = 0
        self.collision_det_case = 0

        self.ls_IDransac = 0
        self.ls_info_detref = 0

        self.completed_initmode = 0
        self.completed_navmode = 0

        self.count_result = 0
        self.sum_tfx = 0
        self.sum_tfy = 0
        self.sum_tfr = 0
        self.mean_tfx = 0
        self.mean_tfy = 0
        self.mean_tfr = 0

        self.NUMBER_COUNT_RESULT = 5

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
    def callBack_NNinfoRespond(self, data):
        self.data_NNinfoRespond = data
        self.is_recv_NNinfo = True

    def euler_to_quaternion(self, euler):
        quat = Quaternion()
        odom_quat = quaternion_from_euler(0, 0, euler)
        quat.x = odom_quat[0]
        quat.y = odom_quat[1]
        quat.z = odom_quat[2]
        quat.w = odom_quat[3]
        return odom_quat

    def pub_marker(self, marker_pub, list_point, r, g, b):
        # Tạo Marker kiểu SPHERE_LIST để hiển thị danh sách các điểm tâm
        marker = Marker()

        # Đặt các thuộc tính cơ bản cho Marker
        marker.header.frame_id = 'scanner_link'  # Frame tham chiếu, ví dụ: "world" hoặc "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "circle_centers"
        marker.id = 0  # ID của marker
        marker.type = Marker.SPHERE_LIST  # Sử dụng SPHERE_LIST để hiển thị nhiều điểm
        marker.action = Marker.ADD  # Thao tác: thêm vào hiển thị

        # Kích thước của các hình cầu (tất cả các điểm tâm sẽ có cùng kích thước)
        marker.scale.x = 0.2  # Bán kính SPHERE trên trục x
        marker.scale.y = 0.2  # Bán kính SPHERE trên trục y
        marker.scale.z = 0.2  # Bán kính SPHERE trên trục z

        # Màu sắc của các hình cầu (RGBA)
        marker.color.r = r  # Màu đỏ
        marker.color.g = g  # Màu xanh lá
        marker.color.b = b  # Màu xanh dương
        marker.color.a = 1.0  # Độ đậm của màu (1.0 là không trong suốt)

        # Pose
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # Thêm danh sách các điểm vào Marker
        for center in list_point:
            p = Point()
            p.x = center[0]
            p.y = center[1]
            p.z = center[2]
            marker.points.append(p)

        marker_pub.publish(marker)

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
                    print("The number of mirrors is less than 3")
                    # time.sleep(5)
                    self.rate.sleep()
                    continue
                
                if self.completed_initmode == 0:
                    self.process = 2
                else:
                    self.process = 3
                
            # -- Tìm ra vị trí khởi tạo cuả robot trên bản đồ và những gương nào đang được khớp với nhau
            elif self.process == 2:
                if self.completed_initmode == 0:

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
                    
                    # -- tìm list gương tham chiếu
                    self.ls_reference_ref_xy = []
                    self.ls_reference_ref_d = []
                    self.ls_reference_ref_theta = []
                    self.ls_reference_ref_dr = []

                    for index, ref in enumerate(self.data_map.reflectors):
                        self.ls_reference_ref_dr.append([ref.GlobalID, ref.Polar_Dist, ref.Polar_Phi])
                        self.ls_reference_ref_d.append(ref.Polar_Dist)
                        self.ls_reference_ref_theta.append(ref.Polar_Phi)
                        self.ls_reference_ref_xy.append([ref.GlobalID, ref.Cart_X, ref.Cart_Y])
                    
                    # print("Các điểm gương tham chiếu là", self.ls_reference_ref_xy)
                                
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

                        theta_r = acos((a*a + b*b - c*c)/(2*a*b))

                        if degrees(theta_r) < self.ANGLE_LIMIT_LOWER or degrees(theta_r) > self.ANGLE_LIMIT_UPPER:
                            self.list_N_angle.append(2*PI)
                        else:
                            self.list_N_angle.append(theta_r)

                    # print(f'Mảng thông số góc giữa các gương phát hiện với {len(self.list_N_angle)} tổ hợp là: {self.list_N_angle}')

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

                        theta_r = acos((a*a + b*b - c*c)/(2*a*b))
                        if degrees(theta_r) < self.ANGLE_LIMIT_LOWER or degrees(theta_r) > self.ANGLE_LIMIT_UPPER:
                            self.list_M_angle.append(2*PI)
                        else:
                            self.list_M_angle.append(theta_r)

                        # self.list_M_angle.append(theta_r)

                    # print(f'Mảng thông số góc giữa các gương tham chiếu với {len(self.list_M_angle)} tổ hợp là: {self.list_M_angle}')  # số lượng là n!/(3! * (n-3)!)

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

                    # Step 1: tạo list gương từ góc thành dạng giống với list khoảng cách
                    # list_IDref_satify_fromtheta_new = []
                    # for id_comb in list_IDref_satify_fromtheta:
                    #     ls_new = []
                    #     id0_comb_ref = id_comb[0][0]
                    #     id1_comb_ref = id_comb[0][1]
                    #     id2_comb_ref = id_comb[0][2]

                    #     id0_comb_det = id_comb[1][0]
                    #     id1_comb_det = id_comb[1][1]
                    #     id2_comb_det = id_comb[1][2]
                        
                    #     ls_new.append((id0_comb_ref, id0_comb_det))
                    #     ls_new.append([(id0_comb_ref, id1_comb_ref), (id0_comb_det, id1_comb_det)])
                    #     ls_new.append([(id0_comb_ref, id2_comb_ref), (id0_comb_det, id2_comb_det)])

                    #     #-- add new 10/3/2025
                    #     # ls_new.append([(id0_comb_ref, id1_comb_ref), (id0_comb_det, id2_comb_det)])
                    #     # ls_new.append([(id0_comb_ref, id2_comb_ref), (id0_comb_det, id1_comb_det)])

                    #     list_IDref_satify_fromtheta_new.append(ls_new)

                    # - Step 2: Tìm trong list khoảng cách có phần từ nào giống trong list góc ko -> đưa ra Id trùng
                    ls_IDmatch = []
                    # for item_theta_new in list_IDref_satify_fromtheta_new:
                    for item_theta_new in list_IDref_satify_fromtheta:
                        # id_theta = item_theta_new[0]
                        id0_theta_ref = item_theta_new[0][0]        # 2
                        id1_theta_ref = item_theta_new[0][1]        # 3
                        id2_theta_ref = item_theta_new[0][2]        # 5

                        id0_theta_det = item_theta_new[1][0]        # 1
                        id1_theta_det = item_theta_new[1][1]        # 2
                        id2_theta_det = item_theta_new[1][2]        # 4

                        for item_d in list_IDref_satify_fromd:
                            id0_dis_ref = item_d[0][0]                 # 2
                            id1_dis_ref = item_d[0][1]                 # 3
                            id0_dis_det = item_d[1][0]                 # 1
                            id1_dis_det = item_d[1][1]                 # 4

                            # -- kc1
                            # - case 1: tồn tại: 23 & 23
                            if id0_theta_ref == id0_dis_ref and id1_theta_ref == id1_dis_ref:
                                # subcase 1: tồn tại 12 và 12 
                                if id0_theta_det == id0_dis_det and id1_theta_det == id1_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id1_theta_det))

                                # subcase 2: tồn tại 12 và 21
                                elif id0_theta_det == id1_dis_det and id1_theta_det == id0_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id1_theta_det))
                                
                                # subcase 3: tồn tại 14 và 14
                                if id0_theta_det == id0_dis_det and id2_theta_det == id1_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id2_theta_det))
                                
                                # subcase 4: tồn tại 14 và 41
                                elif id0_theta_det == id1_dis_det and id2_theta_det == id0_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id2_theta_det))

                            # -- case 2: tồn tại: 23 && 32
                            elif id0_theta_ref == id1_dis_ref and id1_theta_ref == id0_dis_ref:
                                # subcase 1: tồn tại 12 và 12 
                                if id0_theta_det == id0_dis_det and id1_theta_det == id1_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id1_theta_det))

                                # subcase 2: tồn tại 12 và 21
                                elif id0_theta_det == id1_dis_det and id1_theta_det == id0_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id1_theta_det))
                                
                                # subcase 3: tồn tại 14 và 14
                                if id0_theta_det == id0_dis_det and id2_theta_det == id1_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id2_theta_det))
                                
                                # subcase 4: tồn tại 14 và 41
                                elif id0_theta_det == id1_dis_det and id2_theta_det == id0_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id1_theta_ref, id2_theta_det))

                            # -- kc2
                            # - case 1: tồn tại 25 ref và 25 det
                            if id0_theta_ref == id0_dis_ref and id2_theta_ref == id1_dis_ref:
                                # - subcase: 14 và 14
                                if id0_theta_det == id0_dis_det and id2_theta_det == id1_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id2_theta_det))
                                # - subcase: 14 và 41
                                elif id0_theta_det == id1_dis_det and id2_theta_det == id0_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id2_theta_det))

                                # subcase 3: tồn tại 12 và 12 
                                if id0_theta_det == id0_dis_det and id1_theta_det == id1_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id1_theta_det))

                                # subcase 4: tồn tại 12 và 21
                                elif id0_theta_det == id1_dis_det and id1_theta_det == id0_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id1_theta_det))

                            # - case 1: tồn tại 25 ref và 52 det
                            elif id0_theta_ref == id1_dis_ref and id2_theta_ref == id0_dis_ref:
                                # - subcase: 14 và 14
                                if id0_theta_det == id0_dis_det and id2_theta_det == id1_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id2_theta_det))
                                # - subcase: 14 và 41
                                elif id0_theta_det == id1_dis_det and id2_theta_det == id0_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id2_theta_det))

                                # subcase 3: tồn tại 12 và 12 
                                if id0_theta_det == id0_dis_det and id1_theta_det == id1_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id1_theta_det))

                                # subcase 4: tồn tại 12 và 21
                                elif id0_theta_det == id1_dis_det and id1_theta_det == id0_dis_det:
                                    ls_IDmatch.append((id0_theta_ref, id0_theta_det))
                                    ls_IDmatch.append((id2_theta_ref, id1_theta_det))

                    ls_IDresult_before = sorted(set(ls_IDmatch))
                    ls_IDresult = sorted(ls_IDresult_before, key=lambda x: x[1])

                    print("ID các điểm khớp là: ", ls_IDresult)
                    print("-------")

                    if len(ls_IDresult) < 3:
                        # - Tăng ngưỡng check
                        self.distance_matching_error_threshold += self.DISTANCE_THRESHOLD_INCREMENT

                        self.count_cal_again_zeromatch += 1
                        print(f"Số gương khớp < 3 => yêu cầu tính toán lại lần thứ {self.count_cal_again_zeromatch} vơí hệ số khớp theo khoảng cách là {self.distance_matching_error_threshold} và hệ số theo góc là {self.angle_matching_error_threshold}")

                        if self.distance_matching_error_threshold >= self.DISTANCE_THRESHOLD_MAX:
                            self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_ORIGIN
                            self.angle_matching_error_threshold += self.ANGLE_THRESHOLD_INCREMENT
                        
                        if self.angle_matching_error_threshold > self.ANGLE_THRESHOLD_MAX:
                            print("Ko có đủ gương cho AGV di chuyển => lắp thêm")
                            self.process = -1 
                            self.rate.sleep()
                            continue

                    else:

                        # - Kiểm tra xem có id nào cuả map tham chiếu bị trùng hay không ?
                        is_not_uniqueID = False
                        self.ls_IDref_collision = []
                        self.ls_IDdet_collision = []
                        for i in range(0, len(ls_IDresult)):
                            for j in range(i, len(ls_IDresult)):
                                if j != i:
                                    idj_ref = ls_IDresult[j][0]
                                    idi_ref = ls_IDresult[i][0]

                                    idj_det = ls_IDresult[j][1]
                                    idi_det = ls_IDresult[i][1]

                                    if idj_ref == idi_ref:
                                        is_not_uniqueID = True
                                        self.collision_ref_case += 1
                                        self.ls_IDref_collision.append(ls_IDresult[i])
                                        self.ls_IDref_collision.append(ls_IDresult[j])
                                    
                                    if idj_det == idi_det:
                                        is_not_uniqueID = True
                                        self.collision_det_case += 1
                                        self.ls_IDdet_collision.append(ls_IDresult[i])
                                        self.ls_IDdet_collision.append(ls_IDresult[j])
                            
                        if is_not_uniqueID:
                            is_not_uniqueID = False
                            # trường hợp có 2 cặp gương trùng nhau: 1 ref trùng, 1 det trùng, dộ dài bằng 4 => use Ransac 
                            # if self.collision_ref_case == 1 and self.collision_det_case == 1 and len(ls_IDresult) == 4:
                            #     print(f"Yêu câù tính toán vị trí sử dụng thuật toán Ransac!!")     
                            #     # - find id collision and the others in 4 case
                            #     ls_case = []
                            #     ls_case1 = [self.ls_IDdet_collision[0], self.ls_IDref_collision[0]]
                            #     ls_case2 = [self.ls_IDdet_collision[0], self.ls_IDref_collision[1]]
                            #     ls_case3 = [self.ls_IDdet_collision[1], self.ls_IDref_collision[0]]
                            #     ls_case4 = [self.ls_IDdet_collision[1], self.ls_IDref_collision[1]]

                            #     ls_case.append(ls_case1)
                            #     ls_case.append(ls_case2)
                            #     ls_case.append(ls_case3)
                            #     ls_case.append(ls_case4)

                            #     ls_case_full = []
                            #     for case in ls_case:
                            #         ls_IDref_others = []
                            #         ls_IDdet_others = []
                            #         for i, ref in enumerate(self.ls_reference_ref_xy):
                            #             if ref[0] != case[0][0] and ref[0] != case[1][0]:
                            #                 ls_IDref_others.append(ref[0])
                                    
                            #         for i, ref in enumerate(ls_detected_ref):
                            #             if ref[0] != case[0][1] and ref[0] != case[1][1]:
                            #                 ls_IDdet_others.append(ref[0])

                            #         for idj in ls_IDref_others:
                            #             for idi in ls_IDdet_others:
                            #                 ls_case_full.append([case[0], case[1], (idj, idi)])

                            #     # find rmse of each elements in each case
                            #     self.process = 4
                            #     self.ls_IDransac = ls_case_full
                            #     self.ls_info_detref = ls_detected_ref
                            #     print("Case có thể là: ", self.ls_IDransac)

                            # else:
                            # -- giảm ngưỡng check
                            self.distance_matching_error_threshold -= self.DISTANCE_THRESHOLD_INCREMENT

                            self.count_cal_again_samematch += 1
                            print(f"tồn tại gương trùng => yêu cầu tính toán lại lần thứ {self.count_cal_again_samematch} vơí hệ số khớp theo khoảng cách là {self.distance_matching_error_threshold} và hệ số theo góc là {self.angle_matching_error_threshold}")
                            
                            # -- reset check variables
                            self.collision_det_case = 0
                            self.collision_ref_case = 0
                        
                        else:

                            # # -- Use SVD for find pos of robot on map
                            # # - Step 0: Lọc map tham chiếu khớp trong map gốc
                            n = len(self.ls_reference_ref_xy)
                            ls_reference_refFilter = []
                            ls_reference_refFilter_dr = []
                            for id_ref in ls_IDresult:
                                for i, ref in enumerate(self.ls_reference_ref_xy):
                                    if ref[0] == id_ref[0]:
                                        ls_reference_refFilter.append([ref[1], ref[2]])
                                        ls_reference_refFilter_dr.append([self.ls_reference_ref_dr[i][1], self.ls_reference_ref_dr[i][2]])
                            
                            print(" Độ dài thông số các gương tham chiếu khớp là: ", len(ls_reference_refFilter))

                            n = len(ls_detected_ref)
                            # print("Thông tin các gương phát hiện là ", ls_detected_ref)
                            ls_detected_refFilter = []
                            for id_ref in ls_IDresult:
                                for ref in ls_detected_ref:
                                    if ref[0] == id_ref[1]:
                                        # print(f"Phần tử mới được thêm vào list là: x = {ref[1]}, y = {ref[2]}")
                                        ls_detected_refFilter.append([ref[1], ref[2]])
                            
                            # print("Độ dài thông số các gương phát hiện khớp là: ", len(ls_detected_refFilter))

                            # -- Step 1: find pose of robot
                            xr, yr, phiR = self.SVD_algorithm(ls_reference_refFilter, ls_detected_refFilter)
                            print("num ref matching: ", len(ls_detected_refFilter), "| Pose: ", xr, yr, degrees(phiR))

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
                            self.rmse_xy = np.sqrt(np.mean(np.sum(error**2, axis=1)))
                            print(f" Hệ số rms là: {self.rmse_xy}")

                            # errors = np.sqrt(np.sum((detected_arr - reference_arr)**2, axis=1))
                            # # In kết quả
                            # for i, rmse_value in enumerate(errors):
                            #     print(f"RMSE cặp gương {i+1}: {rmse_value}")
                            # self.process = -1 

                            # - Tính RMSE for dr => trọng số w cho navigation mode
                            detected_arr_dr = np.array(detected_mirrors_dr)
                            reference_arr_dr = np.array(reference_mirrors_dr)
                            
                            self.rmse_dr = [abs((x - a)*(y - b)) for (x, y), (a, b) in zip(detected_arr_dr, reference_arr_dr)]

                            self.w_start = self.WEIGHT_COEF * max(self.rmse_dr)
                            print("Trọng số khởi tạo là: ", self.w_start)

                            self.pre_w = self.w_start

                            if self.rmse_xy <= self.RMSE_THRESHOLD_INITMODE:        # sai số nhỏ hơn 2 cm theo x, y -> tạm thời cho nhỏ hơn 1 để debug
                                # # - send transform to rviz
                                # self.translation = (xr, yr, 0.0)
                                # self.quanternion = self.euler_to_quaternion(phiR)
                                # self.br.sendTransform(self.translation, self.quanternion, rospy.Time.now(), self.ref_frame, self.origin_frame)

                                # -- publish data
                                self.ls_localID = []
                                self.ls_globalID = []
                                for id_ref in ls_IDresult:
                                    self.ls_globalID.append(id_ref[0])
                                    self.ls_localID.append(id_ref[1])

                                # self.pub_lidar_pose(self.lidar_pose_pub, xr, yr, phiR, ls_localID, ls_globalID, len(ls_IDresult))
                                
                                # -- update tranform data for next step
                                self.x_tf = xr
                                self.y_tf = yr
                                self.r_tf = phiR

                                self.translation = (self.x_tf, self.y_tf, 0.0)
                                self.quanternion = self.euler_to_quaternion(self.r_tf)
                                self.br.sendTransform(self.translation, self.quanternion, rospy.Time.now(), self.ref_frame, self.origin_frame)

                                # -- update data to ros
                                self.ls_w[0] = self.distance_matching_error_threshold
                                self.ls_w[1] = self.angle_matching_error_threshold
                                self.ls_nomatch[0] = self.count_cal_again_zeromatch
                                self.ls_samematch[0] = self.count_cal_again_samematch
                                self.ls_highrmse[0] = self.count_cal_again_highrmse

                                self.ls_rmse[0] = self.rmse_xy

                                # self.pub_lidar_pose_full(self.lidar_pose_pub, self.ls_rmse, self.ls_w, self.ls_nomatch, self.ls_samematch, self.ls_highrmse, self.x_tf, self.y_tf, self.r_tf, self.ls_localID, self.ls_globalID, self.w_start, self.process, len(self.ls_localID))

                                print("-- Matching good in init mode-> move to next step --")
                                # -- move to next step
                                self.completed_initmode = 1
                                
                                # -- reset varibles
                                self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_ORIGIN
                                self.angle_matching_error_threshold = self.ANGLE_THRESHOLD_ORIGIN 

                            else:                                # hướng tới báo lôĩ mât gương
                                self.count_cal_again_highrmse += 1
                                print(f"rms > {self.RMSE_THRESHOLD_INITMODE} => yêu cầu tính toán lại lần thứ {self.count_cal_again_highrmse} vơí hs khớp theo khoảng cách là {self.distance_matching_error_threshold} và hệ số theo góc là {self.angle_matching_error_threshold}")                        

                else:
                    self.process = 1

            # - Tìm ra vị trí robot với thuật toán ransac
            elif self.process == 4:
                if self.completed_initmode == 0:
                    # Tìm các rmse từ các case 
                    ls_rmse_xyvalue = []
                    ls_rmse_drvalue = []
                    ls_tf_value = []

                    for case in self.ls_IDransac:
                        # # -- Use SVD for find pos of robot on map
                        # # - Step 0: Lọc map tham chiếu khớp trong map gốc
                        n = len(self.ls_reference_ref_xy)
                        ls_reference_refFilter = []
                        ls_reference_refFilter_dr = []
                        for id_ref in case:
                            for i, ref in enumerate(self.ls_reference_ref_xy):
                                if ref[0] == id_ref[0]:
                                    ls_reference_refFilter.append([ref[1], ref[2]])
                                    ls_reference_refFilter_dr.append([self.ls_reference_ref_dr[i][1], self.ls_reference_ref_dr[i][2]])
                        
                        # print(" Độ dài thông số các gương tham chiếu khớp là: ", len(ls_reference_refFilter))

                        n = len(self.ls_info_detref)
                        # print("Thông tin các gương phát hiện là ", self.ls_info_detref)
                        ls_detected_refFilter = []
                        for id_ref in case:
                            for ref in self.ls_info_detref:
                                if ref[0] == id_ref[1]:
                                    # print(f"Phần tử mới được thêm vào list là: x = {ref[1]}, y = {ref[2]}")
                                    ls_detected_refFilter.append([ref[1], ref[2]])
                        
                        # print("Độ dài thông số các gương phát hiện khớp là: ", len(ls_detected_refFilter))

                        # -- Step 1: find pose of robot
                        xr, yr, phiR = self.SVD_algorithm(ls_reference_refFilter, ls_detected_refFilter)
                        # print("num ref matching of case: ", len(ls_detected_refFilter), "| Pose: ", xr, yr, degrees(phiR))

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
                        self.rmse_xy = np.sqrt(np.mean(np.sum(error**2, axis=1)))
                        print(f" Hệ số rms of case {case} là: {self.rmse_xy}")

                        # - Tính RMSE for dr => trọng số w cho navigation mode
                        detected_arr_dr = np.array(detected_mirrors_dr)
                        reference_arr_dr = np.array(reference_mirrors_dr)
                        
                        self.rmse_dr = [abs((x - a)*(y - b)) for (x, y), (a, b) in zip(detected_arr_dr, reference_arr_dr)]

                        ls_rmse_xyvalue.append(self.rmse_xy)
                        ls_rmse_drvalue.append(self.rmse_dr)
                        ls_tf_value.append([xr, yr, phiR])
                    
                    print("Các hệ số rmse là: ", ls_rmse_xyvalue)
                    min_rmse = min(ls_rmse_xyvalue)
                    print("Giá trị rmse min là: ", min_rmse)
                    ik = ls_rmse_xyvalue.index(min_rmse)

                    print("mảng giá trị rmse dr là:", ls_rmse_drvalue[ik])
                    self.w_start = self.WEIGHT_COEF * max(ls_rmse_drvalue[ik])
                    print("Trọng số khởi tạo là: ", self.w_start)

                    self.pre_w = self.w_start

                    self.x_tf = ls_tf_value[ik][0]
                    self.y_tf = ls_tf_value[ik][1]
                    self.r_tf = ls_tf_value[ik][2]
                    print("-- Matching good -> move to next step --")

                    # -- move to next step
                    self.completed_initmode = 1
                    
                    # -- reset varibles
                    self.distance_matching_error_threshold = self.DISTANCE_THRESHOLD_ORIGIN
                    
                else:
                    self.process = 3

            # - Tìm ra vị trí robot sau khi đã tìm ra vị trí khởi tạo
            elif self.process == 3:
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
                convert_posReflector = []
                convert_posReflector_d = []
                convert_posReflector_theta = []

                convert_posReflector_d_predict = []

                for i, ref in enumerate(self.data_raw_reflector.reflectors):
                    x_conpensate = ref.Cart_X - vlidar_x*(1 - (i+1)/k)
                    y_conpensate = ref.Cart_Y - vlidar_y*(1 - (i+1)/k)

                    ls_detected_ref.append([ref.LocalID, x_conpensate, y_conpensate])

                    x_cv, y_cv = self.calculate_reflector_position_in_map(self.x_tf, self.y_tf, self.r_tf, x_conpensate, y_conpensate)
                    d_raw = sqrt(x_cv*x_cv + y_cv*y_cv)
                    theta_raw = atan2(y_cv, x_cv)

                    convert_posReflector.append([ref.LocalID, d_raw, theta_raw])
                    convert_posReflector_d.append(d_raw)
                    convert_posReflector_theta.append(theta_raw) 

                    x_cv_predict, y_cv_predict = self.calculate_reflector_position_in_map(x_predict, y_predict, r_predict, x_conpensate, y_conpensate)
                    d_predict = sqrt(x_cv_predict*x_cv_predict + y_cv_predict*y_cv_predict)
                    convert_posReflector_d_predict.append(d_predict)

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

                d_near = min(convert_posReflector_d_predict)
                d_far = max(convert_posReflector_d_predict)

                d_near = (1-self.DISTANCE_COEF)*d_near
                d_far = (1+self.DISTANCE_COEF)*d_far

                print(f"khoảng cách dnear = {d_near}, dfar = {d_far}")
                print("-----")
                ls_reference_ref_xy_filter = []
                ls_reference_ref_d_filter = []
                ls_reference_ref_theta_filter = []
                ls_reference_ref_dr_filter = []

                for i in range(0, len(self.ls_reference_ref_d)):
                    d_current = self.ls_reference_ref_d[i]

                    if d_near <= d_current <= d_far:
                        ls_reference_ref_dr_filter.append(self.ls_reference_ref_dr[i])
                        ls_reference_ref_d_filter.append(d_current)
                        ls_reference_ref_theta_filter.append(self.ls_reference_ref_theta[i])
                        ls_reference_ref_xy_filter.append(self.ls_reference_ref_xy[i])

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
                v_lidar = sqrt(vlidar_x*vlidar_x + vlidar_y*vlidar_y)
                w_sigma = self.w_start + self.VEL_COEF*v_lidar + self.OMEGA_COEF*vlidar_w     # sai số khoảng cách * sai số góc 

                print(f"Vận tốc hiện tại của lidar: {v_lidar}. Trọng số: {w_sigma}")
                # Tìm giá trị nhỏ nhất tại mỗi cột
                min_values = np.min(w, axis=0)

                # Tìm chỉ số hàng tương ứng với giá trị nhỏ nhất tại mỗi cột
                min_indices = np.argmin(w, axis=0)

                # So sánh với ngưỡng w_sigma
                valid_indices = np.where(min_values < w_sigma, min_indices, -1)

                # In kết quả
                # print("Giá trị nhỏ nhất tại mỗi cột:", min_values)
                print("Chỉ số hàng tương ứng (hoặc -1 nếu không đạt ngưỡng):", valid_indices)

                # -- xử lý khi bị có trường hợp các id trùng nhau 
                is_not_uniqueID = False
                # ls_index_id = []
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

                # - Tạo list thoả mãn để tính lại vị trí của robot
                # # - Step 0: Lọc map tham chiếu khớp trong map gốc
                n = len(ls_reference_ref_xy_filter)
                ls_reference_refFilter = []
                ls_reference_refFilter_dr = []
                for id_ref in valid_indices:
                    for i, ref in enumerate(ls_reference_ref_xy_filter):
                        if id_ref != -1 and i == id_ref:
                            ls_reference_refFilter.append([ref[1], ref[2]])
                            ls_reference_refFilter_dr.append([ls_reference_ref_dr_filter[i][1], ls_reference_ref_dr_filter[i][2]])

                # print(f"List tham chiếu khớp là: {ls_reference_refFilter}")
                # print("-----")

                ls_detected_refFilter = []
                
                for i, id_ref in enumerate(valid_indices):
                    if id_ref != -1:
                        ls_detected_refFilter.append([ls_detected_ref[i][1], ls_detected_ref[i][2]])

                # print(f"List phát hiện khớp là: {ls_detected_refFilter}")
                # print("-----")    

                if len(ls_detected_refFilter) < 3:
                    print("Số gương khớp nhỏ hơn 3 => navigation mode fail => init again")
                    # -- Tạm thời dừng chương trình để đánh giá
                    self.process = 1
                    self.completed_initmode = 0
                    self.count_zeromatch += 1

                else:
                    # -- Step 1: find pose of robot
                    xr, yr, phiR = self.SVD_algorithm(ls_reference_refFilter, ls_detected_refFilter)
                    print("num ref matching: ", len(ls_detected_refFilter), "| Pose: ", xr, yr, degrees(phiR))

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

                    detected_arr = np.array(detected_mirrors)
                    reference_arr = np.array(reference_mirrors)
                    
                    # Tính RMSE
                    error = detected_arr - reference_arr
                    self.rmse_xy = np.sqrt(np.mean(np.sum(error**2, axis=1)))
                    print(f" Hệ số rms là: {self.rmse_xy}")

                    # - Tính RMSE for dr => cập nhật trọng số w cho navigation mode
                    detected_arr_dr = np.array(detected_mirrors_dr)
                    reference_arr_dr = np.array(reference_mirrors_dr)
                    
                    self.rmse_dr = [abs((x - a)*(y - b)) for (x, y), (a, b) in zip(detected_arr_dr, reference_arr_dr)]

                    self.w_start = self.WEIGHT_COEF * max(self.rmse_dr)
                    print("Trọng số khởi tạo là: ", self.w_start)

                    if self.w_start < self.pre_w:
                        self.w_start = self.pre_w
                        print("Trọng số khởi tạo update lại là: ", self.w_start)
                        
                    else:
                        self.pre_w = self.w_start

                    if self.data_NNinfoRespond.process == 6:   # agv di chuyển parking
                        self.RMSE_THRESHOLD_NAVMODE = 0.03
                    else:
                        self.RMSE_THRESHOLD_NAVMODE = 0.1

                    if self.rmse_xy <= self.RMSE_THRESHOLD_NAVMODE:
                        print("-- Matching in navigation mode good -> move to next step --")
                        # # # - send transform to rviz
                        # self.translation = (xr, yr, 0.0)
                        # self.quanternion = self.euler_to_quaternion(phiR)
                        # self.br.sendTransform(self.translation, self.quanternion, rospy.Time.now(), self.ref_frame, self.origin_frame)

                        # -- publish data
                        self.ls_localID = []
                        self.ls_globalID = []
                        for i, id_ref in enumerate(valid_indices):
                            if id_ref != -1:
                                self.ls_globalID.append(id_ref+1)
                                self.ls_localID.append(i+1)

                        # self.pub_lidar_pose(self.lidar_pose_pub, xr, yr, phiR, ls_localID, ls_globalID, len(ls_localID))

                        # -- update tranform data for next step
                        self.x_tf = xr
                        self.y_tf = yr
                        self.r_tf = phiR

                        self.translation = (self.x_tf, self.y_tf, 0.0)
                        self.quanternion = self.euler_to_quaternion(self.r_tf)
                        self.br.sendTransform(self.translation, self.quanternion, rospy.Time.now(), self.ref_frame, self.origin_frame)

                        self.ls_nomatch[1] = self.count_zeromatch
                        self.ls_samematch[1] = 0
                        self.ls_highrmse[1] = self.count_highrmse

                        self.ls_rmse[1] = self.rmse_xy
                        # publish to ros
                        self.pub_lidar_pose_full(self.lidar_pose_pub, self.ls_rmse, self.ls_w, self.ls_nomatch, self.ls_samematch, self.ls_highrmse, self.x_tf, self.y_tf, self.r_tf, self.ls_localID, self.ls_globalID, self.w_start, self.process, len(self.ls_localID))
                        # publish to ros
                        # self.pub_lidar_pose_full(self.lidar_pose_pub, self.ls_rmse, self.ls_w, self.ls_nomatch, self.ls_samematch, self.ls_highrmse, self.mean_tfx, self.mean_tfy, self.mean_tfr, self.ls_localID, self.ls_globalID, self.w_start, self.process, len(self.ls_localID))

                    else:
                        self.count_highrmse += 1
                        print(f"-- Navigation mode fail -> back to init mode --, tính toán lại lần thứ {self.count_highrmse} vơí trọng số là {self.distance_matching_error_threshold}")
                        self.completed_initmode = 0
                    
                    # - update new pose of reflector
                    self.process = 1
                    print("#######################################")

            self.rate.sleep()

def main():
    print ("--- Run navigation mode---")
    program = Navigation_mode()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




