#!/usr/bin/env python3

"""
*** infomation
*** Task Description: 
    + Tìm ra vị trí cuả robot bằng việc so sánh toạ độ x, y giữa các gương
        + Đã test tốc dộ linear_x = 0.4 m/s, tốc độ quay 0.25 m/s

    + chưa sử dụng tới thuật toán bù chuyển động
    
    => Thực tế, đây mới là chế độ initial mode

*** Need to do
    + Cải tiến thuật toán khớp gương
    + sử dụng thêm thuật toán bù chuyển động
    + Vấn đề gương ảo ???? Lôĩ gương nhỏ hơn 3 thì sao ????
    + Mục tiêu vận tốc >= 0.8 m/s
    
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
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped, TwistWithCovarianceStamped
from navigation_reflector.msg import *
from itertools import combinations

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import Counter

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
        # rospy.Subscriber("/Keyboard_cmd", String, self.callback_KeyboardCmd)
        # self.keyboad_cmd = String()        

        rospy.Subscriber('/info_raw_reflector', Raw_reflector, self.callback_infoRawReflector, queue_size = 10)
        self.data_raw_reflector = Raw_reflector()
        self.is_rawReflector = False

        rospy.Subscriber('/init_mode_respond', Init_mode, self.callback_initmode_respond, queue_size = 10)
        self.data_initmode_respond = Init_mode()
        self.is_recv_initmode_data = False

        rospy.Subscriber('/map', Raw_reflector, self.callback_map, queue_size = 10)
        self.data_map = Raw_reflector()
        self.is_recv_mapdata = False

        rospy.Subscriber('/raw_vel', TwistWithCovarianceStamped, self.callback_rawvel, queue_size = 10)
        self.data_rawvel = TwistWithCovarianceStamped()
        self.is_recv_rawvel = False

        self.navigation_reflector_pub = rospy.Publisher('/nav_reflector_respond', PoseStamped, queue_size=10)
        self.navigation_reflector_data = PoseStamped()

        self.marker_pub_predict = rospy.Publisher('/visualization_marker_predict', Marker, queue_size=10)
        self.marker_pub_matching = rospy.Publisher('/visualization_marker_matching', Marker, queue_size=10)

        self.lidar_pose_pub = rospy.Publisher('/r2000_data', R2000_data, queue_size=10)
        # self.lidar_pose_data = R2000_data()

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
        self.origin_frame = "map"

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
        self.distance_matching_error_threshold = 0.05  # (zf = 2 cm )
        self.angle_matching_error_threshold = 5*PI/180     # (gf = 1 độ)
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

        self.pre_phiR = 0
        self.step = 0

    def callback_infoRawReflector(self, data):
        self.data_raw_reflector = data
        self.is_rawReflector = True

    def callback_initmode_respond(self, data):
        self.data_initmode_respond = data
        self.is_recv_initmode_data = True

    def callback_map(self, data):
        self.data_map = data
        self.is_recv_mapdata = True

    def callback_rawvel(self, data):
        self.data_rawvel = data
        self.is_recv_rawvel = True

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

    def remove_absolute_duplicates(self, n, data):
        seen = set()  # Tập hợp lưu trữ các giá trị tuyệt đối của d đã xuất hiện
        filtered_data = []  # Danh sách kết quả

        for item in data:
            if n == 2:
                x, y, d = item
            elif n == 3:
                x, y, z, d = item
            elif n == 0:
                d = item
                
            abs_d = abs(d)

            if abs_d not in seen:
                seen.add(abs_d)
                filtered_data.append(item)

        return filtered_data

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

        # Phân tích SVD trên ma trận Hstivietnam
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

    def covariance_matrix(self, A, B):
        """Tính ma trận hiệp phương sai H có dấu chuyển vị T"""
        n = len(A)
        H = [[0, 0], [0, 0]]
        
        for i in range(n):
            # Nhân ma trận hàng của A_n với ma trận cột của B_n
            H[0][0] += A[i][0] * B[i][0]  # H11 = ∑(x_A * x_B)
            H[0][1] += A[i][0] * B[i][1]  # H12 = ∑(x_A * y_B)
            H[1][0] += A[i][1] * B[i][0]  # H21 = ∑(y_A * x_B) (chính là chuyển vị của H12)
            H[1][1] += A[i][1] * B[i][1]  # H22 = ∑(y_A * y_B)

        return H

    def transpose(self, matrix):
        """Chuyển vị ma trận"""
        return [[matrix[j][i] for j in range(len(matrix))] for i in range(len(matrix[0]))]

    def svd_decomposition(self, H):
        """Thực hiện SVD bằng cách tính ma trận riêng"""
        # Đơn giản hóa: Tính trị riêng (eigenvalues) và vector riêng (eigenvectors)
        # Sử dụng thư viện math để tính toán thay vì numpy
        trace = H[0][0] + H[1][1]
        det = (H[0][0] * H[1][1]) - (H[0][1] * H[1][0])
        lambda1 = (trace + sqrt(trace**2 - 4 * det)) / 2
        lambda2 = (trace - sqrt(trace**2 - 4 * det)) / 2
        
        # Vector riêng của H (cột của U)
        U = [[H[0][0] - lambda2, H[0][1]], [H[1][0], H[1][1] - lambda2]]
        norm = sqrt(U[0][0]**2 + U[1][0]**2)
        U = [[U[0][0] / norm, U[0][1] / norm], [U[1][0] / norm, U[1][1] / norm]]
        
        V = [[H[0][0] - lambda1, H[0][1]], [H[1][0], H[1][1] - lambda1]]
        norm = sqrt(V[0][0]**2 + V[1][0]**2)
        V = [[V[0][0] / norm, V[0][1] / norm], [V[1][0] / norm, V[1][1] / norm]]
        
        return U, V

    # def svd_decomposition(self, A):
    #     """
    #     Tính SVD của ma trận A mà không sử dụng np.linalg.svd.
    #     """
    #     # Bước 1: Tính eigenvalues & eigenvectors của AA^T để lấy U
    #     AAT = np.dot(A, A.T)  # Ma trận hiệp phương sai
    #     eigenvalues_U, U = np.linalg.eig(AAT)  # Lấy eigenvalues & eigenvectors
    #     U = U[:, np.argsort(-eigenvalues_U)]  # Sắp xếp giảm dần
        
    #     # Bước 2: Tính eigenvalues & eigenvectors của A^T A để lấy V
    #     ATA = np.dot(A.T, A)
    #     eigenvalues_V, V = np.linalg.eig(ATA)
    #     V = V[:, np.argsort(-eigenvalues_V)]  # Sắp xếp giảm dần

    #     # Bước 3: Tính sigma (giá trị kỳ dị)
    #     singular_values = np.sqrt(np.abs(eigenvalues_U))  # Căn bậc hai của eigenvalues_U
    #     Sigma = np.zeros_like(A, dtype=float)
    #     for i in range(min(A.shape)):
    #         Sigma[i, i] = singular_values[i]  # Điền vào đường chéo

    #     return U, Sigma, V.T  # Trả về V chuyển vị

    def invert_matrix(self, R):
        """Tính ma trận nghịch đảo của ma trận quay 2x2"""
        det_R = R[0][0] * R[1][1] - R[0][1] * R[1][0]
        
        print("Định thức R là:")
        print(det_R)
        print("--------------------------------")

        if det_R == 0:
            raise ValueError("Ma trận quay không khả nghịch")
        
        # Tính ma trận nghịch đảo
        R_inv = [[R[1][1] / det_R, -R[0][1] / det_R],
                [-R[1][0] / det_R, R[0][0] / det_R]]

        print("Ma trận quay R nghịch đảo là:")
        print(R_inv)
        print("--------------------------------")

        return R_inv

    def global_position(self, local_pos, R, t):
        """Tính vị trí của robot trong hệ tọa độ toàn cục bằng công thức gốc"""
        # Bước 1: Trừ vector tịnh tiến
        x_prime = local_pos[0] - t[0][0]
        y_prime = local_pos[1] - t[1][0]

        # Bước 2: Tính nghịch đảo của ma trận R
        R_inv = self.invert_matrix(R)

        # Bước 3: Nhân ma trận R_inv với tọa độ đã được dịch
        x_g = R_inv[0][0] * x_prime + R_inv[0][1] * y_prime
        y_g = R_inv[1][0] * x_prime + R_inv[1][1] * y_prime

        return [x_g, y_g]

    def compute_robot_angle(self, R):
        """Tính góc quay của robot trong hệ tọa độ toàn cục"""
        theta_g = -atan2(R[1][0], R[0][0]) * (180 / PI)
        return theta_g


    def count_duplicate_pairs(self, lst):
        # Chuyển danh sách con thành tuple để có thể đếm được
        tuple_list = [tuple(sublist) for sublist in lst]
        
        # Đếm số lần xuất hiện của từng cặp
        count_dict = Counter(tuple_list)
        
        return dict(count_dict)

    def find_max_element(self, x, data):
        # Lọc các phần tử có giá trị thứ 2 trong tuple bằng x
        filtered = {key: value for key, value in data.items() if (key[1] == x and value >= 3)}
        
        # print(filtered)
        if not filtered:
            return None  # Nếu không có phần tử nào thỏa mãn, trả về None
        
        # Tìm phần tử có giá trị lớn nhất trong dictionary đã lọc
        max_element = max(filtered, key=filtered.get)
        
        return max_element

    def pub_lidar_pose(self, pub, x, y, phi, no_ref):
        lidar_data = R2000_data()
        lidar_data.header.frame_id = "scanner_link"
        lidar_data.x = x
        lidar_data.y = y
        lidar_data.phi = phi
        lidar_data.number_reflectors = no_ref
        pub.publish(lidar_data)

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

                # if self.is_recv_initmode_data == True:
                #     c_k = c_k + 1
                # else:
                #     self.log_mess("warn","Wait data from Initialization node", c_k)

                # if self.is_recv_rawvel == True:
                #     c_k = c_k + 1
                # else:
                #     self.log_mess("warn","Wait data from Kinematic node", c_k)

                if c_k == 2:
                    rospy.loginfo("Completed wakeup ('_')")
                    self.process = 2              

            # -- give out predict pose of reflector after scanning period time
            elif self.process == 1:
                if self.is_rawReflector == False:
                    self.rate.sleep()
                    continue
                    
                self.is_rawReflector = False

                # -- 
                ls_poseReflector = []
                vx = self.data_rawvel.twist.twist.linear.x
                vy = 0
                thetaz = self.data_rawvel.twist.twist.angular.z

                vlidar_x = vx - thetaz * self.DISTANCE_Y_BETWEEN_LIDAR_RB
                vlidar_y = vy + thetaz * self.DISTANCE_X_BETWEEN_LIDAR_RB
                k = len(self.data_raw_reflector.points)
                self.ls_predict_ref = []

                for index, ref in enumerate(self.data_raw_reflector.points):
                    x = ref.x - vlidar_x*self.SCANNING_PERIOD*(1 - (index + 1)/k)
                    y = ref.y - vlidar_y*self.SCANNING_PERIOD*(1 - (index + 1)/k)
                    ls_poseReflector.append([x, y, 0.0])

                    # -- create predict reflector pos
                    theta_f = atan2(y, x)
                    d_f = x*cos(theta_f)

                    self.ls_predict_ref.append([(index + 1), d_f, theta_f])               

                # -- pub predict ref to rviz
                self.pub_marker(self.marker_pub_predict, ls_poseReflector, 0.0, 1.0, 0.0)

                # -- move to next step
                self.process = 2
            
            # -- Tìm ra các vị trí gương khớp trên map tham chiếu theo toạ độ x, y
            elif self.process == 2:
                if self.is_rawReflector == False:
                    self.rate.sleep()
                    continue
                    
                self.is_rawReflector = False

                # -- kiểm tra số lượng gương quét được
                # num_reflector = len(self.data_raw_reflector.points)
                # if num_reflector < 3:
                #     print("The number of mirrors is less than 3")
                #     self.rate.sleep()
                #     continue

                # -- đưa ra khoảng tìm kiếm tiếp theo 
                # d_near, d_far = self.find_min_max(self.ls_predict_ref)
                # print(f"Khoảng tìm kiếm là từ {d_near} tới {d_far}")

                # -- tìm list gương detected trong khoảng tìm kiêms
                ls_detected_ref = []
                # ls_detected_refFilter = []
                ls_index = []
                for index, ref in enumerate(self.data_raw_reflector.points):
                    # -- create raw reflector pos
                    theta_raw = atan2(ref.y, ref.x)
                    d_raw = ref.x*cos(theta_raw)
                    # if d_near <= d_raw <= d_far:
                    ls_index.append(index+1)
                    ls_detected_ref.append([(index + 1), ref.x, ref.y])
                    # ls_detected_refFilter.append([ref.x, ref.y])
                
                # print(f"List gương phát hiệnlà: {ls_index}")

                # - Xác định khoảng cách và góc giưã các gương
                n = len(ls_detected_ref)
                # list_N_distance = []
                self.list_N_distance = []
                for i in range(0, n):
                    for j in range(i, n):
                        if j != i:
                            dx = ls_detected_ref[j][1] - ls_detected_ref[i][1]
                            dy = ls_detected_ref[j][2] - ls_detected_ref[i][2]
                            d = sqrt(dx*dx + dy*dy)
                            self.list_N_distance.append([ls_detected_ref[i][0], ls_detected_ref[j][0], d])

                # print("Mảng thông số các gương raw là: ", list_M_distance)
                # self.list_N_distance = self.remove_absolute_duplicates(2, list_N_distance)
                ######### print("Mảng thông số khoảng cách giưã các gương phát hiện là: ", self.list_N_distance)

                # -- Step 2: Tính list góc giữa các điểm gương
                # n = len(ls_detected_ref)
                # combs = list(combinations(ls_detected_ref, 3))
                # # Tìm góc của các tổ hợp đó
                # self.list_N_angle = []
                # for comb in combs:
                #     list_N_comb_final = []
                #     for i in range(0, 3):
                #         for j in range(i, 3):             
                #             if j != i:
                #                 dx = comb[j][1] - comb[i][1]
                #                 dy = comb[j][2] - comb[i][2]
                #                 d = sqrt(dx*dx + dy*dy)
                #                 list_N_comb_final.append([comb[0][0], comb[1][0], comb[2][0], d])

                #     a = list_N_comb_final[0][3]
                #     b = list_N_comb_final[1][3]
                #     c = list_N_comb_final[2][3]

                #     theta_r = acos((a*a + b*b - c*c)/(2*a*b))
                #     self.list_N_angle.append([comb[0][0],comb[1][0], comb[2][0], theta_r])

                # print(f'Mảng thông số góc giữa các gương phát hiện với {len(self.list_N_angle)} tổ hợp là: {self.list_N_angle}')

                # -- tìm list gương tham chiếu trong khoảng tìm kiếm
                ls_reference_ref = []
                ls_index = []
                for index, ref in enumerate(self.data_map.points):
                    theta_ref = atan2(ref.y, ref.x)
                    d_ref = ref.x*cos(theta_ref)

                    # if d_near - 3 <= d_ref <= d_far + 3:
                    ls_reference_ref.append([(index + 1), ref.x, ref.y])
                    ls_index.append(index + 1)
                            
                # print(f"list gương tham chiếu trong khoảng là: {ls_index}")

                # - Step 1: Tìm khoảng cách giữa các gương tham chiếu
                n = len(ls_reference_ref)
                # list_M_distance = []
                self.list_M_distance = []

                for i in range(0, n):
                    for j in range(i, n):
                        if j != i:
                            dx = ls_reference_ref[j][1] - ls_reference_ref[i][1]
                            dy = ls_reference_ref[j][2] - ls_reference_ref[i][2]
                            d = sqrt(dx*dx + dy*dy)
                            self.list_M_distance.append([i+1, j+1, d])
                
                # self.list_M_distance = self.remove_absolute_duplicates(2, list_M_distance)
                # print("Mảng thông số khoảng cách các gương tham chiếu là: ", self.list_M_distance)    ## số lượng là nC2 giá trị

                # - Step 2: Tìm các góc của hệ gương
                # Tìm tổ hợp các tổ hợp chập 3 cuả hệ gương
                # n = len(ls_reference_ref)
                # combs = list(combinations(ls_reference_ref, 3))

                # # Tìm góc của các tổ hợp đó
                # self.list_M_angle = []
                # for comb in combs:
                #     list_M_comb_final = []
                #     for i in range(0, 3):
                #         for j in range(i, 3):             
                #             if j != i:
                #                 dx = comb[j][1] - comb[i][1]
                #                 dy = comb[j][2] - comb[i][2]
                #                 d = sqrt(dx*dx + dy*dy)
                #                 list_M_comb_final.append([comb[0][0],comb[1][0], comb[2][0], d])
                    
                #     a = list_M_comb_final[0][3]
                #     b = list_M_comb_final[1][3]
                #     c = list_M_comb_final[2][3]

                #     theta_r = acos((a*a + b*b - c*c)/(2*a*b))
                #     self.list_M_angle.append([comb[0][0],comb[1][0], comb[2][0], theta_r])

                # print(f'Mảng thông số góc giữa các gương tham chiếu với {len(self.list_M_angle)} tổ hợp là: {self.list_M_angle}')  # số lượng là n!/(3! * (n-3)!)

                # - Tìm sự khác nhau giữa 2 map gương tham chiếu và map gương detect được
                self.list_Z_distance = [[[x[0],x[1], y[0], y[1], abs(x[2] - y[2])] for y in self.list_N_distance] for x in self.list_M_distance]

                # self.list_Z_angle = [[[x[0], x[1], x[2], y[0], y[1], y[2], abs(x[3] - y[3])] for y in self.list_N_angle] for x in self.list_M_angle]

                # - Tìm giá trị nhỏ nhất từ các cột của list khoảng cách
                self.min_distance_values, posOf_min_distance_values = self.column_minimums(2, self.list_N_distance, self.list_Z_distance)
                # self.min_angle_values, posOf_min_angle_values = self.column_minimums(3, self.list_N_angle, self.list_Z_angle)

                # print("\nGiá trị khoảng cách nhỏ nhất trong từng cột:")
                # for i in range(len(self.min_distance_values)):
                #     print(f"Cột {i+1}: Giá trị khoảng cách nhỏ nhất = {self.min_distance_values[i]}, Vị trí = {posOf_min_distance_values[i]}")

                # print("\nGiá trị góc nhỏ nhất trong từng cột:")
                # for i in range(len(self.min_angle_values)):
                #     print(f"Cột {i+1}: Giá trị góc nhỏ nhất = {self.min_angle_values[i]}, Vị trí = {posOf_min_angle_values[i]}")

                # - Đưa ra giá trị khoảng cách và góc thoả mãn, kết hợp bươcs trene và bước dươí được
                list_IDref_satify_fromd = []

                for i in range(len(self.min_distance_values)):
                    if self.min_distance_values[i] <= self.distance_matching_error_threshold:
                        row = posOf_min_distance_values[i][0]
                        col = posOf_min_distance_values[i][1]

                        val = self.list_Z_distance[row][col]
                        list_IDref_satify_fromd.append([val[0], val[1], val[2], val[3]])

                # print("Các vị trí gương khớp cua xet khoang cach là:", list_IDref_satify_fromd)
                # print("---")
                # print("Các vị trí gương khớp phát hiện xet khoang cach là:", list_IDref_detected_satify_fromd)
                # print("******\n")
                
                # Chỉ ra ID nào của phát hiện trùng với gương tham chiếu dựa trên khoảng cách
                ls_IDmatch_distance = []
                for i in range(len(list_IDref_satify_fromd)):
                    for j in range(i, len(list_IDref_satify_fromd)):
                        if j != i:
                            idj_ref = list_IDref_satify_fromd[j]
                            idi_ref = list_IDref_satify_fromd[i]
                            id_same_ref = 0
                            id_same_det = 0
                            if idj_ref[0] == idi_ref[0]:
                                id_same_ref = idj_ref[0]

                            if idj_ref[0] == idi_ref[1]:
                                id_same_ref = idj_ref[0]

                            if idj_ref[1] == idi_ref[0]:
                                id_same_ref = idj_ref[1]                            

                            if idj_ref[1] == idi_ref[1]:
                                id_same_ref = idj_ref[1]
                            
                            # -- 
                            if idj_ref[2] == idi_ref[2]:
                                id_same_det = idj_ref[2]

                            if idj_ref[2] == idi_ref[3]:
                                id_same_det = idj_ref[2]

                            if idj_ref[3] == idi_ref[2]:
                                id_same_det = idj_ref[3]                            

                            if idj_ref[3] == idi_ref[3]:
                                id_same_det = idj_ref[3]

                            # -
                            if id_same_ref != 0 and id_same_det != 0:
                                ls_IDmatch_distance.append([id_same_ref, id_same_det])
                                # print(f"Gương tham chiếu thứ {id_same_ref} có thể trùng với gương phát hiện thứ {id_same_det}")

                # - Tính số lượng của từng ID của gương detect vơí gương tham chiếu theo góc 
                id_match_distance = self.count_duplicate_pairs(ls_IDmatch_distance)
                print("Số lượng điểm khớp theo kc là: ", id_match_distance)
                ls_match_distance = []

                # Lặp qua các giá trị của gương detect và tìm phần tử có giá trị lớn nhất và qua ngưỡng nhất định
                for ref in ls_detected_ref:
                    result = self.find_max_element(ref[0], id_match_distance)
                    if result:
                        ls_match_distance.append(result)

                print("Kết quả theo Khoảng cách là:", ls_match_distance)
                print("-----\n")

                # list_IDref_satify_fromtheta = []
                # ls_IDmatch_angle = []
                # for i in range(len(self.min_angle_values)):
                #     if self.min_angle_values[i] <= self.angle_matching_error_threshold:
                #         row = posOf_min_angle_values[i][0]
                #         col = posOf_min_angle_values[i][1]

                #         val = self.list_Z_angle[row][col]
                #         ls_IDmatch_angle.append([val[0], val[3]])
                #         list_IDref_satify_fromtheta.append([val[1], val[2], val[4], val[5]])

                # # print("Các vị trí gương khớp cua xet goc là:", list_IDref_satify_fromtheta)
                # # print("---")

                # # Chỉ ra ID nào của phát hiện trùng với gương tham chiếu dựa trên khoảng cách
                # for i in range(len(list_IDref_satify_fromtheta)):
                #     for j in range(i, len(list_IDref_satify_fromtheta)):
                #         if j != i:
                #             idj_ref = list_IDref_satify_fromtheta[j]
                #             idi_ref = list_IDref_satify_fromtheta[i]
                #             id_same_ref = 0
                #             id_same_det = 0
                #             if idj_ref[0] == idi_ref[0]:
                #                 id_same_ref = idj_ref[0]

                #             if idj_ref[0] == idi_ref[1]:
                #                 id_same_ref = idj_ref[0]

                #             if idj_ref[1] == idi_ref[0]:
                #                 id_same_ref = idj_ref[1]                            

                #             if idj_ref[1] == idi_ref[1]:
                #                 id_same_ref = idj_ref[1]
                            
                #             # -- 
                #             if idj_ref[2] == idi_ref[2]:
                #                 id_same_det = idj_ref[2]

                #             if idj_ref[2] == idi_ref[3]:
                #                 id_same_det = idj_ref[2]

                #             if idj_ref[3] == idi_ref[2]:
                #                 id_same_det = idj_ref[3]                            

                #             if idj_ref[3] == idi_ref[3]:
                #                 id_same_det = idj_ref[3]

                #             # -
                #             if id_same_ref != 0 and id_same_det != 0:
                #                 ls_IDmatch_angle.append([id_same_ref, id_same_det])

                # # - Tính số lượng của từng ID của gương detect vơí gương tham chiếu theo góc 
                # id_match_angle = self.count_duplicate_pairs(ls_IDmatch_angle)
                # # print(id_match)
                # ls_match_angle = []

                # # Lặp qua các giá trị x từ 1 đến 5 và tìm phần tử có giá trị lớn nhất
                # for ref in ls_detectestivietnamd_ref:
                #     result = self.find_max_element(ref[0], id_match_angle)
                #     if result:
                #         ls_match_angle.append(result)

                # print("Kết quả theo góc là:", ls_match_angle)
                # print("--------------------\n")
                  
                # - Kiểm tra xem có id nào cuả map tham chiếu bị trùng hay không ?
                is_not_uniqueID = False
                ls_index_id = []
                for i in range(0, len(ls_match_distance)):
                    for j in range(i, len(ls_match_distance)):
                        if j != i:
                            idj_ref = ls_match_distance[j][0]
                            idi_ref = ls_match_distance[i][0]

                            if idj_ref == idi_ref:
                                is_not_uniqueID = True
                                ls_index_id.append(i)
                                ls_index_id.append(j)
                                break
                    
                if is_not_uniqueID:
                    is_not_uniqueID = False
                    print(f"ID trên map tham chiếu khi khớp gương bị trùng => clear cac id trung {ls_index_id}")
                    del ls_match_distance[ls_index_id[0]]
                    del ls_match_distance[ls_index_id[1]-1]

                if len(ls_match_distance) < 3:
                    print("Số gương khớp < 3")
                else:
                    # # -- Use SVD for find pos of robot on map
                    # # - Step 0: Lọc map tham chiếu khớp trong map gốc
                    n = len(ls_reference_ref)
                    ls_reference_refFilter = []
                    for ref in ls_reference_ref:
                        for id_ref in ls_match_distance:
                            if ref[0] == id_ref[0]:
                                ls_reference_refFilter.append([ref[1], ref[2]])

                    n = len(ls_detected_ref)
                    ls_detected_refFilter = []
                    for ref in ls_detected_ref:
                        for id_ref in ls_match_distance:
                            if ref[0] == id_ref[1]:
                                ls_detected_refFilter.append([ref[1], ref[2]])
                    
                    # -- Step 1: find pose of robot
                    xr, yr, phiR = self.SVD_algorithm(ls_reference_refFilter, ls_detected_refFilter)
                    print("num ref matching: ", len(ls_detected_refFilter), "| Pose: ", xr, yr, degrees(phiR))

                    # # - send transform to rviz
                    self.translation = (xr, yr, 0.0)
                    self.quanternion = self.euler_to_quaternion(phiR)
                    self.br.sendTransform(self.translation, self.quanternion, rospy.Time.now(), self.ref_frame, self.origin_frame)

                    # -- publish data
                    self.pub_lidar_pose(self.lidar_pose_pub, xr, yr, phiR, len(ls_match_distance))

                    # -- debug
                    # delta_phi = self.pre_phiR - phiR
                    
                    # if abs(degrees(delta_phi)) > 45 and abs(degrees(phiR)) < 179:   # stop node for debug
                    #     self.step += 1
                    #     self.pre_phiR = phiR

                    #     if self.step == 2:
                    #         self.process = -1
                    #         self.pre_phiR = phiR
                    #         # print("num ref matching: ", len(ls_detected_refFilter), "| Pose: ", xr, yr, degrees(phiR))

                    #         # print("Số lượng điểm khớp theo kc là: ", id_match_distance)
                    #         # print("Kết quả theo Khoảng cách là:", ls_match_distance)
                    #         print("-----\n")
                    #         print("pause program!!!!")
                    #         continue

                # k = False
                # if self.data_rawvel.twist.twist.linear.x != 0 or self.data_rawvel.twist.twist.angular.z != 0:
                #     if k:
                #         k = False
                #         print("---------AGV bắt đầu di chuyển-------------------")
                # else:
                #     k = False

                # print("####################################################################")

                # -- move to next step
                self.process = 1

                # - Tính svd from equation 
                # - Step 1: Tìm trung bình toạ độ của các điểm gương phát hiện và tham chiếu
                # sum_detected_x = 0
                # sum_detected_y = 0
                # sum_reference_x = 0
                # sum_reference_y = 0

                # for i in range(0, n_reference):
                #     sum_detected_x += ls_detected_refFilter[i][0]
                #     sum_detected_y += ls_detected_refFilter[i][1]

                #     sum_reference_x += ls_reference_refFilter[i][0]
                #     sum_reference_y += ls_reference_refFilter[i][1]

                # mean_detected = [[sum_detected_x/n_reference], [sum_detected_y/n_reference]]
                # mean_reference = [[sum_reference_x/n_reference], [sum_reference_y/n_reference]]           

                # # - Step 2: Tìm sai số vị trí của các điểm gương với điểm trung bình
                # ls_tolerance_detected_ref = []
                # ls_tolerance_reference_ref = []
                        
                # for i in range(0, n_reference):
                #     delta_x = ls_detected_refFilter[i][0] - mean_detected[0][0]
                #     delta_y = ls_detected_refFilter[i][1] - mean_detected[1][0]
                #     ls_tolerance_detected_ref.append([delta_x, delta_y])

                #     delta_x = ls_reference_refFilter[i][0] - mean_reference[0][0]
                #     delta_y = ls_reference_refFilter[i][1] - mean_reference[1][0]
                #     ls_tolerance_reference_ref.append([delta_x, delta_y])

                # # - tìm ma trận hiệp phương sai H
                # H = self.covariance_matrix(ls_tolerance_detected_ref, ls_tolerance_reference_ref)

                # # print("Ma trận hiệp phương sai H là:")
                # # print(H)
                # # print("--------------------------------")

                # # # - Tìm các ma trận kết quả từ thuật toán svd
                # U, V = self.svd_decomposition(H)
                # U_T = self.transpose(U)
                # # print("Ma trận kết quả sau SVD là:")
                # # print(U)
                # # print("--------------------------------")
                # # print("Ma trận chuyển vị U_T là:")
                # # print(U_T)
                # # print("--------------------------------")
                # # print(V)
                # # print("--------------------------------")

                # # # Tính ma trận quay R
                # R = [[0, 0], [0, 0]]
                # for i in range(2):
                #     for j in range(2):
                #         R[i][j] = V[i][0] * U_T[0][j] + V[i][1] * U_T[1][j]

                # # R = [[sum(U_T[i][k] * V[k][j] for k in range(2)) for j in range(2)] for i in range(2)]
                # #                 
                # # # Kiểm tra tính hợp lệ của ma trận quay R
                # # det_R = R[0][0] * R[1][1] - R[0][1] * R[1][0]

                # # if det_R < 0:
                # #     print("Phát hiện ma trận phản chiếu, điều chỉnh V")
                # #     V[1][0] *= -1
                # #     V[1][1] *= -1
                # #     R[0][1] *= -1  # Điều chỉnh trực tiếp
                # #     R[1][0] *= -1  # Điều chỉnh trực tiếp

                # print("Ma trận quay R là:")
                # print(R)
                # print("--------------------------------")

                # # # Tính vector tịnh tiến t
                # # t = [[0, 0], [0, 0]]
                # # for i in range(2):
                # #     for j in range(2):
                # #         t[i][j] = mean_reference - R[i][j] * mean_detected   # Nhân với k, rồi cộng với c
                # # # t = [mean_reference[i] - sum(R[i][j] * mean_detected[j] for j in range(2)) for i in range(2)]
                # R_neg = [[-R[i][j] for j in range(2)] for i in range(2)]
                # R_multi = [[0], [0]]  # Khởi tạo ma trận kết quả 2x1

                # for i in range(2):
                #     R_multi[i][0] = R_neg[i][0] * mean_detected[0][0] + R_neg[i][1] * mean_detected[1][0]

                # t = [[R_multi[i][0] + mean_reference[i][0]] for i in range(2)]
                # print("Ma trận tịnh tiến t là:")
                # print(t)
                # print("--------------------------------")

                # # # - tìm ra vị trí cuả robot trong hệ toàn cục
                # pose_rb = self.global_position([0,0], R, t)
                # angle_rb = self.compute_robot_angle(R)

                # print("num ref matching: ", len(ls_detected_refFilter), "| Pose: ", pose_rb[0], pose_rb[1], angle_rb)                    

            # - tìm ra vị trí gương khớp trên map tham chiếu vơí biến d, theta và trọng số w => test sau
            elif self.process == 15:
                if self.is_rawReflector == False:
                    self.rate.sleep()
                    continue
                    
                self.is_rawReflector = False

                # -- đưa ra khoảng tìm kiếm tiếp theo 
                d_near, d_far = self.find_min_max(self.ls_predict_ref)
                print(f"Khoảng tìm kiếm là từ {d_near} tới {d_far}")

                ls_detected_ref = []

                for index, ref in enumerate(self.data_raw_reflector.points):
                    # -- create raw reflector pos
                    theta_raw = atan2(ref.y, ref.x)
                    d_raw = ref.x*cos(theta_raw)
                    # if d_near <= d_raw <= d_far:
                    ls_detected_ref.append([(index + 1), d_raw, theta_raw])
                
                print(f"Chiều dài của list gương phát hiện trong khoảng là: {ls_detected_ref}")

                # -- create reference reflector pos in search distance
                ls_reference_ref = []
                for index, ref in enumerate(self.data_map.points):
                    theta_ref = atan2(ref.y, ref.x)
                    d_ref = ref.x*cos(theta_ref)

                    if d_near <= d_ref <= d_far:
                        ls_reference_ref.append([(index + 1), d_ref, theta_ref])
                            
                print(f"Chiều dài của list gương tham chiếu trong khoảng là: {ls_reference_ref}")

                # -- Tìm sai số khoảng cách và góc giữa gương tham chiếu và gương phát hiện
                ls_xichma_d = []
                ls_xichma_a = []
                # ls_xichma_index = []

                numberOflist_detected_ref = len(ls_detected_ref)
                numberOflist_reference_ref = len(ls_reference_ref)

                for i in range(0, numberOflist_reference_ref):
                    ls_xichma_d_unit = []
                    ls_xichma_a_unit = []
                    ls_xichma_index_unit = []

                    for j in range(0, numberOflist_detected_ref):
                        xichma_d = abs(ls_detected_ref[j][1] - ls_reference_ref[i][1])
                        xichma_a = abs(ls_detected_ref[j][2] - ls_reference_ref[i][2])

                        ls_xichma_d_unit.append(xichma_d)
                        ls_xichma_a_unit.append(xichma_a)
                        # ls_xichma_index_unit.append([j+1, i+1])
                    
                    ls_xichma_d.append(ls_xichma_d_unit)
                    ls_xichma_a.append(ls_xichma_a_unit)
                    # ls_xichma_index.append(ls_xichma_index_unit)

                print(f"Kích thước của ma trận sai số là: {self.get_matrix_size(ls_xichma_d)}")
                
                # - Tìm ma trận trọng số giữa 2 sai số
                ls_w = self.multiply_matrices(ls_xichma_d, ls_xichma_a)

                print(f"Kích thước của ma trận trọng số là: {self.get_matrix_size(ls_w)}")

                # Tìm giá trị nhỏ nhất và vị trí
                min_vals, min_positions = self.find_min_in_columns(ls_w)

                # In kết quả
                print("Giá trị nhỏ nhất ở mỗi cột:", min_vals)
                print("Vị trí của các giá trị nhỏ nhất:", min_positions)

                # -- move to next step
                self.process = -1

            self.rate.sleep()

def main():
    print ("--- Run navigation mode---")
    program = Navigation_mode()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




