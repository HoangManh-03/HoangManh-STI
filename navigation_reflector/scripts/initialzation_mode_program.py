#!/usr/bin/env python3
"""
*** infomation

*** Task Description: 
    + Tìm ra vị trí cuả robot khi chưa có tốc độ

*** Need to do
    + Fix lỗi khi robot start ngay taị gốc 0 cuả map tham chiêú khi khởi động lên
    
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
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from navigation_reflector.msg import Raw_reflector, Init_mode

from itertools import combinations

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class reflectorMap():
    def __init__(self, _id = 0, _x = 0., _y = 0.):
        self.id = _id
        self.x = _x
        self.y = _y

class Initialization_mode():
    def __init__(self):
        rospy.init_node('initialization_node', anonymous = True)
        self.rate = rospy.Rate(50)

        # -- ros pub && sub
        # rospy.Subscriber("/Keyboard_cmd", String, self.callback_KeyboardCmd)
        # self.keyboad_cmd = String()        

        rospy.Subscriber('/info_raw_reflector', Raw_reflector, self.callback_infoRawReflector, queue_size = 10)
        self.data_raw_reflector = Raw_reflector()
        self.is_rawReflector = False

        self.initmode_respond_pub = rospy.Publisher('/init_mode_respond', Init_mode, queue_size=10)
        self.initmode_respond_data = Init_mode()

        rospy.Subscriber('/init_mode_query', Int8, self.callback_initmode_query, queue_size = 10)
        self.data_initmode_query = Int8()
        # self.is_rawReflector = False

        # self.initmode_rviz_pub = rospy.Publisher('/init_mode_rviz', PoseStamped, queue_size=10)
        # self.initmode_rviz_data = PoseStamped()

        # -- global variables
        self.process = 0

        # self.dir_mapReflector = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/map_reflector.json'  # map directory
        # self.lsReflector_inMap = []  # list reflectỏr and its ID
        rospy.Subscriber('/map', Raw_reflector, self.callback_map, queue_size = 10)
        self.data_map = Raw_reflector()
        self.is_recv_mapdata = False

        # arr_kdtree_refInMap = []
        # arr_edge = []

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
        self.distance_matching_error_threshold = 0.06  # (zf = 2 cm )
        self.angle_matching_error_threshold = PI/180     # (gf = 1 độ)
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

        # # -- Khai báo tf -- 
        self.br = tf.TransformBroadcaster()
        self.ref_frame = "world"
        self.origin_frame = "map"

        self.translation = (0,0,0)
        self.quanternion = quaternion_from_euler(0, 0, 0)

    def callback_infoRawReflector(self, data):
        self.data_raw_reflector = data
        self.is_rawReflector = True

    def callback_initmode_query(self, data):
        self.data_initmode_query = data

    def callback_map(self, data):
        self.data_map = data
        self.is_recv_mapdata = True

    # def callback_KeyboardCmd(self, data):
    #     self.keyboad_cmd = data

    # def remove_absolute_duplicates(self, lst):
    #     seen = set()
    #     result = []
        
    #     for num in lst:
    #         if abs(num) not in seen:
    #             seen.add(abs(num))
    #             result.append(num)
        
    #     return result

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

    def euler_to_quaternion(self, euler):
        quat = Quaternion()
        odom_quat = quaternion_from_euler(0, 0, euler)
        quat.x = odom_quat[0]
        quat.y = odom_quat[1]
        quat.z = odom_quat[2]
        quat.w = odom_quat[3]
        return quat

    def common_elements(self, list1, list2):
        # Chuyển danh sách nhỏ thành set để tìm kiếm nhanh hơn
        if len(list1) > len(list2):
            list1, list2 = list2, list1  # Đảm bảo list1 luôn nhỏ hơn

        set_list2 = set(list2)
        return [x for x in list1 if x in set_list2]

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
        x_l, y_l = np.array([0, 0])

        # Tính tọa độ của robot trong hệ tọa độ toàn cục (với chuyển vị)
        robot_position_global = (np.linalg.inv(R) @ (-t)).T
        
        # Tính góc quay theta_g
        theta_g = -np.arctan2(R[1, 0], R[0, 0])

        return robot_position_global[0], robot_position_global[1], theta_g

        print(f"Ma trận quay R:\n{R}")
        print(f"Vector tịnh tiến t: {t}")
        print(f"Tọa độ trong hệ toàn cục: {robot_position_global}")
        print(f"Góc quay trong hệ toàn cục: {theta_g} độ")

    def run(self):
        while not rospy.is_shutdown():
            # -- Tính toán các thông tin của map gương tham chiếu
            if self.process == 0:
                #  -- Load map from json file

                # try:
                #     with open(self.dir_mapReflector, 'r') as file:
                #         data = json.load(file)
                #         for index, ref in enumerate(data["info"]):
                #             self.lsReflector_inMap.append(reflectorMap(ref["id"], ref["x"], ref["y"]))

                # except Exception as e:
                #     print("Read json file fail: ", e)
                #     return
                if self.is_recv_mapdata == True:
                    # -- Lấy toạ độ gương lọc
                    self.lsReflector_inMap = []
                    for index, ref in enumerate(self.data_map.points):
                        self.lsReflector_inMap.append([index + 1, ref.x, ref.y])

                    # print("Mảng gương có giá trị là: ", self.lsReflector_inMap[0].id)
                    # print("Mảng gương có chiều dài là: ", self.lsReflector_inMap)

                    # -- find spec detail of reflector map
                    # - Step 1: Tìm khoảng cách giữa các gương
                    n = len(self.lsReflector_inMap)
                    list_M_distance = []
                    self.list_M_distance = []

                    for i in range(0, n):
                        for j in range(0, n):
                            if j != i:
                                dx = self.lsReflector_inMap[j][1] - self.lsReflector_inMap[i][1]
                                dy = self.lsReflector_inMap[j][2] - self.lsReflector_inMap[i][2]
                                d = sqrt(dx*dx + dy*dy)
                                list_M_distance.append([i+1, j+1, d])
                    
                    # print("Mảng thông số các gương raw là: ", list_M_distance)

                    # Xoá các điểm có giá trị tuyệt đôí bằng nhau
                    self.list_M_distance = self.remove_absolute_duplicates(2, list_M_distance)
                    # print("Mảng thông số khoảng cách các gương tham chiếu là: ", self.list_M_distance)    ## số lượng là nC2 giá trị

                    # - Step 2: Tìm các góc của hệ gương
                    # Tìm tổ hợp các tổ hợp chập 3 cuả hệ gương
                    n = len(self.lsReflector_inMap)
                    combs = list(combinations(self.lsReflector_inMap, 3))
                    # In danh sách các tổ hợp
                    # for comb in combs:
                    #     print(comb[1].id)

                    # Tổng số tổ hợp
                    # print("Số lượng tổ hợp không kể thứ tự:", len(combs))

                    # Tìm góc của các tổ hợp đó
                    self.list_M_angle = []
                    for comb in combs:
                        list_M_comb = []
                        for i in range(0, 3):
                            for j in range(0, 3):             
                                if j != i:
                                    dx = comb[j][1] - comb[i][1]
                                    dy = comb[j][2] - comb[i][2]
                                    d = sqrt(dx*dx + dy*dy)
                                    list_M_comb.append([comb[0][0],comb[1][0], comb[2][0], d])
                        
                        list_M_comb_final = self.remove_absolute_duplicates(3, list_M_comb)
                        # print(list_M_comb_final)

                        a = list_M_comb_final[0][3]
                        b = list_M_comb_final[1][3]
                        c = list_M_comb_final[2][3]

                        theta_r = acos((a*a + b*b - c*c)/(2*a*b))
                        self.list_M_angle.append([comb[0][0],comb[1][0], comb[2][0], theta_r])

                    # print(f'Mảng thông số góc giữa các gương tham chiếu với {len(self.list_M_angle)} tổ hợp là: {self.list_M_angle}')  # số lượng là n!/(3! * (n-3)!)
                        
                    # - move to next step
                    self.process = 1

            # -- Kiểm tra xem đã nhận được list các điểm gương hiện taị chưa
            elif self.process == 1:
                if self.is_rawReflector:
                    self.process = 2

            # -- Tính toán spec của lượng gương detect được
            elif self.process == 2:
                if self.is_rawReflector == False:
                    self.rate.sleep()
                    continue
                
                self.is_rawReflector = False

                # -- kiểm tra số lượng gương quét được
                num_reflector = len(self.data_raw_reflector.points)
                if num_reflector < 3:
                    print("The number of mirrors is less than 3")
                    self.rate.sleep()
                    continue

                # -- Lấy toạ độ gương lọc
                ls_refFilter = []
                for index, ref in enumerate(self.data_raw_reflector.points):
                    ls_refFilter.append([index+ 1, ref.x, ref.y])

                # if self.num_refStart == -1:
                #     self.num_refStart = len(ls_refFilter)

                #     for r in range(self.num_refStart):
                #         self.data_store.append([0., 0.])

                # # -- lấy toạ độ trung bình cho các điểm gương
                # if self.num_refStart != len(ls_refFilter):
                #     print("số lượng gương các lần đọc không phù hơp")
                #     return

                # numave = 20
                # if self.num_getPoseReflector < numave:
                #     # print("Lấy mẫu gương lần thứ {x}".format(x = self.num_getPoseReflector))
                #     for index, ref in enumerate(ls_refFilter):
                #         self.data_store[index][0] += ref[0]
                #         self.data_store[index][1] += ref[1]

                #     self.num_getPoseReflector += 1
                #     self.rate.sleep()
                #     continue

                # # -- Tính trung bình 20 lần đọc tạo độ gương
                # self.ls_refFilter_average = []
                # for index, ave_ref in enumerate(self.data_store):
                #     self.ls_refFilter_average.append([index + 1, ave_ref[0]/numave, ave_ref[1]/numave])

                self.ls_refFilter_average = ls_refFilter
                # print("Giá trị trung bình của các gương mới là: ",ls_refFilter_average)
                # print("##################################################################")

                # -- Step 1: Tính list khoảng cách các điểm gương
                n = len(self.ls_refFilter_average)
                list_N_distance = []
                self.list_N_distance = []
                for i in range(0, n):
                    for j in range(0, n):
                        if j != i:
                            dx = self.ls_refFilter_average[j][1] - self.ls_refFilter_average[i][1]
                            dy = self.ls_refFilter_average[j][2] - self.ls_refFilter_average[i][2]
                            d = sqrt(dx*dx + dy*dy)
                            list_N_distance.append([self.ls_refFilter_average[i][0], self.ls_refFilter_average[j][0], d])
                
                # print("Mảng thông số các gương raw là: ", list_M_distance)

                # Xoá các điểm có giá trị tuyệt đôí bằng nhau
                self.list_N_distance = self.remove_absolute_duplicates(2, list_N_distance)
                ######### print("Mảng thông số khoảng cách giưã các gương phát hiện là: ", self.list_N_distance)

                # -- Step 2: Tính list góc giữa các điểm gương
                n = len(self.ls_refFilter_average)
                combs = list(combinations(self.ls_refFilter_average, 3))
                # In danh sách các tổ hợp
                # for comb in combs:
                #     print(comb[1].id)

                # Tổng số tổ hợp
                # print("tổ hợp không kể thứ tự:", combs)

                # Tìm góc của các tổ hợp đó
                self.list_N_angle = []
                for comb in combs:
                    list_N_comb = []
                    for i in range(0, 3):
                        for j in range(0, 3):             
                            if j != i:
                                dx = comb[j][1] - comb[i][1]
                                dy = comb[j][2] - comb[i][2]
                                d = sqrt(dx*dx + dy*dy)
                                list_N_comb.append([comb[0][0], comb[1][0], comb[2][0], d])
                    
                    list_N_comb_final = self.remove_absolute_duplicates(3, list_N_comb)
                    # print(list_M_comb_final)

                    a = list_N_comb_final[0][3]
                    b = list_N_comb_final[1][3]
                    c = list_N_comb_final[2][3]

                    theta_r = acos((a*a + b*b - c*c)/(2*a*b))
                    self.list_N_angle.append([comb[0][0],comb[1][0], comb[2][0], theta_r])

                ######### print(f'Mảng thông số góc giữa các gương phát hiện với {len(self.list_N_angle)} tổ hợp là: {self.list_N_angle}')
                # - move to next step
                self.process = 3
            
            # -- Tìm ra robot đang nằm gần với đám gương nào
            elif self.process == 3:
                # - Tìm sự khác nhau giữa 2 map gương tham chiếu và map gương detect được
                self.list_Z_distance = [[[x[0],x[1], y[0], y[1],abs(x[2] - y[2])] for y in self.list_N_distance] for x in self.list_M_distance]

                self.list_Z_angle = [[[x[0], x[1], x[2], y[0], y[1], y[2], abs(x[3] - y[3])] for y in self.list_N_angle] for x in self.list_M_angle]

                # print("Ma trận khoảng cách đầu ra:")
                # for row in self.list_Z_distance:
                #     print(row)

                # print("Ma trận góc đầu ra:")
                # for row in self.list_Z_angle:
                #     print(row)

                print("##################################################################")

                # - Tìm giá trị nhỏ nhất từ các cột của list khoảng cách
                self.min_distance_values, posOf_min_distance_values = self.column_minimums(2, self.list_N_distance, self.list_Z_distance)
                self.min_angle_values, posOf_min_angle_values = self.column_minimums(3, self.list_N_angle, self.list_Z_angle)

                # print("\nGiá trị khoảng cách nhỏ nhất trong từng cột:")
                # for i in range(len(self.min_distance_values)):
                #     print(f"Cột {i+1}: Giá trị khoảng cách nhỏ nhất = {self.min_distance_values[i]}, Vị trí = {posOf_min_distance_values[i]}")

                # print("\nGiá trị góc nhỏ nhất trong từng cột:")
                # for i in range(len(self.min_angle_values)):
                #     print(f"Cột {i+1}: Giá trị góc nhỏ nhất = {self.min_angle_values[i]}, Vị trí = {posOf_min_angle_values[i]}")

                # - Đưa ra giá trị khoảng cách và góc thoả mãn
                self.posOf_min_distance_values = []
                self.posOf_min_angle_values = []

                for i in range(len(self.min_distance_values)):
                    if self.min_distance_values[i] <= self.distance_matching_error_threshold:
                        self.posOf_min_distance_values.append([posOf_min_distance_values[i][0], posOf_min_distance_values[i][1]])

                for i in range(len(self.min_angle_values)):
                    if self.min_angle_values[i] <= self.angle_matching_error_threshold:
                        self.posOf_min_angle_values.append([posOf_min_angle_values[i][0], posOf_min_angle_values[i][1]])

                # - Đưa ra các ID gương nào trên map tham chiêú thoả mãn
                list_IDref_referential_satify_fromd = []
                list_IDref_referential_satify_fromtheta = []
                list_IDref_referential_satify = []
                self.list_IDref_referential_satify = []

                for pos in self.posOf_min_distance_values:
                    row = pos[0]
                    col = pos[1]

                    val = self.list_Z_distance[row][col]

                    list_IDref_referential_satify_fromd.append(val[0])
                    list_IDref_referential_satify_fromd.append(val[1])
                
                print("\nCác vị trí gương khớp chung cua xet khoang cach là:", list_IDref_referential_satify_fromd)
                
                # -- ko cần góc đúng ko, vì góc có thể được suy ra từ cạnh mà
                for pos in self.posOf_min_angle_values:
                    row = pos[0]
                    col = pos[1]

                    val = self.list_Z_angle[row][col]

                    list_IDref_referential_satify_fromtheta.append(val[0])
                    list_IDref_referential_satify_fromtheta.append(val[1])
                    list_IDref_referential_satify_fromtheta.append(val[2])
                
                print("\nCác vị trí gương khớp chung cua xet goc là:", list_IDref_referential_satify_fromtheta)

                # - Tìm các phần tử chung của 2 list trên.
                # list_IDref_referential_satify = self.common_elements(list_IDref_referential_satify_fromd, list_IDref_referential_satify_fromtheta)
                # - Xoá các giá trị bị lặp lại
                self.list_IDref_referential_satify = self.remove_absolute_duplicates(0, list_IDref_referential_satify_fromd)

                # -- Sắp xếp laị các ID từ bé tới lớn
                self.list_IDref_referential_satify.sort()

                print("\nCác vị trí gương khớp trong map tham chiêú là:", self.list_IDref_referential_satify)
                
                # - move to next step
                self.process = 4

            # - Đưa ra vị trí và góc quay của robot so với vị trí ban đâù
            elif self.process == 4:
                # Tách gương khớp trong map tham chiêú ra
                list_reference_ref_x = []
                list_reference_ref_y = []

                for id_ref in self.list_IDref_referential_satify:
                    for ref_reflector in self.lsReflector_inMap:
                        if ref_reflector[0] == id_ref:
                            list_reference_ref_x.append(ref_reflector[1])
                            list_reference_ref_y.append(ref_reflector[2])
                
                # print(f"Chiều dài của list gương tham chiêú là : {len(list_reference_ref_x)} so với list gương detect là: {len(self.ls_refFilter_average)}" )
                
                n_reference = len(list_reference_ref_x)
                n_detected = len(self.ls_refFilter_average)

                if n_detected > n_reference:
                    print(f"Số lượng detect được {n_detected} đang lớn hơn số lượng điểm map khop {n_reference}. Cần quét lại map")
                
                else:
                    # - Tìm khoảng dịch chuyển
                    sum_dx = 0 
                    sum_dy = 0
                    
                    for index, detected_ref in enumerate(self.ls_refFilter_average):
                        a =  list_reference_ref_x[index] - detected_ref[1]
                        b =  list_reference_ref_y[index] - detected_ref[2]
                        # print(f"Độ dịch chuyển cuả gương số {detected_ref[0]} là: {a} và {b}")
                        sum_dx += a
                        sum_dy += b
                        
                    self.displacement_dx = sum_dx/ len(self.ls_refFilter_average)
                    self.displacement_dy = sum_dy/ len(self.ls_refFilter_average)
                    print(f"Vị trí khởi tạo của robot là: {self.displacement_dx} và {self.displacement_dy}")

                    # - Tìm hướng của robot so vơí vị trí ban đầu
                    m = self.displacement_dx
                    n = self.displacement_dy

                    if m == 0 and n == 0:
                        print("Không có dịch chuyển. Góc theta không xác định.")
                    elif m == 0:
                        if n > 0:
                            theta_degrees = 90
                        else:
                            theta_degrees = 270
                        print(f"Hướng của vị trí mới O' so với gốc O: {theta_degrees} độ")
                    elif n == 0:
                        if m > 0:
                            theta_degrees = 0
                        else:
                            theta_degrees = 180
                        print(f"Hướng của vị trí mới O' so với gốc O: {theta_degrees} độ")
                    else:
                        # Tính góc theta (radian)
                        theta_radians = atan2(n, m)
                        # Chuyển đổi góc từ radian sang độ
                        theta_degrees = degrees(theta_radians)
                        # Đảm bảo góc nằm trong khoảng [0, 360)
                        if theta_degrees < 0:
                            theta_degrees += 360
                            theta_radians += 2*PI
                        # In kết quả
                        print(f"Hướng của vị trí mới O' so với gốc O: {theta_degrees:.2f} độ")

                    # -- Publish data
                    self.initmode_respond_data.list_id = self.list_IDref_referential_satify
                    self.initmode_respond_data.init_point.position.x = self.displacement_dx
                    self.initmode_respond_data.init_point.position.y = self.displacement_dy
                    self.initmode_respond_data.init_point.position.z = 0.0

                    self.initmode_respond_data.init_point.orientation = self.euler_to_quaternion(theta_radians)
                
                    # -
                    # self.initmode_rviz_data.header.stamp = rospy.Time.now()
                    # self.initmode_rviz_data.header.frame_id = 'world'
                    # self.initmode_rviz_data.pose.position.x = self.displacement_dx
                    # self.initmode_rviz_data.pose.position.y = self.displacement_dy
                    # self.initmode_rviz_data.pose.position.z = 0

                    # self.initmode_rviz_data.pose.orientation = self.euler_to_quaternion(theta_radians)

                    # self.initmode_rviz_pub.publish(self.initmode_rviz_data)
                
                # else:
                #     print("Chiều dài 2 list này ko bằng nhau")
                #     print("--------------------------------------------")

                # - move to next step
                self.process = 5
            
            elif self.process == 5: 
                self.initmode_respond_pub.publish(self.initmode_respond_data)

                # -- Cần chạy lại chế độ init mode
                if self.data_initmode_query.data == 1:
                    self.process = 1

            self.rate.sleep()

def main():
    print ("--- Run init mode---")
    program = Initialization_mode()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




