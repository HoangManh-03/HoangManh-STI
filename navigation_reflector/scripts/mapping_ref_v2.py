#!/usr/bin/env python3
"""
*** Task Description: 
    + Thêm điểm mới vào map tham chiếu => lưu map tham chiếu

"""


import signal
import sys
import os
from datetime import datetime

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Int8
 
from math import atan2, sin, cos, sqrt, fabs, degrees, isnan, radians
from math import pi as PI
import rospy
import time
import copy

import json

import numpy as np
from scipy.spatial import KDTree
from scipy.optimize import minimize

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Pose
from navigation_reflector.msg import *

from itertools import combinations

from collections import deque

# from triangleAlgorithm import triangleAlgorithm
# from svd_algorithm import SVD_algorithm

class reflectorMap():
    def __init__(self, _id = 0, _x = 0., _y = 0.):
        self.id = _id
        self.x = _x
        self.y = _y

class dataReflector():
    def __init__(self, _x = 0., _y = 0., _dis = 0., _angle = 0.):
        self.map = reflectorMap()
        self.x_relative = _x
        self.y_relative = _y
        self.distance_reflector = _dis
        self.angle_reflector = _angle

class ReflectorMatching():
    def __init__(self, ls_reflector_):
        self.ls_reflector = ls_reflector_

        self.ls_reflectorMap = []

class DetectReflector():
    def __init__(self):
        rospy.init_node('mapping_reflector', anonymous = True)
        self.rate = rospy.Rate(20)

        # -- load data map
        self.dir_mapReflector = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/map_reflector_archie.json'
        self.lsReflector_inMap = []

        try:
            with open(self.dir_mapReflector, 'r') as file:
                data = json.load(file)
                for ref in data["info"]:
                    self.lsReflector_inMap.append(reflectorMap(ref["id"], ref["x"], ref["y"]))
                    # arr_kdtree_refInMap.append([ref["id"], ref["x"], ref["y"]])

                # arr_kdtree_refInMap = np.array(arr_kdtree_refInMap)
                # self.kdtree = KDTree(arr_kdtree_refInMap[:, 1:3])

        except Exception as e:
            print("Read json file fail: ", e)

        self.have_map_old = False
        if len(self.lsReflector_inMap) > 2:
            self.have_map_old = True
        
        # -- param
        self.x_init = rospy.get_param('~x_init', 0.)
        self.y_init = rospy.get_param('~y_init', 0.)
        self.r_init = rospy.get_param('~r_init', 0.)

        # -- param
        # self.x_init = rospy.get_param('~x_init', 2.074)
        # self.y_init = rospy.get_param('~y_init', 2.011)
        # self.r_init = rospy.get_param('~r_init', radians(38.14))

        # self.use_sensorPose = rospy.get_param('~use_sensorPose', False)

        # -------- Topic Sub -------- #
        # --
        rospy.Subscriber('/r2000_reflectors', R2000_reflectors, self.callback_infoRawReflector, queue_size = 10)
        self.data_raw_reflector = R2000_reflectors()
        self.is_rawReflector = False

        rospy.Subscriber('/r2000_data', R2000_data, self.callback_r2000data, queue_size = 10)
        self.data_r2000 = R2000_data()
        self.is_r2000 = False

        # if self.use_sensorPose:
        #     rospy.Subscriber('/sensorPose_reference', PoseStamped, self.callback_poseSensorReference, queue_size = 10)
        #     self.is_poseSensorRef = False
        #     self.poseSensorRef = Pose()

        # -- 
        self.process = 0
        self.status_match = False

        # -- 
        self.num_refStart = -1
        self.num_getPoseReflector = 0
        self.save_numRef = -1
        self.data_store = []

        # -
        self.step = 0
        self.num_poseStart = 0
        self.num_getPoseRobot = 0
        self.data_storePose_x = []
        self.data_storePose_y = []
        self.data_storePose_phi = []
        self.num_reflector = 0
        # -- 
        self.savePose_robot = [0. ,0. ,0.]
        self.num_savePoseRobot = 0

    def callback_infoRawReflector(self, data):
        self.data_raw_reflector = data
        self.is_rawReflector = True
        # print("im here")

    def callback_r2000data(self, data):
        self.data_r2000 = data
        self.is_r2000 = True
    
    def distance_squared(self, point):
        return point.x**2 + point.y**2
    
    def calculate_reflector_position_in_map(self, x_ss, y_ss, r_ss, x_ref, y_ref):
        # Tính toán ma trận quay thủ công (cos(R), sin(R))
        cos_R = cos(r_ss)
        sin_R = sin(r_ss)

        x_m = x_ss + (cos_R*x_ref - sin_R*y_ref)
        y_m = y_ss + (sin_R*x_ref + cos_R*y_ref)

        return x_m, y_m
    
    def add_reflectorMap(self, ls_ref):
        # -- lấy id gương lớn nhất
        id_refAdd = 0
        for refMap in self.lsReflector_inMap:
            if refMap.id > id_refAdd:
                id_refAdd = refMap.id
        # -- 
        # -- thêm gương vào list cũ
        for ref in ls_ref:
            id_refAdd += 1
            self.lsReflector_inMap.append(reflectorMap(id_refAdd, ref[0], ref[1]))

        # -- ghi file
        now = datetime.now().strftime("%Y-%m-%d %H:%M")
        json_out = {"name": 'map_reflector', "time_update": str(now), "info": []}

        for info_ref in self.lsReflector_inMap:
            json_infoRef = {"id": info_ref.id, "x": info_ref.x, "y": info_ref.y}
            json_out["info"].append(json_infoRef)

        temp_file_path = self.dir_mapReflector + '.tmp' 
        try:
            with open(temp_file_path, 'w', encoding='utf-8') as temp_file:
                json.dump(json_out, temp_file, ensure_ascii=False, indent=4)
                temp_file.flush()
                os.fsync(temp_file.fileno())
            os.rename(temp_file_path, self.dir_mapReflector)

        except Exception as e:
            print("Write json file backup error", e)

    def add_reflector(self, ref):
        # -- lấy id gương lớn nhất
        id_refAdd = 0
        for refMap in self.lsReflector_inMap:
            if refMap.id > id_refAdd:
                id_refAdd = refMap.id
        # -- 
        # -- thêm gương vào list cũ
        id_refAdd += 1
        self.lsReflector_inMap.append(reflectorMap(id_refAdd, ref[0], ref[1]))

        # -- ghi file
        now = datetime.now().strftime("%Y-%m-%d %H:%M")
        json_out = {"name": 'map_reflector', "time_update": str(now), "info": []}

        for info_ref in self.lsReflector_inMap:
            json_infoRef = {"id": info_ref.id, "x": info_ref.x, "y": info_ref.y}
            json_out["info"].append(json_infoRef)

        temp_file_path = self.dir_mapReflector + '.tmp' 
        try:
            with open(temp_file_path, 'w', encoding='utf-8') as temp_file:
                json.dump(json_out, temp_file, ensure_ascii=False, indent=4)
                temp_file.flush()
                os.fsync(temp_file.fileno())
            os.rename(temp_file_path, self.dir_mapReflector)

        except Exception as e:
            print("Write json file backup error", e)

    def run(self):
        while not rospy.is_shutdown():
            if self.process == 0:
                if self.is_rawReflector and self.is_r2000 == True:
                    self.process = 1

            elif self.process == 1:

                # -- Trường hợp không có gương trong map 
                if self.have_map_old == False:
                    if self.is_rawReflector == False:
                        self.rate.sleep()
                        continue
                    
                    self.is_rawReflector = False

                    # -- kiểm tra số lượng gương quét được
                    # num_reflector = len(self.data_raw_reflector.reflectors)
                    # if num_reflector < 3:
                    #     print("The number of mirrors is less than 3")
                    #     self.rate.sleep()
                    #     continue
                    
                    # -- 
                    # -- Lấy toạ độ gương lọc
                    ls_refFilter = []
                    for ref in self.data_raw_reflector.reflectors:
                        ls_refFilter.append([ref.Cart_X, ref.Cart_Y])

                    if self.num_refStart == -1:
                        self.num_refStart = len(ls_refFilter)

                        for r in range(self.num_refStart):
                            self.data_store.append([0., 0.])

                    # -- lấy toạ độ trung bình cho các điểm gương
                    if self.num_refStart != len(ls_refFilter):
                        print("số lượng gương các lần đọc không phù hơp")
                        return

                    numave = 30
                    if self.num_getPoseReflector < numave:
                        print("Lấy mẫu gương lần thứ {x}".format(x = self.num_getPoseReflector))
                        for index, ref in enumerate(ls_refFilter):
                            self.data_store[index][0] += ref[0]
                            self.data_store[index][1] += ref[1]

                        self.num_getPoseReflector += 1
                        self.rate.sleep()
                        continue

                    # -- Tính trung bình 20 lần đọc tạo độ gương
                    ls_refFilter_average = []
                    for ave_ref in self.data_store:
                        ls_refFilter_average.append([ave_ref[0]/numave, ave_ref[1]/numave])

                    print("Giá trị trung bình của các gương mới là: ",ls_refFilter_average)

                    numref_before = len(self.lsReflector_inMap)
                    # Tính lại các vị trí gương
                    # convert_posReflector = []
                    # for ref in ls_refFilter_average:
                    #     x_cv, y_cv = self.calculate_reflector_position_in_map(self.x_init, self.y_init, self.r_init, ref[0], ref[1])
                    #     convert_posReflector.append([x_cv, y_cv])

                    # Thêm gương vào map
                    # print("im here")
                    self.add_reflectorMap(ls_refFilter_average)
                    numref_after = len(self.lsReflector_inMap)
                    
                    print("add reflector done, number reflector new is ", numref_after - numref_before)
                    return
                
                # -- Trường hợp đã có gương trong map
                else:
                    if self.is_r2000 == False or self.is_rawReflector == False :
                        self.rate.sleep()
                        continue
                    
                    self.is_r2000 = False
                    self.is_rawReflector = False

                    self.num_reflector = self.data_raw_reflector.num_reflector

                    # -- Lấy toạ độ gương lọc
                    ls_refFilter = []
                    for ref in self.data_raw_reflector.reflectors:
                        ls_refFilter.append([ref.Cart_X, ref.Cart_Y])

                    if self.num_refStart == -1:
                        self.num_refStart = len(ls_refFilter)

                        for r in range(self.num_refStart):
                            self.data_store.append([0., 0.])

                    # -- lấy toạ độ trung bình cho các điểm gương
                    if self.num_refStart != len(ls_refFilter):
                        print("số lượng gương các lần đọc không phù hơp")
                        continue

                    numave = 20
                    if self.num_getPoseReflector < numave:
                        self.data_storePose_x.append(self.data_r2000.x)
                        self.data_storePose_y.append(self.data_r2000.y)
                        self.data_storePose_phi.append(self.data_r2000.phi)

                        print("Lấy giá trị mẫu lần thứ {x}".format(x = self.num_getPoseReflector))
                        for index, ref in enumerate(ls_refFilter):
                            self.data_store[index][0] += ref[0]
                            self.data_store[index][1] += ref[1]

                        self.num_getPoseReflector += 1
                        self.rate.sleep()
                        continue

                    # -- Tính trung bình 20 lần đọc tạo độ gương
                    ls_refFilter_average = []
                    for ave_ref in self.data_store:
                        ls_refFilter_average.append([ave_ref[0]/numave, ave_ref[1]/numave])

                    print(f"Giá trị trung bình của các {len(ls_refFilter_average)} gương mới là: {ls_refFilter_average}")
                    
                    print(f"Giá trị trung bình của các {len(self.data_storePose_x)} phep bien doi là: {self.data_storePose_x}")
                    
                    pose_x = np.array(self.data_storePose_x)
                    pose_y = np.array(self.data_storePose_y)
                    pose_phi = np.array(self.data_storePose_phi)

                    self.x_init = np.mean(pose_x)
                    self.y_init = np.mean(pose_y)
                    self.r_init = np.mean(pose_phi)

                    print(f"Vị trí khởi tạo trung bình là: x = {self.x_init}, y = {self.y_init}, z = {self.r_init}")

                    # done thi reset variable
                    self.data_storePose_x = []
                    self.data_storePose_y = []
                    self.data_storePose_phi = []

                    # Tính lại các vị trí gương
                    convert_posReflector = []
                    for ref in ls_refFilter_average:
                        x_cv, y_cv = self.calculate_reflector_position_in_map(self.x_init, self.y_init, self.r_init, ref[0], ref[1])
                        convert_posReflector.append([x_cv, y_cv])
                    
                    # -- 
                    print(f"Number new reflector: {self.num_reflector} và {self.data_r2000.number_reflectors}")
                    if self.num_reflector > self.data_r2000.number_reflectors:
                        # -- tính lại toạ độ
                        ls_refNotMatchmap = []
                        ls_refMatchmap = []

                        for id_ref in self.data_r2000.localID:
                            for index, ref in enumerate(convert_posReflector):
                                if id_ref == (index + 1): 
                                    ls_refMatchmap.append(index)

                        # - clear reflector point has index in above list
                        indices_to_remove_set = set(ls_refMatchmap)

                        # Duyệt qua danh sách và giữ lại các phần tử không nằm trong indices_to_remove
                        filtered_list = [element for index, element in enumerate(convert_posReflector) if index not in indices_to_remove_set]
                        ls_refNotMatchmap = filtered_list

                        print(f"Có {len(ls_refNotMatchmap)} toạ độ cần thêm là: {ls_refNotMatchmap}")
                        
                        for ref in ls_refNotMatchmap:
                            next = None
                            while next != 'no' and next != 'yes':
                                try:
                                    next = input("Add new reflector (yes/no): ")
                                except:
                                    return

                            if next == 'no':
                                continue
                            
                            numref_before = len(self.lsReflector_inMap)

                            # print()
                            # Thêm gương vào map
                            self.add_reflector(ref)
                            numref_after = len(self.lsReflector_inMap)
                            print("add reflector done, number reflector new is ", numref_after - numref_before)
                        return

                    else:
                        print("Have no new reflector")
                        return

                print("---end loop---")

            self.rate.sleep()

def main():
    print ("--- Run Detect Reflector ---")
    program = DetectReflector()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()

