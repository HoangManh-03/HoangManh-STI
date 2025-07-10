#!/usr/bin/env python3
"""
*** infomation

*** Task Description: 
    + Đọc dữ liệu từ lmk file 
    + ghi dữ liệu vào file json

*** Need to do
    
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
from navigation_reflector.msg import *

from itertools import combinations

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import os
from datetime import datetime

class reflectorMap():
    def __init__(self, _id = 0, _x = 0., _y = 0.):
        self.id = _id
        self.x = _x
        self.y = _y

class MAP():
    def __init__(self):
        rospy.init_node('map_node', anonymous = True)
        self.rate = rospy.Rate(50)

        # -- ros pub && sub

        self.map_pub = rospy.Publisher('/map', R2000_reflectors, queue_size=10)
        self.map_data = R2000_reflectors()

        self.marker_pub = rospy.Publisher('/visualization_marker_map', Marker, queue_size=10)

        # -- global variables
        self.process = 0

        # self.dir_mapReflector = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/map_reflector_archie.json'  # map directory
        self.dir_mapReflector = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/map_reflector_nav350_v2.json'  # map directory
        
        self.dir_mapReflector_lmk = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/Layout_STI_6_9_new.lmk'
        self.lsReflector_inMap = []  # list reflectỏr and its ID
        self.lsReflector_inMap_arrange = []

        self.ls_ref_lmk = []

    def read_landmark_data(self, filename):
        data_list = []
        with open(filename, "r") as file:
            lines = file.readlines()

        # Tìm dòng bắt đầu chứa dữ liệu (bỏ qua phần metadata)
        data_start = 0
        for i, line in enumerate(lines):
            if line.strip().startswith("globID"):  # Tìm dòng tiêu đề cột
                data_start = i + 1
                break

        # Đọc dữ liệu từ dòng có tiêu đề trở đi
        for line in lines[data_start:]:
            parts = line.split()  # Tách dữ liệu theo khoảng trắng
            if len(parts) >= 3:
                data_list.append([int(parts[1])/1000, int(parts[2])/1000])  # Lưu 3 cột đầu

        return data_list

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

    def run(self):
        while not rospy.is_shutdown():
            # -- Tính toán các thông tin của map gương tham chiếu
            if self.process == 0:

                self.ls_ref_lmk = self.read_landmark_data(self.dir_mapReflector_lmk)
                print(self.ls_ref_lmk)

                numref_before = len(self.lsReflector_inMap)

                self.add_reflectorMap(self.ls_ref_lmk)
                numref_after = len(self.lsReflector_inMap)
                
                print("add reflector done, number reflector new is ", numref_after - numref_before)
                return

            self.rate.sleep()

def main():
    print ("--- Run map node ---")
    program = MAP()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




