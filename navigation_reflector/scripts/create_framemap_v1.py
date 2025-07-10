#!/usr/bin/env python3
"""
*** infomation

*** Task Description: 
    + Tạo frame map, truy ra vị trí khơi taọ của cảm biến trên map tham chiếu.
    + Tạo gốc dữ liệu để dữ liệu hiển thị trên rviz không bị trôi.

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

        self.map_pub = rospy.Publisher('/map', Raw_reflector, queue_size=10)
        self.map_data = Raw_reflector()

        self.marker_pub = rospy.Publisher('/visualization_marker_map', Marker, queue_size=10)

        # -- global variables
        self.process = 0

        self.dir_mapReflector = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/map_reflector.json'  # map directory
        self.lsReflector_inMap = []  # list reflectỏr and its ID

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
            sphere_marker.header.frame_id = "map"
            sphere_marker.header.stamp = rospy.Time.now()
            sphere_marker.ns = "spheres"
            sphere_marker.id = i  # ID duy nhất
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = p.x
            sphere_marker.pose.position.y = p.y
            sphere_marker.pose.position.z = p.z
            sphere_marker.scale.x = 0.3
            sphere_marker.scale.y = 0.3
            sphere_marker.scale.z = 0.3
            sphere_marker.color.a = 1.0  # Độ trong suốt
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.8
            sphere_marker.color.b = 0.0
            # Pose
            sphere_marker.pose.orientation.x = 0.0
            sphere_marker.pose.orientation.y = 0.0
            sphere_marker.pose.orientation.z = 0.0
            sphere_marker.pose.orientation.w = 1.0

            marker_array.append(sphere_marker)

            # Marker hiển thị ID
            text_marker = Marker()
            text_marker.header.frame_id = "map"
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
            # -- Tính toán các thông tin của map gương tham chiếu
            if self.process == 0:
                #  -- Load map from json file
                try:
                    with open(self.dir_mapReflector, 'r') as file:
                        data = json.load(file)
                        for index, ref in enumerate(data["info"]):
                            self.lsReflector_inMap.append(reflectorMap(ref["id"], ref["x"], ref["y"]))

                except Exception as e:
                    print("Read json file fail: ", e)
                    return

                # print("Mảng gương có giá trị là: ", self.lsReflector_inMap[0].id)
                # print("Mảng gương có chiều dài là: ", len(self.lsReflector_inMap))
                # - move to next step
                self.process = 1

            # -- Kiểm tra xem đã nhận được list các điểm gương hiện taị chưa
            elif self.process == 1:
            
                # - display reference map to rviz use visualization marker
                ls_poseReflector = []
                msg_raw_reflector = Raw_reflector()
                # msg_raw_reflector.header.frame_id = 'map'
                # msg_raw_reflector.header.stamp = rospy.Time.now()
                # msg_raw_reflector.num_reflector = len(self.lsReflector_inMap)
                
                # n = 0
                for ref in self.lsReflector_inMap:
                    # Reflector_info = Reflector_data()
                    pose_reflector = Point()

                    ls_poseReflector.append([ref.x, ref.y, 0.0])

                    pose_reflector.x = ref.x
                    pose_reflector.y = ref.y
                    
                    # -- 
                    # d_f = sqrt(ref.x*ref.x + ref.y*ref.y)
                    # theta_f = atan2(ref.y, ref.x)

                    # for ros
                    # Reflector_info.Cart_X = ref.x
                    # Reflector_info.Cart_Y = ref.y
                    # Reflector_info.Polar_Dist = d_f
                    # Reflector_info.Polar_Phi = theta_f
                    # Reflector_info.GlobalID = ref.id
                    # Reflector_info.Size = 0
                    # Reflector_info.Index_Begin = 0
                    # Reflector_info.Index_End = 0

                    msg_raw_reflector.points.append(pose_reflector)
                    
                # print(msg_raw_reflector.points)
                self.pub_marker(ls_poseReflector)
                self.map_pub.publish(msg_raw_reflector)

            self.rate.sleep()

def main():
    print ("--- Run estimate_reflector_center ---")
    program = MAP()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




