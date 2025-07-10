#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import csv
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt5.QtCore import Qt, QRectF, QTimer

from PyQt5 import QtGui

from PyQt5.QtGui import QBrush, QColor
import pandas as pd
import rospy
from geometry_msgs.msg import PoseStamped
from message_pkg.msg import *
from sti_msgs.msg import *

# def read_coordinates(file_path):
#     df = pd.read_csv(file_path)
#     df.columns = df.columns.str.strip()  # Loại bỏ khoảng trắng thừa
#     return df[['x', 'y']].values


# class Point_Robot_pose:
#     def __init__(self):
#         rospy.init_node('sub_reflector', anonymous=True)

#         rospy.Subscriber('robotPose_nav', PoseStamped, self.callback_pose)
#         self.poseRb = PoseStamped

#         rospy.Subscriber('/nav350_reflectors', Reflector_array, self.callback_navReflector)
#         self.reflector = Reflector_array

#     def callback_pose(self, data):
#         self.poseRb = data


#     def callback_navReflector(self, data):
#         self.ref = data

#     def euler_to_quaternion(self, euler):
#         quat = Quaternion()
#         odom_quat = quaternion_from_euler(0, 0, euler)
#         quat.x = odom_quat[0]
#         quat.y = odom_quat[1]
#         quat.z = odom_quat[2]
#         quat.w = odom_quat[3]
#         return quat

#     def quaternion_to_euler(self, qua):
#         quat = (qua.x, qua.y, qua.z, qua.w )
#         a, b, euler = euler_from_quaternion(quat)
#         return euler

#     def run(self):
#         while not rospy.is_shutdown():
#             print(self.poseRb)


# class GraphicsView(QGraphicsView):
#     def __init__(self, coordinates):
#         super().__init__()

#         self.scene = QGraphicsScene(self)
#         self.setScene(self.scene)

#         self.point_current = Point_Robot_pose()

#         self.setRenderHint(QtGui.QPainter.Antialiasing)
#         self.setFixedSize(800, 600)
#         self.setFocus()
#         self.draw_grid()
#         self.draw_points(coordinates)
#         # Đặt kích thước giới hạn cho scene
#         # self.scene.setSceneRect(0, 0, 800, 600)  # (x, y, width, height)
#         self.setDragMode(QGraphicsView.ScrollHandDrag)  # Đặt chế độ kéo



# ##############################################################################################


#     def draw_grid(self):
#         # Thiết lập màu và độ dày của đường lưới
#         pen = QtGui.QPen(Qt.lightGray, 100)

#         # Vẽ đường kẻ theo chiều ngang
#         for y in range(-300000, 300000, 5000):  # Vẽ từ -300 đến 300 với khoảng cách 50
#             self.scene.addLine(-10000000, y, 10000000, y, pen)  # Vẽ đường ngang

#         # Vẽ đường kẻ theo chiều dọc
#         for x in range(-300000, 300000, 5000):  # Vẽ từ -400 đến 400 với khoảng cách 50
#             self.scene.addLine(x, -10000000, x, 10000000, pen)  # Vẽ đường dọc

#     def draw_points(self, coordinates):
#         for x, y in coordinates:
#             # Tạo hình tròn nhỏ để đại diện cho điểm
#             ellipse = QGraphicsEllipseItem(x, y, 500, 500)  # Kích thước 5x5 cho mỗi điểm
#             ellipse.setBrush(Qt.blue)  # Đặt màu cho hình tròn
#             self.scene.addItem(ellipse)  # Thêm hình tròn vào scene

#             print("x, y la: ",x, y)

#     def draw_robot_pose(self, x, y):

#         ellipse = QGraphicsEllipseItem(x, y, 2000, 2000)
#         ellipse.setBrush(Qt.red)  # Đặt màu cho hình tròn
#         self.scene.addItem(ellipse)  # Thêm hình tròn vào scene

#     # Xử lý sự kiện phím để phóng to và thu nhỏ
#     def keyPressEvent(self, event):
#         if event.key() == Qt.Key_Plus:  # Phím +
#             self.scale(1.2, 1.2)  # Phóng to 1.2 lần
#         elif event.key() == Qt.Key_Minus:  # Phím -
#             self.scale(0.8, 0.8)  # Thu nhỏ 0.8 lần
#         else:
#             super().keyPressEvent(event)  # Xử lý phím khác


#     def wheelEvent(self, event):
#         if event.angleDelta().y() > 0:  # Lăn chuột lên
#             self.scale(1.2, 1.2)  # Phóng to
#         else:  # Lăn chuột xuống
#             self.scale(0.8, 0.8)  # Thu nhỏ

#     def run(self):
#         # while True:
#         self.point_current.run()
#         self.draw_robot_pose(self.point_current.poseRb.x, self.point_current.poseRb.y)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     # file_path = 'outputmap12.csv'
#     file_path = '/home/stivietnam/catkin_ws/src/app_ros/script_new/outputmap12.csv'
#     coordinates = read_coordinates(file_path)
#     view = GraphicsView(coordinates)
#     view.run()
#     view.show()

#     sys.exit(app.exec_())


def read_coordinates(file_path):
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip()  # Loại bỏ khoảng trắng thừa
    return df[['x', 'y']].values

class Point_Robot_pose:
    def __init__(self):
        rospy.init_node('sub_reflector', anonymous=True)

        rospy.Subscriber('robotPose_nav', PoseStamped, self.callback_pose)
        self.poseRb = PoseStamped()

        rospy.Subscriber('/nav350_reflectors', Reflector_array, self.callback_navReflector)
        self.reflector = Reflector_array()

    def callback_pose(self, data):
        self.poseRb = data

    def callback_navReflector(self, data):
        self.ref = data

    def run(self):
        # Không cần vòng lặp while ở đây
        pass

class GraphicsView(QGraphicsView):
    def __init__(self, coordinates):
        super().__init__()

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)

        self.point_current = Point_Robot_pose()

        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setFixedSize(800, 600)
        self.setFocus()
        self.draw_grid()
        self.draw_points(coordinates)

        self.setDragMode(QGraphicsView.ScrollHandDrag)  # Đặt chế độ kéo

        # Tạo QTimer để cập nhật vị trí robot định kỳ
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_robot_pose)
        self.timer.start(100)  # Cập nhật mỗi 100ms

    def draw_grid(self):
        # Thiết lập màu và độ dày của đường lưới
        pen = QtGui.QPen(Qt.lightGray, 100)

        # Vẽ đường kẻ theo chiều ngang
        for y in range(-300000, 300000, 5000):  # Vẽ từ -300 đến 300 với khoảng cách 50
            self.scene.addLine(-10000000, y, 10000000, y, pen)  # Vẽ đường ngang

        # Vẽ đường kẻ theo chiều dọc
        for x in range(-300000, 300000, 5000):  # Vẽ từ -400 đến 400 với khoảng cách 50
            self.scene.addLine(x, -10000000, x, 10000000, pen)  # Vẽ đường dọc

    def draw_points(self, coordinates):
        for x, y in coordinates:
            ellipse = QGraphicsEllipseItem(x, y, 500, 500)
            ellipse.setBrush(Qt.blue)
            self.scene.addItem(ellipse)
            print("x, y la: ", x, y)

    def draw_robot_pose(self, x, y):
        ellipse = QGraphicsEllipseItem(x, y, 2000, 2000)
        ellipse.setBrush(Qt.red)
        self.scene.addItem(ellipse)

    def update_robot_pose(self):
        # Cập nhật vị trí của robot từ ROS
        print("updataaaaaaaaaaaaaaaaaaaaaaaaeeeeeeeeeeeeeeeeeeeeeeee")
        x = self.point_current.poseRb.pose.position.x
        y = self.point_current.poseRb.pose.position.y
        print(x, y)
        self.draw_robot_pose(x, y)

    # Xử lý sự kiện phím để phóng to và thu nhỏ
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Plus:
            self.scale(1.2, 1.2)
        elif event.key() == Qt.Key_Minus:
            self.scale(0.8, 0.8)
        else:
            super().keyPressEvent(event)

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.scale(1.2, 1.2)
        else:
            self.scale(0.8, 0.8)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    file_path = '/home/stivietnam/catkin_ws/src/app_ros/script_new/outputmap12.csv'
    coordinates = read_coordinates(file_path)
    view = GraphicsView(coordinates)
    view.show()

    sys.exit(app.exec_())
