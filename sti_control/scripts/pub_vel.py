#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socketio
import time
import json
import subprocess
import rospy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
import threading
import math
import numpy as np
from tf.transformations import quaternion_multiply, quaternion_conjugate
from sti_msgs.msg import *
from message_pkg.msg import *
import psutil

class SocketPublisher:
    def __init__(self, host, port, mac_address):
        self.sio = socketio.Client()
        self.host = host
        self.port = port
        self.mac_address = mac_address
        self.last_velocity = None
        self.ip_address = None
        self.infoRespond = NN_infoRespond()
        self.process = 0
        self.converted_process = ""
        self.robot = rospy.get_param("robot", 'robot1')
        # Khởi tạo kết nối socket
        self.sio.on('connect', self.connect)
        self.sio.on('disconnect', self.disconnect)
        self.velocity = 0
        self.x = 0 
        self.y = 0
        self.z = 0
        self.array_error = []
        self.converted_errors = []
        self.MAC = "10.20.30.40.50"
        self.battery = 90.0
        self.rotation_translation_matrix = np.array([
            [-1, 0, 8.714],
            [0, -1, 3.159],
            [0, 0, 1]
        ])

    # Hàm khi client kết nối thành công với server
    def connect(self):
        print("Connected to the server.")

    # Hàm khi client mất kết nối với server
    def disconnect(self):
        print("Disconnected from the server.")
 
    # Hàm gửi dữ liệu đến server
    def get_mac_addresses(self):
        mac_addresses = {}
        for interface, addrs in psutil.net_if_addrs().items():
            for addr in addrs:
                if addr.family == psutil.AF_LINK:
                    mac_addresses[interface] = addr.address
        return mac_addresses

    def send_data(self):
        self.MAC = "b4:d5:bd:e8:74:fc"
        if self.ip_address:
            data = {
                "ip": self.ip_address,
                "mac": self.MAC,
                "vel": self.velocity,
                "battery": self.battery,
                "x": self.x,
                "y": self.y,
                "a": self.z,
                "error": "AGV hoạt động bình thường",
                "process": "Đang di chuyển"
            }
            print(data)
            message = f"{json.dumps(data)}"

        print(self.velocity,self.x, self.y, self.z, self.converted_errors, self.converted_process)

        if self.sio.connected:
            self.sio.emit('Update-Velocity', data)
        else:
            print("Socket.IO is not connected. Retrying...")
            #print(f"Sent data: {message}")

    def callback_error(self, msg):
        self.infoRespond = msg
        self.array_error = self.infoRespond.listError
        self.process = self.infoRespond.process 
        self.converted_errors = [self.convert_errorAll(val) for val in self.array_error]
        self.converted_process = self.show_job(self.process)
        
    def convert_errorAll(self, val):
        switcher={
            0:'AGV Hoạt Động Bình Thường',
            2:'Đã Khởi Tạo Lại Quy Trình Hiện Tại',
            3:'Nút Xóa Lỗi Đã Được Nhấn',
            311:'Mất kết nối với Mạch STI-RTC',         
            361:'Mất Kết Nối Với Mạch STI-CPD', # 
            362:'Mất Kết Nối Với Mạch STI-CPD2', #
            351:'Mất Kết Nối Với Mạch STI-HC', # 
            352:'Không Giao Tiếp CAN Với Mạch STI-HC', # 

            341:'Mất Kết Nối Với Mạch STI-OC', #
            342:'Mất Cổng USB của USB của Mạch STI-OC', # 
            343:'Không Giao Tiếp CAN Với Mạch STI-OC', # 
            344:'Không Giao Tiếp Với Mạch STI-OC1', # 
            345:'Không Giao Tiếp Với Mạch STI-OC2', # 
            346:'Không Giao Tiếp Với Mạch STI-OC3', # 

            323:'Mạng CAN Không Gửi Được', # 
            321:'Mất Kết Nối Với Mạch STI-Main', # 
            322:'Mất Cổng USB của USB của Mạch STI-Main', # 
            250:'Mất Kết Nối Với Driver', # 
            251:'Mất Kết Nối Với Driver1', # 
            252:'Lỗi Động Cơ Số 1', # 
            261:'Mất Kết Nối Với Driver2', # 
            262:'Lỗi Động Cơ Số 2', # 
            231:'Mất Kết Nối Với Cảm Biến Góc', # 
            232:'Mất Cổng USB của Cảm Biến IMU', # 
            241:'Lỗi Định Vị: Matching', # 
            242:'Lỗi Định Vị: Localization', # 
            243:'Lỗi Định Vị: Matching',
            221:'Mất Kết Nối Với Cảm Biến NAV350', # 
            181:'LoadCell-Ket Noi', # 
            182:'LoadCell-Dau noi', # 
            183:'LoadCell-USB', # 
            184:'Quá Tải 700kg', # 
            222:'Mất Tọa Độ Định vị', # 
            223:'Parking: Không Phát Hiện mã Tag', #
            224:'Parking: Mất Dữ Liệu Odom Lidar', #
            225:'FANUC xảy ra lỗi', #
            141:'Lỗi Không Chạm Được Cảm Biến Bàn Nâng', # 
            121:'Trạng Thái Dừng Khẩn - EMG', # 
            122:'AGV Bị Chạm Blsock', #
            272:'Không Phát Hiện Được Đủ Gương', #
            281:'Mất TF Parking', #
            282:'Mất Gói Điều Hướng Di Chuyển', #

            440:'Parking: Không phát hiện được Tag', #
            441:'AGV Đã Di Chuyển Hết Điểm', #
            442:'AGV Đang Dừng Để Nhường Đường Cho AGV Khác', # 
            460:'FANUC: Đang Chưa Được Khởi Động', # 
            461:'FANUC: Đang Tạm Dừng Chương Trình', # 
            411:'Vướng Vật Cản - Di Chuyển Giữa Các Điểm', #
            412:'Vướng Vật Cản - Di Chuyển Vào Vị Trí Tag', # 
            414:'Parking: ID Tag Không Khớp', # 
            413:'Không Giao Tiếp Được Với Máy TAIFUN Qua Toyo', # 
            431:'AGV Không Giao Tiếp Với Phần Mềm Traffic', #
            451:'Điện Áp Của AGV Đang Rất Thấp', # 
            452:'AMR Không Sạc Được Pin', # 
            453:'Chưa Thực Hiện Thao Tác Định Vị', # 
            471:'Không Có Kệ Tại Vị Trí', # 
            454:'Đang Khởi Tạo Lại Vị Trí của AGV',
        }
        return switcher.get(val, 'UNK')

    def show_job(self, val):
        job_now = 'Không\nXác Định'     
        switcher={
            0:'...', #
            1:'Kiểm Tra Lại Nhiệm Vụ', # 
            2:'Thực Hiện Nhiệm Vụ Trước', # 
            3:'Kiểm Tra Trạng Thái Kệ',
            4:'Di Chuyển Ra Khỏi Vị Trí', # 
            5:'Di Chuyển Giữa Các Điểm', #
            6:'Di Chuyển Vào Vị Trí Thao Tác', # 
            7:'Thực Hiện Nhiệm Vụ Sau', # 
            8:'Đợi Lệnh Mới', # 
            9:'Đợi Hoàn Thành Lệnh Cũ', # 
            20:'Chế Độ Bằng Tay', # 
            30:'Chế Độ Tự Động', # 
            50:'Kiểm Tra Vị Trí Trả Hàng', # 
        }
        return switcher.get(val, job_now)

    # Callback khi nhận dữ liệu từ topic cmd_vel
    def cmd_vel_callback(self, msg):
        self.velocity = msg.twist.twist.linear.x

        # self.send_data(linear_velocity)
    def pose_callback(self,msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        x_ = msg.pose.orientation.x
        y_ = msg.pose.orientation.y
        z_ = msg.pose.orientation.z
        w_ = msg.pose.orientation.w

        self.x = x*1000
        self.y = y*1000
        theta_rad = 2 * math.atan2(z_, w_)
        theta_deg = math.degrees(theta_rad)
        self.z = theta_deg

    # Hàm để khởi động ROS node và kết nối socket
    def start(self):
        rospy.init_node('socket_publisher', anonymous=True) 
        self.ip_address = rospy.get_param("~ip_recieve", "192.168.1.162")
        topic_pose = rospy.get_param('~topic_pose', "robotPose_nav")
        topic_cmdvel = rospy.get_param('~topic_cmdvel', "cmd_vel")
        topic_NN_InfoRespond = rospy.get_param('~NN_infoRespond', "NN_infoRespond")

        # Thực hiện lấy địa chỉ IP trong một thread riêng
        # threading.Thread(target=self.get_ip_address, daemon=True).start()
        # self.ip_address = rospy.get_param("~ip_recieve", "127.0.0.1")

        # self.sio.connect(f'http://{self.host}:{self.port}')  # Kết nối đến server
        while not rospy.is_shutdown():
            try:
                print("Attempting to connect to the server...")
                self.sio.connect(f'http://{self.host}:{self.port}')  # Kết nối đến server
                print("Successfully connected to the server.")
                break  # Thoát khỏi vòng lặp khi kết nối thành công
            except socketio.exceptions.ConnectionError:
                print("Failed to connect to the server. Retrying in 5 seconds...")
                time.sleep(5)  # Đợi 5 giây trước khi thử lại
            except ROSInterruptException:
                return  # Thoát nếu ROS bị tắt

        rospy.Subscriber(topic_cmdvel, TwistWithCovarianceStamped, self.cmd_vel_callback)
        # rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        # rospy.Subscriber('/{self.robot}NN_infoRespond', NN_infoRespond, self.callback_error)
        rospy.Subscriber(topic_pose, PoseStamped, self.pose_callback)
        rospy.Subscriber(topic_NN_InfoRespond, NN_infoRespond, self.callback_error)

        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                self.send_data()  # Hàm publish dữ liệu
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            self.sio.disconnect()
            print("Socket disconnected")

if __name__ == "__main__":
    # Tạo đối tượng và gọi phương thức start để chạy
    publisher = SocketPublisher(host='192.168.1.2', port=3000, mac_address="50:76:af:ab:ee:6f")
    publisher.start()

