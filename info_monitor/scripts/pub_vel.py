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
from collections import deque


class SocketPublisher:
    def __init__(self, host, port, mac_address, window_size=10):
        self.sio = socketio.Client()
        self.host = host
        self.port = port
        self.mac_address = mac_address
        self.last_velocity = None
        self.ip_address = None

        # Khởi tạo kết nối socket
        self.sio.on('connect', self.connect)
        self.sio.on('disconnect', self.disconnect)
        self.velocity = 0
        self.x = 0
        self.y = 0
        self.z = 0

        self.x_window = deque(maxlen=7)  # Cửa sổ giá trị x
        self.y_window = deque(maxlen=7)  # Cửa sổ giá trị y


    # Hàm khi client kết nối thành công với server
    def connect(self):
        print("Connected to the server.")

    # Hàm khi client mất kết nối với server
    def disconnect(self):
        print("Disconnected from the server.")

    # Hàm để lấy địa chỉ IP của máy (Chạy trong thread riêng)
    def get_ip_address(self):
        try:
            # Gọi lệnh nmcli để lấy địa chỉ IP
            result = subprocess.run(
                ["nmcli", "-g", "ip4.address", "device", "show", "wlo2"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=True
            )
            ip_address = result.stdout.decode('utf-8').strip().split('/')[0]
            self.ip_address = ip_address if ip_address else None
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error occurred: {e}")

    # Hàm gửi dữ liệu đến server
    def send_data(self):
        data = {}
        if self.ip_address:
            data = {
                "ip": self.ip_address,
                "vel": self.velocity,
                "x": self.x,
                "y": self.y,
                "a": self.z
            }
            message = f"{json.dumps(data)}"

        print(self.velocity,self.x, self.y, self.z)

        if self.sio.connected:
            self.sio.emit('Update-Velocity', data)
        else:
            print("Socket.IO is not connected. Retrying...")
            #print(f"Sent data: {message}")

    # Callback khi nhận dữ liệu từ topic cmd_vel
    def cmd_vel_callback(self, msg):
        self.velocity = msg.twist.twist.linear.x

        # self.send_data(linear_velocity)
    def pose_callback(self,msg):
        # self.x = msg.pose.position.x*1000
        # self.y = msg.pose.position.y*1000
        raw_x = msg.pose.position.x*1000
        raw_y = msg.pose.position.y*1000

        self.x_window.append(raw_x)
        self.y_window.append(raw_y)

        self.x = sum(self.x_window) / len(self.x_window)
        self.y = sum(self.y_window) / len(self.y_window)

        x_ = msg.pose.orientation.x
        y_ = msg.pose.orientation.y
        z_ = msg.pose.orientation.z
        w_ = msg.pose.orientation.w

        self.z = math.atan2(2.0 * (w_ * z_ + x_ * y_), 1.0 - 2.0 * (y_ * y_ + z_ * z_))
        self.z = math.degrees(self.z)
        self.z = self.z % 360





    # Hàm để khởi động ROS node và kết nối socket
    def start(self):
        rospy.init_node('socket_publisher', anonymous=True)

        # Thực hiện lấy địa chỉ IP trong một thread riêng
        threading.Thread(target=self.get_ip_address, daemon=True).start()

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
                print("ROS shutdown detected. Exiting...")
                return  # Thoát nếu ROS bị tắt

        rospy.Subscriber('/raw_vel', TwistWithCovarianceStamped, self.cmd_vel_callback)
        rospy.Subscriber('/robotPose_nav', PoseStamped, self.pose_callback)

        rate = rospy.Rate(5)
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

