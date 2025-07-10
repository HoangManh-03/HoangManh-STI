#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import websocket
import threading
import time
from urllib.parse import urlparse

# get ip
import os                                                                                                                                                           
import re 
import subprocess

import sys
import struct
from decimal import *
from math import degrees, radians
import rospy
from datetime import datetime

import json

from sti_msgs.msg import NN_cmdRequest     # request to client udp
from sti_msgs.msg import NN_infoRequest     # request to client udp
from sti_msgs.msg import NN_infoRespond     #   respond from client udp

# import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point

class FrameSendServer:
    def __init__(self, type_agv = 0):
        self._data = {
            "type": type_agv,
            "info": {
                "ip": "",
                "mac": "",
                "x": 0,
                "y": 0,
                "r": 0,
                "rfid_lastcode": 0,
                "rfid_code": 0,
                "direction": 0,
                "battery": 0,
                "status": 0,
                "offset": 0,
                "mode": 0,
                "task_status": 0,
                "error_code": 0
            }
        }

    # Magic method để truy cập giá trị bằng obj["key"]
    def __getitem__(self, key):
        return self._data[key]

    # Magic method để gán giá trị bằng obj["key"] = value
    def __setitem__(self, key, value):
        self._data[key] = value

    # Tùy chọn: Hàm in đẹp
    def __repr__(self):
        return str(self._data)
    
    def to_dict(self):
        return self._data

class WebSocketClient:
    def __init__(self, url):
        self.is_connected = False
        self.data_recieved = ''
        self.closeByProgram = False

        self.url = url
        self.ws = websocket.WebSocketApp(
            self.url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )

        # print("create websocket client!")

    def on_open(self, ws):
        """Xử lý sự kiện khi kết nối WebSocket được mở."""
        parsed_url = urlparse(self.url)
        ip = parsed_url.hostname
        port = parsed_url.port if parsed_url.port else (80 if parsed_url.scheme == 'http' else 443)
        print(f"Connected to WebSocket server at {ip}:{port}")
        self.is_connected = True

    def on_message(self, ws, message):
        """Xử lý sự kiện khi nhận được tin nhắn."""
        # print("Data recievce: ", message)
        self.data_recieved = message

    def on_error(self, ws, error):
        """Xử lý sự kiện khi có lỗi xảy ra."""
        print("Have an Error Websocket:", error)

    def on_close(self, ws, close_status_code, close_msg):
        """Xử lý sự kiện khi kết nối WebSocket bị đóng."""
        self.is_connected = False
        print("WebSocket connection closed")

    def run(self):
        while True and not self.closeByProgram:
            self.ws.run_forever()
            time.sleep(1.5)

    def send_message(self, message):
        """Gửi tin nhắn tới máy chủ nếu kết nối đang hoạt động."""
        if self.is_connected:
            self.ws.send(message)

    def close(self):
        """Đóng kết nối WebSocket."""
        self.closeByProgram = True
        if self.ws:
            print("WebSocket connection closed by program")
            self.ws.close()


class ROSCommunication():
    def __init__(self):
        rospy.init_node('stiClient_ws', anonymous=False)
        self.rate = rospy.Rate(30)

        self.type_agv = 2

        # Param Server
        self.ip_server = rospy.get_param("~ip_server", '192.168.1.21')
        self.port_server = rospy.get_param("~port_server", '8080')

        self.ip_server = '192.168.1.2'
        self.port_server = 8080

        # Tham số ROS
        self.name_card = rospy.get_param("~name_card", "wlp0s20f3")
        self.name_card = 'wlo2'

        self.topic_NNcmdRequest = rospy.get_param("~topic_NNcmdRequest", "NN_cmdRequest")
        self.topic_NNinfoRespond = rospy.get_param("~topic_NNinfoRespond", "NN_infoRespond")
        self.topic_NNinfoRequest = rospy.get_param("~topic_NNinfoRequest", "NN_infoRequest")

        rospy.Subscriber(self.topic_NNinfoRespond, NN_infoRespond, self.NN_infoCallback)
        self.NN_infoRespond = NN_infoRespond()
        self.NN_is_infoReceived = 0

        self.NN_cmdPub = rospy.Publisher(self.topic_NNcmdRequest, NN_cmdRequest, queue_size=50)
        self.NN_cmdRequest = NN_cmdRequest()
        self.NN_infoRequestPub = rospy.Publisher(self.topic_NNinfoRequest, NN_infoRequest, queue_size=50)
        self.NN_infoRequest = NN_infoRequest()

        self.NN_cmdRequest.list_id = [0, 0, 0 ,0 , 0]
        self.NN_cmdRequest.list_x = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.NN_cmdRequest.list_y = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.NN_cmdRequest.list_speed = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.telegram_count = 0
        # -- 
        self.ip_robot, self.mac_robot = self.get_ip_and_mac(self.name_card)
        while self.ip_robot == "-1":
            print("Connection not available, reconnect after 1 second")
            time.sleep(1.)
            self.ip_robot, self.mac_robot = self.get_ip_and_mac(self.name_card)

        print("IP: %s, MAC: %s" %(self.ip_robot, self.mac_robot))

    def NN_infoCallback(self, dat):
        self.NN_infoRespond = dat
        self.NN_is_infoReceived = 1

    def convert_angleSendServer(self, angle_rad):
        angle_deg = degrees(angle_rad)
        if angle_deg < 0:
            converted_angle = 180 + (-angle_deg)
        else:
            converted_angle = angle_deg
        return converted_angle
    
    def get_ip_and_mac(self, interface):
        try:
            output = subprocess.check_output(["ip", "addr", "show", interface], text=True)
            ip_match = re.search(r'inet\s+(\d+\.\d+\.\d+\.\d+)', output)
            ip_address = ip_match.group(1) if ip_match else None

            mac_match = re.search(r'link/ether\s+([\da-fA-F:]+)', output)
            mac_address = mac_match.group(1) if mac_match else None

            return ip_address if ip_address else "-1", mac_address if mac_address else "-1"
        
        except Exception as e:
            print(f"Error: {e}")
            return "-1", "-1"

    def convert_positionRecieveServer(self, data):
        return data/1000.
    
    def convert_offsetRecieveServer(self, data):
        return data/1000.
    
    def convert_angleRecieveServer(self, data):
        # angle = data/100. 
        angle = data/100. + 180.
        if angle > 180:
            angle = 360 - angle
            return round(radians(angle)*(-1), 3)
        
        return round(radians(angle), 3)

    def convert_error(self, list_error):
        x = list_error[0] if len(list_error) > 0 else 0
        switcher={
            # NN
            0:0,   # ALL RIGHT
            111:1,  # Va vào Blsock.
            121:2, # Ấn EMG.
            131:3, # Ra khỏi đường từ.
            141:4, # Bàn nâng.
            211:5, # Camera:Mất kết nối vật lý.
            212:6, # Camera: Không giao tiếp truyền thông.
            221:7, # Lidar. Phía trước.
            222:8, # Lidar. Phía sau.
            223:9, # Lidar. Cả 2.
            231:10, # IMU Không giao tiếp truyền thông.
            241:11, # PS2 Không giao tiếp truyền thông.
            251:12, # DRIVER 1 (trái) NN – Natual Navigatiroson.
            261:13, # DRIVER 2 (phải) NN – Natual Navigation.
            271:14, # Mangnetic line: NN - RS232 – PC lỗi (phía trước).
            272:15, # Mangnetic line: NN - RS232 – PC lỗi (phía sau).
            311:16, # Mạch MC - NN.	Mất kết nối vật lý Serial.
            312:17, # Mạch MC - NN. Không giao tiếp truyền thông Serial.
            321:18, # Mạch Main - NN:	Mất kết nối vật lý Serial.
            322:19, # Mạch Main - NN: Không giao tiếp truyền thông Serial.
            331:20, # Mạch SC: Mất kết nối vật lý Serial.
            332:21, # Mạch SC: Không giao tiếp truyền thông Serial.		
            341:22,	# Mạch OC: Mất kết nối vật lý Serial.
            342:23,	# Mạch OC: Không giao tiếp truyền thông Serial.
            351:24,	# Mạch HC: Mất kết nối vật lý Serial.
            352:25,	# Mạch HC: Không giao tiếp truyền thông Serial.
            411:201, # Di chuyển NN (không vạch từ)- Không thể đến được đích.
            421:202, # Có vật cản.
            431:203, # Mất kết nối server.
            441:204, # Không thể thấy Tag.
            451:205, # Điện Áp Thấp.
            461:206, # Có vật cản khi vào kệ.
            471:207  # không có kệ hoặc lệch kệ khi nâng.
        }
        return switcher.get(x, 0)

    def NN_cmdAnalysis(self, data_receive):
        """
        Xử lý JSON server gửi xuống (data_receive) => publish lên ROS topic
        """
        try:
            self.NN_infoRequest.id_agv       = data_receive["id"]
            self.NN_infoRequest.name_agv     = data_receive["name"]
            self.NN_infoRequestPub.publish(self.NN_infoRequest)

            self.NN_cmdRequest.id_command    = data_receive["tran_id"]
            self.NN_cmdRequest.process       = data_receive["process"]
            self.NN_cmdRequest.tag           = data_receive["offset"]
            self.NN_cmdRequest.target_id     = data_receive["target"]
            self.NN_cmdRequest.target_x      = self.convert_positionRecieveServer(data_receive["target_x"])
            self.NN_cmdRequest.target_y      = self.convert_positionRecieveServer(data_receive["target_y"])
            self.NN_cmdRequest.target_z      = self.convert_angleRecieveServer(data_receive["target_angle"])
            self.NN_cmdRequest.offset        = self.convert_offsetRecieveServer(data_receive["target_offset"])

            num_point = len(data_receive["routes"])
            for l in range(5):
                if l < num_point:
                    point = data_receive["routes"][l]
                    self.NN_cmdRequest.list_id[l]    = point["name"]
                    self.NN_cmdRequest.list_x[l]     = self.convert_positionRecieveServer(point["x"])
                    self.NN_cmdRequest.list_y[l]     = self.convert_positionRecieveServer(point["y"])
                    self.NN_cmdRequest.list_speed[l] = point["speed"]
                else:
                    self.NN_cmdRequest.list_id[l]    = 0
                    self.NN_cmdRequest.list_x[l]     = 0.0
                    self.NN_cmdRequest.list_y[l]     = 0.0
                    self.NN_cmdRequest.list_speed[l] = 0

            self.NN_cmdRequest.before_mission = data_receive["precode"]
            self.NN_cmdRequest.after_mission  = data_receive["subcode"]
            self.NN_cmdRequest.command        = data_receive["mes"]

            self.NN_cmdPub.publish(self.NN_cmdRequest)

        except Exception as e:
            print("Lỗi khi bóc tách dữ liệu JSON:", e)
            print(data_receive)
    
    def convertDataRosToServer(self):
        """Lấy dữ liệu từ self.NN_infoRespond => đóng gói JSON gửi lên server."""
        if self.NN_is_infoReceived == 0:
            print("waiting data from sti_control")
            return ''

        x = int(self.NN_infoRespond.x*1000)
        y = int(self.NN_infoRespond.y*1000)
        z_rad = self.NN_infoRespond.z
        cv_angle = self.convert_angleSendServer(z_rad)

        frame_send = FrameSendServer(self.type_agv)
        frame_send["info"]["ip"]            = self.ip_robot
        frame_send["info"]["mac"]           = self.mac_robot
        frame_send["info"]["x"]             = x
        frame_send["info"]["y"]             = y
        frame_send["info"]["r"]             = cv_angle
        frame_send["info"]["battery"]       = self.NN_infoRespond.battery
        frame_send["info"]["status"]        = self.NN_infoRespond.status
        frame_send["info"]["offset"]        = int(self.NN_infoRespond.tag)
        frame_send["info"]["mode"]          = self.NN_infoRespond.mode
        frame_send["info"]["task_status"]   = self.NN_infoRespond.task_status
        frame_send["info"]["error_code"]    = self.convert_error(self.NN_infoRespond.listError)
        
        try:
            return json.dumps(frame_send.to_dict())
        except Exception as e:
            print("Lỗi khi convert dict->json:", e)
            return ''
        

def main():
    # - ROS
    ros_comm = ROSCommunication()

    ip_server = ros_comm.ip_server
    port_server = ros_comm.port_server

    websocket_uri = "ws://" + ip_server + ":" + str(port_server) + "/agv"
    print("websocket uri:", websocket_uri)

    client = WebSocketClient(websocket_uri)

    # Tạo một luồng để chạy WebSocket
    ws_thread = threading.Thread(target=client.run)
    ws_thread.start()
    time.sleep(1.)

    # -
    process = 1
    data_recieve = ''
    savetime_sendServer = time.time()

    # Gửi tin nhắn liên tục
    try:
        while not rospy.is_shutdown():
            dentalTime = time.time() - savetime_sendServer
            if dentalTime > 0.5:
                savetime_sendServer = time.time()
                str_send = ros_comm.convertDataRosToServer()
                if str_send:
                    client.send_message(str_send)

            if process == 1:
                if client.is_connected:
                    data_recv = client.data_recieved
                    if len(data_recv) > 0:
                        try:
                            data_recieve = json.loads(data_recv)
                            process = 2
                        except Exception as e:
                            print("loi khi chuyen string to json, error: ", e)

                        client.data_recieved = ''      

            elif process == 2:
                ros_comm.NN_cmdAnalysis(data_recieve)
                process = 1     

            ros_comm.rate.sleep()

        print("Close Program by ROS")
        client.close()
        ws_thread.join()  # Đợi cho luồng WebSocket kết thúc

    except KeyboardInterrupt:
        print("Close program...")
        client.close()
        ws_thread.join()  # Đợi cho luồng WebSocket kết thúc

if __name__ == "__main__":
    main()
