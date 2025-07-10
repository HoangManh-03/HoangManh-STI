#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
# frame AGV send Traffic
    {
        "type":2,
        "info":{
            "ip":"",
            "mac":"",
            "x":0,
            "y":0,
            "r":0,
            "rfid_lastcode":0,
            "rfid_code":0,
            "direction":0,
            "battery":0,
            "status":0,
            "offset":0,
            "mode":0,
            "task_status":0,
            "error_code":0
        }
    }

# frame Traffic send AGV
    {
        "id":0,
        "name":"",
        "tran_id":0,
        "process":0,
        "target":0,
        "target_x":0,
        "target_y":0,
        "target_angle":0,
        "target_code":0,
        "target_dir":0,
        "target_offset":0,
        "moving_dir":0,
        "routes":[
            {
            "name":0,
            "x":0,
            "y":0,
            "code":0,
            "dir":0,
            "speed":0,
            "safety":0,
            "encoder":0
            }
        ],
        "precode":0,
        "subcode":0,
        "mes":""
    }
'''

import re 
import subprocess
import socket
import websocket
import threading
import time
from urllib.parse import urlparse
from datetime import datetime
import json

import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

from decimal import *
from math import degrees, radians

from message_pkg.msg import *
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

    def run(self, ros_kill):
        while not ros_kill and not self.closeByProgram:
            self.ws.run_forever()
            time.sleep(1.5)

    def send_message(self, message):
        """Gửi tin nhắn tới máy chủ nếu kết nối đang hoạt động."""
        if self.is_connected:
            try:
                self.ws.send(message)
            except Exception as e:
                print("Error send mess data socket: ", e)
                
    def close(self):
        """Đóng kết nối WebSocket."""
        self.closeByProgram = True
        if self.ws:
            print("WebSocket connection closed by program")
            self.ws.close()

class ROSCommunication(LifecycleNode):
    def __init__(self):
        super().__init__('stiClient_ws')
        self.get_logger().warn("ROS 2 Node stiClient_ws Initialized!")
        self.killnode = 0

        self.type_agv = 1

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip_server',                           "192.168.1.102"),
                ('port_server',                         8080),
                ('name_card',                           "wlo2"),
                ('topic_NNcmdRequest',                  "NN_cmdRequest"),
                ('topic_NNinfoRespond',                 "NN_infoRespond"),
                ('topic_NNinfoRequest',                 "NN_infoRequest")
            ]
        )
        
        # Param Server
        self.ip_server = self.get_parameter('ip_server').value
        self.port_server = self.get_parameter('port_server').value

        # self.ip_server = '192.168.1.2'
        # self.port_server = 8080

        # Tham số ROS
        self.name_card = self.get_parameter('name_card').value
        self.name_card = 'wlo2'

        self.topic_NNcmdRequest = self.get_parameter('topic_NNcmdRequest').value
        self.topic_NNinfoRespond = self.get_parameter('topic_NNinfoRespond').value
        self.topic_NNinfoRequest = self.get_parameter('topic_NNinfoRequest').value

        # -- Publisher 
        self.NN_cmdPub = self.create_publisher(NNcmdRequest, self.topic_NNcmdRequest, 10)
        self.NN_cmdRequest = NNcmdRequest()
        # -
        self.NN_infoRequestPub = self.create_publisher(NNinfoRequest, self.topic_NNinfoRequest, 10)
        self.NN_infoRequest = NNinfoRequest()

        # -- Subcriber
        self.sub_NNinfoRespond = self.create_subscription(
            NNinfoRespond,
            self.topic_NNinfoRespond,
            self.NN_infoCallback,
            10)
        self.sub_NNinfoRespond

        self.NN_infoRespond = NNinfoRespond()
        self.NN_is_infoReceived = 0

        # -- Param
        self.NN_cmdRequest.list_id = [0, 0, 0 ,0 , 0]
        self.NN_cmdRequest.list_code = [0, 0, 0 ,0 , 0]
        self.NN_cmdRequest.list_dir = [0, 0, 0 ,0 , 0]
        self.NN_cmdRequest.list_speed = [0, 0, 0 ,0 , 0]
        self.NN_cmdRequest.list_encoder = [0, 0, 0 ,0 , 0]

        self.process = 1
        self.data_recieve = ''
        self.savetime_sendServer = time.time()

        # -- 
        self.ip_robot, self.mac_robot = self.get_ip_and_mac(self.name_card)
        while self.ip_robot == "-1":
            print("Connection not available, reconnect after 1 second")
            time.sleep(1.)
            self.ip_robot, self.mac_robot = self.get_ip_and_mac(self.name_card)

        print("IP: %s, MAC: %s" %(self.ip_robot, self.mac_robot))

        # -- Create Websocket client
        websocket_uri = "ws://" + self.ip_server + ":" + str(self.port_server) + "/agv"
        print("websocket uri:", websocket_uri)
        self.ws_client = WebSocketClient(websocket_uri)

        # -- Create timer run 
        self.rate_main = 30
        self.timer_period_main = 1/self.rate_main
        self.timer_main = self.create_timer(self.timer_period_main, self.run)

        # self.timer_period_ws = 0.001
        # self.timer_ws = self.create_timer(self.timer_period_ws, self.run_ws)

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    # -- Callback Function
    def NN_infoCallback(self, dat):
        self.NN_infoRespond = dat
        self.NN_is_infoReceived = 1

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
            self.NN_infoRequest.id_agv      = data_receive["id"]
            self.NN_infoRequest.name_agv    = data_receive["name"]
            self.NN_infoRequestPub.publish(self.NN_infoRequest)

            self.NN_cmdRequest.id_command   = data_receive["tran_id"]
            self.NN_cmdRequest.process      = data_receive["process"]
            self.NN_cmdRequest.target_id    = data_receive["target"]
            self.NN_cmdRequest.target_dir   = data_receive["target_dir"]
            self.NN_cmdRequest.offset       = data_receive["target_offset"]
            self.NN_cmdRequest.moving_dir   = data_receive["moving_dir"]

            num_point = len(data_receive["routes"])
            for l in range(5):
                if l < num_point:
                    point = data_receive["routes"][l]
                    self.NN_cmdRequest.list_id[l]       = point["name"]
                    self.NN_cmdRequest.list_code[l]     = point["code"]
                    self.NN_cmdRequest.list_dir[l]      = point["dir"]
                    self.NN_cmdRequest.list_speed[l]    = point["speed"]
                    self.NN_cmdRequest.list_encoder[l]  = point["encoder"]
                else:
                    self.NN_cmdRequest.list_id[l]       = 0
                    self.NN_cmdRequest.list_code[l]     = 0
                    self.NN_cmdRequest.list_dir[l]      = 0
                    self.NN_cmdRequest.list_speed[l]    = 0
                    self.NN_cmdRequest.list_encoder[l]  = 0

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

        frame_send = FrameSendServer(self.type_agv)
        frame_send["info"]["ip"]            = self.ip_robot
        frame_send["info"]["mac"]           = self.mac_robot
        frame_send["info"]["rfid_lastcode"] = self.NN_infoRespond.rfid_last_code
        frame_send["info"]["rfid_code"]     = self.NN_infoRespond.rfid_code
        frame_send["info"]["direction"]     = self.NN_infoRespond.direction
        frame_send["info"]["battery"]       = self.NN_infoRespond.battery
        frame_send["info"]["status"]        = self.NN_infoRespond.status
        frame_send["info"]["offset"]        = int(self.NN_infoRespond.tag)
        frame_send["info"]["mode"]          = self.NN_infoRespond.mode
        frame_send["info"]["task_status"]   = self.NN_infoRespond.task_status
        frame_send["info"]["error_code"]    = self.convert_error(self.NN_infoRespond.list_error)

        try:
            return json.dumps(frame_send.to_dict())
        except Exception as e:
            print("Lỗi khi convert dict->json:", e)
            return ''
        
    def run_ws(self):
        self.ws_client.run(self.killnode)
        
    def run(self):
        dentalTime = time.time() - self.savetime_sendServer
        if dentalTime > 0.5:
            self.savetime_sendServer = time.time()
            str_send = self.convertDataRosToServer()
            # print(str_send)
            if str_send:
                self.ws_client.send_message(str_send)

        if self.process == 1:
            if self.ws_client.is_connected:
                data_recv = self.ws_client.data_recieved
                if len(data_recv) > 0:
                    try:
                        self.data_recieve = json.loads(data_recv)
                        self.process = 2
                    except Exception as e:
                        print("loi khi chuyen string to json, error: ", e)

                    self.ws_client.data_recieved = ''      

        elif self.process == 2:
            self.NN_cmdAnalysis(self.data_recieve)
            self.process = 1     


        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)

def main():
    # -- Khoi tao ROS
    rclpy.init()
    node = ROSCommunication()

    # Dùng MultiThreadedExecutor để chạy song song
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)

    # Tạo một luồng để chạy WebSocket
    ws_thread = threading.Thread(target=node.run_ws)
    ws_thread.start()
    time.sleep(1.)

    try:
        rclpy.spin(node)
        # executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.ws_client.close()
        ws_thread.join()  # Đợi cho luồng WebSocket kết thúc

        node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')

if __name__ == '__main__':
    main()
