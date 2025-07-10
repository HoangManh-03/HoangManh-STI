#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import json
import threading

import socketio
import asyncio

import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

from decimal import *
from math import degrees, radians

from message_pkg.msg import *
from geometry_msgs.msg import Pose, Point

class DataShare:
    def __init__(self):
        self.ip_socketio  = ''
        self.port_socketio = ''
        self.dataROS = None
        self.dataServer = None
        self.is_connected_server = False
        self.data_lock = threading.Lock()

dshare = DataShare()
sio_comm = socketio.AsyncClient(reconnection_delay_max=1.)
asyncio_loop = None

@sio_comm.on('connect', namespace='/agv')
async def connect():
    print('Connected to server')

@sio_comm.on('disconnect', namespace='/agv')
async def disconnect():
    print('Disconnected to server')

@sio_comm.on('respond_agv', namespace='/agv')
async def respond_agv(data):
    with dshare.data_lock:
        # print('Recieved data: ', data)
        dshare.dataServer = data

async def send_data_server():
    while True:
        data = None
        with dshare.data_lock:
            if dshare.dataROS is not None:
                data = dshare.dataROS
                dshare.dataROS = None

        if data is not None:
            try:
                await sio_comm.emit('status', data, namespace='/agv')
            except Exception as e:
                print('Send data to server Error: ', e)

        await asyncio.sleep(0.5)

async def main_socketio():
    while not sio_comm.connected:
        try:
            socketio_uri = f"http://{dshare.ip_socketio}:{str(dshare.port_socketio)}"
            print("socketio uri:", socketio_uri)
            await sio_comm.connect(socketio_uri, namespaces=['/agv'])
        except Exception as e:
            print('Connect to server failed')

        await asyncio.sleep(1.0)

    await asyncio.gather(
        send_data_server(),
        sio_comm.wait()
    )

# Run socketio on Thread
def start_socketio():
    global asyncio_loop
    asyncio_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(asyncio_loop)
    asyncio_loop.run_until_complete(main_socketio())


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


class ROSCommunication(LifecycleNode):
    def __init__(self):
        super().__init__('stiClient_socketio')
        self.get_logger().warn("ROS 2 Node stiClient_socketio Initialized!")
        self.killnode = 0

        self.type_agv = 1

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip_server',                           "127.0.0.1"),
                ('port_server',                         8001),
                ('name_card',                           "wlo2")
            ]
        )
        
        # Param Server
        self.ip_server = self.get_parameter('ip_server').value
        self.port_server = self.get_parameter('port_server').value

        # Tham số ROS
        self.name_card = self.get_parameter('name_card').value
        self.name_card = 'wlo2'

        with dshare.data_lock:
            dshare.ip_socketio  = self.ip_server 
            dshare.port_socketio = self.port_server
        
        # -- Publisher 
        self.NN_cmdPub = self.create_publisher(NNcmdRequest, 'NN_cmdRequest', 10)
        self.NN_cmdRequest = NNcmdRequest()
        # -
        self.NN_infoRequestPub = self.create_publisher(NNinfoRequest, 'NN_infoRequest', 10)
        self.NN_infoRequest = NNinfoRequest()

        # -- Subcriber
        self.sub_NNinfoRespond = self.create_subscription(
            NNinfoRespond,
            'topic_NNinfoRespond',
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
        self.ip_robot = '127.0.0.1'
        self.mac_robot = '11:22:33:44:55:66'

        # -- Create timer run 
        self.rate_main = 20
        self.timer_period_main = 1/self.rate_main
        self.timer_main = self.create_timer(self.timer_period_main, self.run)

        self.process = 1

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    # -- Callback Function
    def NN_infoCallback(self, dat):
        self.NN_infoRespond = dat
        self.NN_is_infoReceived = 1

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
            self.NN_infoRequest.id_agv          = data_receive["id"]
            self.NN_infoRequest.name_agv        = data_receive["name"]
            self.NN_infoRequestPub.publish(self.NN_infoRequest)

            self.NN_cmdRequest.id_command       = data_receive["tran_id"]
            self.NN_cmdRequest.process          = data_receive["process"]
            self.NN_cmdRequest.target_id        = data_receive["id_target"]
            self.NN_cmdRequest.target_dir       = data_receive["dir_target"]
            self.NN_cmdRequest.offset           = int(data_receive["offset"])
            self.NN_cmdRequest.moving_dir       = int(data_receive["dir_moving"])

            num_point = len(data_receive["routes"])
            for l in range(5):
                if l < num_point:
                    point = data_receive["routes"][l]
                    self.NN_cmdRequest.list_id[l]       = point["name"]
                    self.NN_cmdRequest.list_code[l]     = point["code"]
                    self.NN_cmdRequest.list_dir[l]      = int(point["direction"])
                    self.NN_cmdRequest.list_speed[l]    = point["speed"]
                    # self.NN_cmdRequest.list_encoder[l]  = point["encoder"]
                else:
                    self.NN_cmdRequest.list_id[l]       = 0
                    self.NN_cmdRequest.list_code[l]     = 0
                    self.NN_cmdRequest.list_dir[l]      = 0
                    self.NN_cmdRequest.list_speed[l]    = 0
                    # self.NN_cmdRequest.list_encoder[l]  = 0

            self.NN_cmdRequest.before_mission = data_receive["precode"]
            self.NN_cmdRequest.after_mission  = data_receive["subcode"]
            self.NN_cmdRequest.command        = data_receive["mess"]

            self.NN_cmdPub.publish(self.NN_cmdRequest)

        except Exception as e:
            print("Lỗi khi bóc tách dữ liệu JSON:", e)
            print(data_receive)

    def convertDataRosToServer(self):
        """Lấy dữ liệu từ self.NN_infoRespond => đóng gói JSON gửi lên server."""
        self.NN_is_infoReceived = 1
        if self.NN_is_infoReceived == 0:
            print("waiting data from sti_control")
            return ''
        
        self.NN_infoRespond.rfid_last_code = 1344
        self.NN_infoRespond.rfid_code = 3456

        frame_send = FrameSendServer(self.type_agv)
        frame_send["info"]["ip"]            = self.ip_robot
        frame_send["info"]["mac"]           = self.mac_robot
        frame_send["info"]["rfid_lastcode"] = self.NN_infoRespond.rfid_last_code
        frame_send["info"]["rfid_code"]     = self.NN_infoRespond.rfid_code
        frame_send["info"]["direction"]     = self.NN_infoRespond.direction
        frame_send["info"]["battery"]       = self.NN_infoRespond.battery
        frame_send["info"]["status"]        = self.NN_infoRespond.status
        frame_send["info"]["offset"]        = self.NN_infoRespond.offset
        frame_send["info"]["mode"]          = self.NN_infoRespond.mode
        frame_send["info"]["request"]       = 0
        frame_send["info"]["task_status"]   = self.NN_infoRespond.task_status
        frame_send["info"]["error_code"]    = self.convert_error(self.NN_infoRespond.list_error)

        try:
            return json.dumps(frame_send.to_dict())
        except Exception as e:
            print("Lỗi khi convert dict->json:", e)
            return ''
        
    def run(self):
        if self.process == 1:
            json_str = self.convertDataRosToServer()
            # print(json_str)
            if json_str:
                with dshare.data_lock:
                    dshare.dataROS = json_str

            self.process = 2

        elif self.process == 2:
            data = None
            with dshare.data_lock:
                if dshare.dataServer is not None:
                    # data = dshare.dataServer
                    data = json.loads(json.dumps(dshare.dataServer)) if dshare.dataServer else None
                    dshare.dataServer = None

            if data is not None:
                try:
                    # data_recieve = json.loads(data)
                    self.NN_cmdAnalysis(data)

                except Exception as e:
                    print(e)

            self.process = 1

        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)


def main():
    # Khởi tạo ROS
    rclpy.init()
    ros_node = ROSCommunication()

    # Run socketio
    threading.Thread(target=start_socketio, daemon=True).start()

    # Run ros
    try:
        rclpy.spin(ros_node)
        # executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')

if __name__ == "__main__":
    main()


# {
#   "type":2,         # kiểu frame
#   "info":{          # thông tin AGV
#     "ip":"",        # ip AGV
#     "mac":"",       # mac AGV
#     "x":0.0,        # toạ độ x
#     "y":0.0,        # toạ độ y
#     "r":0.0,        # góc
#     "battery":0.0,  # pin hiện tại
#     "status":0,     #
#     "mode":0,
#     "task_status":0,
#     "error_code":0
#   }
# }
