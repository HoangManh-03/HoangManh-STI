import asyncio
import websockets
import json

# get ip
import os                                                                                                                                                           
import re 

import sys
import struct
import time
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


class WebSocketClient:
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None
        self.is_connected = 0
        self.time_reconnect = 1.5
        self.savetime_reconnect = time.time()

    async def connect(self):
        try:
            self.websocket = await websockets.connect(self.uri, ping_interval=20, ping_timeout=20)
            self.is_connected = 1
            print("Connected to WebSocket server")
            local_address = self.websocket.local_address
            remote_address = self.websocket.remote_address
            print(f'Local address: {local_address}')
            print(f'Remote address: {remote_address}')

        except Exception as e:
            print(f"Failed to connect to WebSocket server: {e}")

    async def send_message(self, message):
        if self.websocket and self.is_connected:
            try:
                await self.websocket.send(message)
                # print(f"Sent message: {message}")
            except Exception as e:
                print(f"Failed to send message: {e}")
                self.is_connected = 0

    async def receive_message(self):
        if self.websocket and self.is_connected:
            try:
                message = await self.websocket.recv()
                # print(f"Received message: {message}")
                return message
            except Exception as e:
                print(f"Failed to receive message: {e}")
                self.is_connected = 0

    async def close(self):
        if self.websocket and self.is_connected:
            try:
                await self.websocket.close()
                print("WebSocket connection closed")
            except Exception as e:
                print(f"Failed to close WebSocket connection: {e}")

    async def reconnect(self):
        if time.time() - self.savetime_reconnect > self.time_reconnect:
            print("Attempting to reconnect...")
            self.savetime_reconnect = time.time()
            try:
                await self.connect()
            except Exception as e:
                print(f"Reconnection attempt failed: {e}")

class ROSCommunication():
    def __init__(self):
        rospy.init_node('stiClient', anonymous=False)
        self.rate = rospy.Rate(30)

        # -- add 22/01/2022
        self.name_card = rospy.get_param("name_card", "wlp0s20f3")
        self.name_card = "wlo2" # "wlp0s20f3"

        self.ip_robot = self.get_ipAuto(self.name_card)
        self.mac_robot = self.get_macAuto(self.name_card)
        
        rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.NN_infoCallback)	
        self.NN_infoRespond = NN_infoRespond()
        self.NN_is_infoReceived = 0 # 0

        self.NN_cmdPub = rospy.Publisher("/NN_cmdRequest", NN_cmdRequest, queue_size=50)
        self.NN_cmdRequest = NN_cmdRequest()

        self.NN_infoRequestPub = rospy.Publisher("/NN_infoRequest", NN_infoRequest, queue_size=50)
        self.NN_infoRequest = NN_infoRequest()		

        self.NN_cmdRequest.list_id = [0, 0, 0 ,0 , 0]
        self.NN_cmdRequest.list_x = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.NN_cmdRequest.list_y = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.NN_cmdRequest.list_speed = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.before_mission = 0  # nhiem vu can thuc hien truoc khi di chuyen
        self.after_mission = 0  # nhiem vu can thuc hien sau khi di chuyen den dich

    def NN_infoCallback(self, dat):
        self.NN_infoRespond = dat
        self.NN_is_infoReceived = 1

    def convert_angle(self, angle_rad):
        angle_deg = degrees(angle_rad)
        if angle_deg < 0:
            converted_angle = 180 + (-angle_deg)
        else:
            converted_angle = angle_deg
        return converted_angle


    def NN_cmdAnalysis(self, data_receive):
        try:
            # pub info request by Server
            self.NN_infoRequest.id_agv              = data_receive["id_agv"]
            self.NN_infoRequest.name_agv            = data_receive["name_agv"]
            # --
            self.NN_infoRequestPub.publish(self.NN_infoRequest)
            # -- 
            self.NN_cmdRequest.id_command           = data_receive["id_command"]
            self.NN_cmdRequest.process              = data_receive["process"]
            self.NN_cmdRequest.tag                  = data_receive["tag"]

            self.NN_cmdRequest.target_id            = data_receive["target_id"]
            self.NN_cmdRequest.target_x             = data_receive["target_x"]
            self.NN_cmdRequest.target_y             = data_receive["target_y"]
            self.NN_cmdRequest.target_z             = data_receive["target_z"]
            self.NN_cmdRequest.offset               = data_receive["offset"]

            num_point = len(data_receive["list_point"])
            for l in range(5):
                if l < num_point:
                    self.NN_cmdRequest.list_id[l] = data_receive["list_point"][l]["id"]
                    self.NN_cmdRequest.list_x[l] = data_receive["list_point"][l]["x"]
                    self.NN_cmdRequest.list_y[l] = data_receive["list_point"][l]["y"]
                    self.NN_cmdRequest.list_speed[l] = data_receive["list_point"][l]["speed"]

                else:
                    self.NN_cmdRequest.list_id[l] = 0
                    self.NN_cmdRequest.list_x[l] = 0.0
                    self.NN_cmdRequest.list_y[l] = 0.0
                    self.NN_cmdRequest.list_speed[l] = 0

            self.NN_cmdRequest.before_mission       = data_receive["before_mission"]
            self.NN_cmdRequest.after_mission        = data_receive["after_mission"]
            self.NN_cmdRequest.command              = data_receive["command"]

            self.NN_cmdPub.publish(self.NN_cmdRequest)

        except Exception as e:
            print("loi khi boc tach du lieu json, error: ", e)


    def convert_error(self, x):
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

    def coordinates_to_bytes_vs3(self, value):
        _v = int(value*1000)
        return struct.pack('>i', _v)
    
    # # -- add 22/01/2022
    def get_ipAuto(self, name_card): # name_card : str()
        try:
            address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
            return address
        except Exception:
            return "-1"
        
    def get_macAuto(self, name_card):
        try:
            mac = re.search(re.compile(r'(?<=link/ether )(.*)(?=\ b)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
            return mac
        except Exception:
            return "-1"

    def coordinates_to_bytes_vs4(self, _value):
        value = int(_value*1000)
        v_out = b'0x00'
        v_out += ((value>>24)&0xFF).to_bytes(1, "big")
        v_out += ((value>>16)&0xFF).to_bytes(1, "big")
        v_out += ((value>>8)&0xFF).to_bytes(1, "big")
        v_out += (value&0xFF).to_bytes(1, "big")
        return v_out
    
    def convertDataRosToServer(self):
        if self.NN_is_infoReceived == 0:
            print("waiting data from sti_control")
            return ''
        
        # cv_angle = self.convert_angle(self.NN_infoRespond.z)
        # frame_send = {"ip"          : self.ip_robot,
        #               "mac"         : self.mac_robot,
        #               "x"           : self.NN_infoRespond.x,
        #               "y"           : self.NN_infoRespond.y,
        #               "r"           : cv_angle,
        #               "battery"     : self.NN_infoRespond.battery,
        #               "status"      : self.NN_infoRespond.status,
        #               "tag"         : self.NN_infoRespond.tag,
        #               "mode"        : self.NN_infoRespond.mode,
        #               "task_status" : self.NN_infoRespond.task_status,
        #               "error_code"  : 0
        #             }

        frame_send = {
            "AGV_ID": self.AGV_ID,
            "RFIDLastCode": self.RFIDLastCode,
            "RFIDTargetCode": self.RFIDTargetCode,
            "Velocity_real": self.vel,
            "Status": self.status,
            "Direction": self.Dir,
            "BatteryLevel": self.battery
        }
        # convert sang json
        try:
            return json.dumps(frame_send, indent = 4)
        except Exception as e:
            print("loi khi convert dic to str json, error: ", e)
            return ''

async def main():
    # - ip | port server
    ip_server = '192.168.1.84'
    port_server = '8888'
    websocket_uri = "ws://" + ip_server + ":" + port_server
    # - websocket
    web_comm = WebSocketClient(websocket_uri)
    await web_comm.connect()
    data_recieve = ''
    savetime_sendServer = time.time()
    # - ROS
    ros_comm = ROSCommunication()
    # -
    process = 0

    try:
        while not rospy.is_shutdown():
            # kiem tra ket noi
            if process == 0:
                if web_comm.is_connected == 1:
                    process = 1
                else:
                    await web_comm.reconnect()

            if process == 1:
                dentalTime = time.time() - savetime_sendServer
                if dentalTime > 3.5:
                    savetime_sendServer = time.time()
                    str_send = ros_comm.convertDataRosToServer()
                    if str_send:
                        # await web_comm.send_message(str_send)
                        await web_comm.send_message("helooooooooooo")
                        if web_comm.is_connected == 1:
                            process = 2  
                        else:
                            print("dong ket noi")
                            process = 0

            elif process == 2:
                mess_rec = await web_comm.receive_message()
                print("data recievce: ", mess_rec)
                # them dieu kien kiam tra tin nhan
                if web_comm.is_connected == 1 and mess_rec:
                    try:
                        data_recieve = json.loads(mess_rec)
                        process = 3
                    except Exception as e:
                        print("loi khi chuyen string to json, error: ", e)
                        process = 1

                else:
                    print("dong ket noi")
                    process = 0

            elif process == 3:
                ros_comm.NN_cmdAnalysis(data_recieve)
                process = 1

            print(process)

            ros_comm.rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        await web_comm.close()

if __name__ == "__main__":
    asyncio.run(main())
