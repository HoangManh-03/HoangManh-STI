#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import roslib

# get ip
import os                                                                                                                                                           
import re 

import sys
import struct
import time
from decimal import *
import math
import rospy
from datetime import datetime

import json

from sti_msgs.msg import NN_cmdRequest     # request to client udp
from sti_msgs.msg import NN_infoRequest     # request to client udp
from sti_msgs.msg import NN_infoRespond     # respond from client udp


# import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point

""" Quy định tất cả dữ liệu được gắn trực tiếp vào biến trong class frame phải ở dạng của server:
 	VD: 
  	1, Tọa độ x, y theo mm
		x,y (ros) = 3,12 m -> x(server) = 3.12*1000 = 312 mm 
 	2, Góc: từ rad -> 360 độ 
		z(ros) = 1.047 (rad) -> z(server) = 59.9 (degree) *100 = 5990 

"""

#------------------------------------------------------------- SUB


# Datagram (udp) socket
class read_and_respond_TCP():
    def __init__(self):
        rospy.init_node('stiClient_nav', anonymous=False)
        self.rate = rospy.Rate(100)

        # -- add 22/01/2022
        self.name_card = rospy.get_param("name_card", "wlp0s20f3")
        self.name_card = "wlo2" # "wlp0s20f3"
        
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

        self.HOST_receive = '192.168.1.67'
        # self.HOST_receive = self.get_ipAuto(self.name_card)
        self.PORT_receive = 8000 # 8888
        self.PORT_sento = 8000   # 8000
        self.server_address = (self.HOST_receive, self.PORT_receive)

        self.time_wait_respond = 0.01   # thoi gian cho phan hoi
        self.process = 0
        self.data_raw_info = ''
        self.pre_mess = ""
        self.data_received = ''

        self.frequency_pub_info = 16.
        self.frequency_pub_cmd = 16.
        self.pre_time_info = 0.0
        self.pre_time_cmd = 0.0

        # Header Frame info NN
        self.NN_infoSum = ''
        self.NN_infoId = 'U'
        self.NN_infoNameU300L = ['','','','']

        # Header Frame info FL
        self.FL_infoSum = ''
        self.saveTime_recieveTCP = rospy.get_time()
        self.saveTime_reconnectServer = rospy.get_time()

        self.is_connectedServer = self.create_connection()

    def create_connection(self):
        # -- Create connection
        try :
            self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print ('Socket created')
        except socket.error as e: #, msg:
            print ('Failed to create socket. Error Code : ', e ) # + str(msg[0]) + ' Message ' + msg[1])
            return 0
            # sys.exit()

        #-- Connect to Server TCP   
        try:
            self.tcp.connect(self.server_address)
            print("connected to server tcp : %s" + str(self.server_address) )
            # self.process = 2
            
        except socket.error as e:
            print(e)
            return 0

        return 1
        

    def NN_infoCallback(self, dat):
        self.NN_infoRespond = dat
        self.NN_is_infoReceived = 1

    # #--------------------------------------------------------------------------------- manipulation with point
    # #-------------------------------------------------------------- CONVERT
    # #-- list bytes to int (256)
    # def bytes_list_to_coor(self, list_byte, pos_byte): # 4 byte
    #     n = 0
    #     x = 0
    #     v = pos_byte + 1
    #     d = int(list_byte[pos_byte])
    #     for t in range(4):
    #         x = int(list_byte[t+v])
    #         n += x*pow(256, 3 - t - 1)
        
    #     if d == 1:    # am
    #         return (n/1000.)*(-1)   # đảm bảo khác 0
    #     elif d == 2:  # duong:
    #         return (n/1000.)
    #     elif d == 0:  # ko xac dinh:
    #         return 1000
    #     else:
    #         print ("d: ", d)
    #         return 5000

    # def bytes_list_to_coor_vs1(self, list_byte, pos_byte): # 4 byte
    #     n = 0
    #     x = 0
    #     v = pos_byte + 1

    #     for t in range(4):
    #         x = int(list_byte[pos_byte + t])
    #         n += x*pow(256, 3 - t)

    #     val = 0
    #     if (n > pow(256, 3)):
    #         val = (n - pow(256, 4))/1000.
    #     else:
    #         val  = n/1000.
    #     return val

    # def bytes_list_to_corner(self, list_byte, pos_byte): # 2 byte
    #     n = 0
    #     x = 0
    #     for t in range(0,2):
    #         x = int(list_byte[pos_byte + t])
    #         n += x*pow(256, 2 - t - 1)

    #     if n == 40000:
    #         return 4
    #     else:
    #         c = n/100. # độ
    #         if c > 180:
    #             c = 360 - c
    #             return (c/180)*math.pi*(-1)
    #         else:
    #             return (c/180)*math.pi

    # def coordinates_to_bytes_vs1(self, co):
    #     ss = b''
    #     if (co >= 0):
    #         ss += self.int_to_byte(0)
    #     else:
    #         ss += self.int_to_byte(1)
    #     x = 0
    #     t = 0
    #     co *= 1000
    #     val = 0
    #     # value
    #     val = int(abs(co))
    #     # print ("val:", val)
    #     ss += self.int_to_bytes(val, 4)
    #     return ss

    def coordinates_to_bytes_vs2(self, co):
        ss = b''
        # if (co >= 0):
        ss += self.int_to_byte(0)
        # else:
            # ss += self.int_to_byte(1)

        x = 0
        t = 0
        co *= 1000
        val = 0
        # value
        val = int(co)
        # print ("val:", val)
        ss += self.intA_to_bytes(val, 4)
        return ss

    # def coordinates_to_bytes(self, co): # ????????????
    #     ss = b''
    #     x = 0
    #     t = 0
    #     co *= 1000
    #     val = 0
    #     # unknown/-/+
    #     if co > 500:
    #         ss += self.int_to_byte(0)
    #     elif co < 0:
    #         ss += self.int_to_byte(1)
    #     else:
    #         ss += self.int_to_byte(2)
    #     # value
    #     val = int(abs(co))
    #     for i in range(0, 4):
    #         t += pow(256, 4 - i)*x
    #         x = (val - t)/pow(256, 3 - i) 
    #         # print(x)
    #         ss += self.int_to_byte(int(x))
    #     return ss
            
    # def direction_to_bytes(self, dir): # chuyen toa do tu Rad {-pi,pi} -> {0,360} -> scale: 100 - 2 byte
    #     if dir >= 4:
    #         return self.int_to_bytes(40000, 4)
    #     elif dir >= 0 and dir < 4:
    #         d = math.degrees(dir)*100
    #         # print(d)
    #         return self.int_to_bytes(int(d), 4)
    #     else:
    #         d = (360 + math.degrees(dir))*100
    #         return self.int_to_bytes(int(d), 4)

    # def bytes_list_to_int(self, list_byte, pos_byte, n_byte): # int (256)
    #     n = 0
    #     x = 0
    #     for t in range(0, n_byte):
    #         x = int(list_byte[t + pos_byte])
    #         n += x*pow(256, n_byte - t - 1)
    #     return int(n)

    # def byte_to_int(self, byt): # byte to int (standard)
    #     x = 0
    #     x = int(byt)
    #     # print ("byt: ", byt)
    #     # foo.encode('utf-8').strip()
    #     # x = int(byt.encode('hex'), 16)
    #     return x

    # def bytes_to_int(self, bytes_): # bytes (multiple) to int (256)
    #     v = 0
    #     for t in range(0,len(bytes_)):
    #         x = int(list_byte[t+pos_byte].encode('hex'), 16)
    #         v =+ x*pow(256, n_byte - t - 1)
    #     return v

    # def int_to_bytes(self, val, n): # int to n bytes
    #     ss = b''
    #     x = 0
    #     t = 0
    #     for i in range(0, n):
    #         t += pow(256, n - i)*x
    #         x = int((val - t)/pow(256, n - i - 1) )
    #         ss += self.int_to_byte(x)
    #     return ss

    def intA_to_bytes(self, val, n): # int to n bytes
        ss = b''
        x = 0
        t = 0

        if (val >= 0):
            val_1 = val
        else:
            val_1 = pow(256, 4) + val

        for i in range(0, n):
            t += pow(256, n - i)*x
            x = int((val_1 - t)/pow(256, n - i - 1) )
            ss += self.int_to_byte(x)
        return ss

    # def offset_to_bytes(self, val): # int to 2 bytes - val mét
    #     ss = b''

    #     x = 0
    #     t = 0
    #     val = val*1000 # chuyen sang 'mm'
    #     for i in range(2):
    #         t += pow(256, 2 - i)*x
    #         x = (val - t)/pow(256, 2 - i - 1) 
    #         # print(x)
    #         ss += self.int_to_byte(x)
    #     return ss

    # def tag_to_bytes(self, val): # int to 2 bytes - int 0 -> 255
    #     ss = b''
    #     x = 0
    #     t = 0
    #     val = val
        
    #     for i in range(2):
    #         t += pow(256, 2 - i)*x
    #         x = (val - t)/pow(256, 2 - i - 1) 
    #         # print(x)
    #         ss += self.int_to_byte(x)
    #     return ss

    # def bytes_list_to_offset(self, list_byte, pos_byte): # 4 byte
    #     n = 0
    #     X = 0
    #     for t in range(0, 4):
    #         x = int(list_byte[t + pos_byte])
    #         n += x*pow(256, 4 - t - 1)
    #     return n/1000.

    # def bytes_list_to_tag(self, list_byte, pos_byte): # 2 byte
    #     n = 0
    #     X = 0
    #     for t in range(0,2):
    #         x = int(list_byte[t+pos_byte].encode('hex'), 16)
    #         n += x*pow(256, 2 - t - 1)
    #     return n

    def int_to_byte(self, val): # int to a bytes
        if val > 255:
            rospy.logerr("int_to_byte: Val error: %s", val)
            val = 255
        elif val < 0:
            rospy.logerr("int_to_byte: Val error: %s", val)
            val = 0
        return struct.pack("B", int(val)) # bytes(chr(int(val)), 'ascii')

    # def build_stringByte(self, list_byte, pos_byte, n_byte): # list | start position read | number byte  -> return array string
    #     ss = b''
    #     for t in range(0, n_byte):
    #         ss += bytes(chr(list_byte[pos_byte + t]), 'ascii')
    #     return ss

    # def build_string(self, list_byte, pos_byte, n_byte): # list | start position read | number byte  -> return array string
    #     ss = ''
    #     for t in range(0, n_byte):
    #         ss += chr(list_byte[pos_byte + t])
    #     return ss

    # #-- chuyen toa do tu float -> chuoi 4 byte dang '\x' : byte 0: +/- byte {1:3}: gia tri toa do theo mm.
    # #----------- Check a point in a Circle ? : point | center circle(point) | radius
    # def check_point_in_list(self, po, ls):
    #     pass

    # def find_location(self, name_, list_): # find_location of a point in list
    #     pass	

    # #--------------------------------------------------------------------------------- analysis data receive
    # def NN_infoAnalysis(self, data_receive):
    #     self.NN_infoSum = self.byte_to_int(data_receive[0])
    #     self.NN_infoNameU300L = self.bytes_list_to_int(data_receive,2,4)


    # def NN_cmdAnalysis(self, data_receive):
    #     # print len(data_receive)
    #     ll = 0
    #     ll = len(data_receive)
    #     # -- add new
    #     self.NN_cmdRequest.id_command = self.bytes_list_to_int(data_receive, 2, 4)

    #     self.NN_cmdRequest.process = self.byte_to_int(data_receive[6])

    #     self.NN_cmdRequest.tag =  self.bytes_list_to_int(data_receive, 7, 2)
    #     self.NN_cmdRequest.target_id = self.bytes_list_to_int(data_receive, 9, 2)

    #     self.NN_cmdRequest.target_x = round(self.bytes_list_to_coor_vs1(data_receive, 9) , 3)
    #     self.NN_cmdRequest.target_y = round(self.bytes_list_to_coor_vs1(data_receive, 13) , 3)
            
    #     self.NN_cmdRequest.target_z = round(self.bytes_list_to_corner(data_receive, 17), 3)
        
    #     # self.NN_cmdRequest.target_id = 
    #     self.NN_cmdRequest.offset = self.bytes_list_to_offset(data_receive, 19) # 4 bytes

    #     for l in range(5):
    #         self.NN_cmdRequest.list_id[l] = self.bytes_list_to_int(data_receive, 23 + l*11, 2)
    #         self.NN_cmdRequest.list_x[l] = round(self.bytes_list_to_coor_vs1(data_receive, 25 + l*11), 3)
    #         self.NN_cmdRequest.list_y[l] = round(self.bytes_list_to_coor_vs1(data_receive, 29 + l*11), 3)
    #         self.NN_cmdRequest.list_speed[l] = int(data_receive[33 + l*11])

    #     self.NN_cmdRequest.before_mission = self.byte_to_int(data_receive[78])
    #     self.NN_cmdRequest.after_mission = self.byte_to_int(data_receive[79])

    #     if ll > 80:
    #         self.NN_cmdRequest.command = self.build_string(data_receive, 80, ll - 80)
    #     else:
    #         self.NN_cmdRequest.command = ""


    # def NN_cmdBuild(self, data_receive):
    #     data_raw = ''
    #     data_raw = '\x06' + 'R' + data_receive[2] + data_receive[4] + data_receive[4] + data_receive[5]
    #     return data_raw 	

    # def convert_error(self, x):
    #     switcher={
    #         # NN
    #         0:0,   # ALL RIGHT
    #         111:1,  # Va vào Blsock.
    #         121:2, # Ấn EMG.
    #         131:3, # Ra khỏi đường từ.
    #         141:4, # Bàn nâng.
    #         211:5, # Camera:Mất kết nối vật lý.
    #         212:6, # Camera: Không giao tiếp truyền thông.
    #         221:7, # Lidar. Phía trước.
    #         222:8, # Lidar. Phía sau.
    #         223:9, # Lidar. Cả 2.
    #         231:10, # IMU Không giao tiếp truyền thông.
    #         241:11, # PS2 Không giao tiếp truyền thông.
    #         251:12, # DRIVER 1 (trái) NN – Natual Navigatiroson.
    #         261:13, # DRIVER 2 (phải) NN – Natual Navigation.
    #         271:14, # Mangnetic line: NN - RS232 – PC lỗi (phía trước).
    #         272:15, # Mangnetic line: NN - RS232 – PC lỗi (phía sau).
    #         311:16, # Mạch MC - NN.	Mất kết nối vật lý Serial.
    #         312:17, # Mạch MC - NN. Không giao tiếp truyền thông Serial.
    #         321:18, # Mạch Main - NN:	Mất kết nối vật lý Serial.
    #         322:19, # Mạch Main - NN: Không giao tiếp truyền thông Serial.
    #         331:20, # Mạch SC: Mất kết nối vật lý Serial.
    #         332:21, # Mạch SC: Không giao tiếp truyền thông Serial.		
    #         341:22,	# Mạch OC: Mất kết nối vật lý Serial.
    #         342:23,	# Mạch OC: Không giao tiếp truyền thông Serial.
    #         351:24,	# Mạch HC: Mất kết nối vật lý Serial.
    #         352:25,	# Mạch HC: Không giao tiếp truyền thông Serial.
    #         411:201, # Di chuyển NN (không vạch từ)- Không thể đến được đích.
    #         421:202, # Có vật cản.
    #         431:203, # Mất kết nối server.
    #         441:204, # Không thể thấy Tag.
    #         451:205, # Điện Áp Thấp.
    #         461:206, # Có vật cản khi vào kệ.
    #         471:207  # không có kệ hoặc lệch kệ khi nâng.
    #     }
    #     return switcher.get(x, 0)		

    # def NN_infoBuild(self):
    #     data_raw = b''

    #     data_raw += self.coordinates_to_bytes_vs2(self.NN_infoRespond.x)
    #     # data_raw += self.coordinates_to_bytes_vs2(123) 
    #     data_raw += self.coordinates_to_bytes_vs2(self.NN_infoRespond.y)
    #     # data_raw += self.coordinates_to_bytes_vs2(-70.0)

    #     data_raw += self.direction_to_bytes(self.NN_infoRespond.z) 
    #     # data_raw += self.direction_to_bytes(-2.141)

    #     data_raw += self.int_to_bytes(self.NN_infoRespond.tag, 2)
    #     # print ("tag hex: ", self.int_to_bytes(self.NN_infoRespond.tag, 2) )
        
    #     data_raw += self.int_to_byte(self.NN_infoRespond.battery)
    #     # data_raw += self.int_to_byte(254)
        
    #     data_raw += self.int_to_byte(self.NN_infoRespond.status)

    #     data_raw += self.int_to_byte(self.NN_infoRespond.mode)
    #     data_raw += self.int_to_byte(0)
    #     data_raw += self.int_to_byte(self.NN_infoRespond.task_status)
    #     return data_raw

    # def log_mess(self, typ, mess, val):
    #     if self.pre_mess != mess:
    #         if typ == "info":
    #             rospy.loginfo (mess + ": %s", val)
    #         elif typ == "warn":
    #             rospy.logwarn (mess + ": %s", val)
    #         else:
    #             rospy.logerr (mess + ": %s", val)
    #     self.pre_mess = mess

    # # -- add 22/01/2022
    # def get_ipAuto(self, name_card): # name_card : str()
    #     try:
    #         address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
    #         print ("address: ", address)
    #         return address
    #     except Exception:
    #         return "-1"


    def coordinates_to_bytes_vs3(self, value):
        _v = int(value*1000)
        return struct.pack('>i', _v)

    def coordinates_to_bytes_vs4(self, _value):
        value = int(_value*1000)
        v_out = b'0x00'
        v_out += ((value>>24)&0xFF).to_bytes(1, "big")
        v_out += ((value>>16)&0xFF).to_bytes(1, "big")
        v_out += ((value>>8)&0xFF).to_bytes(1, "big")
        v_out += (value&0xFF).to_bytes(1, "big")
        return v_out


    #--------------------------------------------------------------------------------- Read and respond
    def run(self):	
        if self.process == 0:
            self.process = 1

        elif self.process == 1: # connect to server tcp
            if self.is_connectedServer == 1:
                try:
                    self.data_received = self.tcp.recv(2048)
                    json_data = json.loads(str(self.data_received, 'utf-8'))
                    print("data_received:", json_data["list_point"][0]["id"], "\n")
                    # kiem tra data
                    if len(self.data_received) > 0:
                        self.process = 2

                except socket.error as e:
                    print("process = 1, error: ", e)
                    self.is_connectedServer = 0

            else:
                dental_time = rospy.get_time() - self.saveTime_reconnectServer
                if dental_time > 1.:
                    print("reconnect server tcp... ")
                    self.is_connectedServer = self.create_connection()
                    self.saveTime_reconnectServer = rospy.get_time()

        elif self.process == 2: #
            self.process = 3

        elif self.process == 3: # nhan dien frame
            self.process = 4
            # self.process = 5

        elif self.process == 4: # info NN - analysis and respond
            try :
                mm = b'xin chao nhe'
                x = int(3.23*1000)
                y = int(5.47*1000)

                self.tcp.sendall(mm)
            
            except socket.error as e:
                print(e)
                self.is_connectedServer = 0

            self.process = 1

        # elif self.process == 2: # kiem tra kich thuoc cua data
        #     if self.byte_to_int(self.data_received[0]) != len(self.data_received):
        #         rospy.logerr ("Error lenght frame: %s - want: %s - current: %s", self.data_received[1], self.byte_to_int(self.data_received[0]), len(self.data_received))
        #     else:
        #         self.process = 3
        # elif self.process == 3: # nhan dien frame

        #     if (self.byte_to_int(self.data_received[1]) == 85): # info NN 'U'
        #         self.process = 4
        #     elif self.data_received[1] == 67: # command NN 'C'
        #         self.process = 5
        #     else:
        #         rospy.logwarn("Recivce dif frame")
        #         rospy.logwarn(self.data_received[1])
        #         self.process = 1

        # elif self.process == 4:  # info NN - analysis and respond
        #     try:
        #         if self.NN_is_infoReceived == 1: # 1:
        #             data_raw_info = self.NN_infoBuild()
        #             mm = b''

        #             mm += self.int_to_byte(27) + self.int_to_byte(self.data_received[1]) + self.build_stringByte(self.data_received, 2, 4) + data_raw_info 
        #             # print ("len(data_raw_info): ", len(data_raw_info))
        #             # abc = []
        #             # for i in range(len(mm)):
        #                 # abc.append(self.byte_to_int(mm[i]))
        #             # print ("sent:", mm)

        #             # st = ''
        #             # for i in mm:
        #             # 	st = st + str(int(i)) + ' '
                    
        #             # print(st)

        #             self.udp.sendto(mm, (self.HOST_sento, self.PORT_sento))
        #         else:
        #             rospy.logwarn("Wait respond from Sti_control")

        #     except socket.error:
        #         rospy.logerr ("NN: Can not send Info!")

        #     # pub info request by Server
        #     self.NN_infoRequest.id_agv = self.bytes_list_to_int(self.data_received, 2, 4)

        #     lenght = len(self.data_received)
        #     if (lenght > 6):
        #         self.NN_infoRequest.name_agv = self.build_string(self.data_received, 6, lenght - 6 )
        #     else:
        #         self.NN_infoRequest.name_agv = ''

        #     self.NN_infoRequestPub.publish(self.NN_infoRequest)
        #     self.process = 1

        # elif self.process == 5:  # command NN - analysis and respond
        #     rr = self.int_to_byte(6) + self.int_to_byte(self.data_received[1]) + self.int_to_byte(self.data_received[2] + self.data_received[3]) + self.int_to_byte(self.data_received[4]) + self.int_to_byte(self.data_received[5])
        #     time.sleep(self.time_wait_respond)
        #     try:
        #         self.udp.sendto(rr, (self.HOST_sento, self.PORT_sento))
        #     except socket.error:
        #         rospy.logerr ("NN: Can not send Command!")

        #     if (len(self.data_received) >= 80):
        #         self.NN_cmdAnalysis(self.data_received)
        #         self.NN_cmdPub.publish(self.NN_cmdRequest)
                
        #     else:
        #         rospy.logerr("NN: Error size of Frame command: %s", len(self.data_received))
        #     self.process = 1

        self.rate.sleep()

def main():

    print('Starting main program')
    # Start the job threads
    class_1 = read_and_respond_TCP()		
    # Keep the main thread running, otherwise signals are ignored.
    while not rospy.is_shutdown():
        class_1.run()
    class_1.tcp.close()

if __name__ == '__main__':
    main()
    