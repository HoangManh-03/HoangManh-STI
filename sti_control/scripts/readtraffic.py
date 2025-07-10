#!/usr/bin/env python3

import socket
import rospy
import os                                                                                                                                                           
import re 

import sys
import struct
import time
from decimal import *
import math
import rospy
from datetime import datetime

from sti_msgs.msg import NN_cmdRequest     # request to client udp
from sti_msgs.msg import NN_infoRequest     # request to client udp
from sti_msgs.msg import NN_infoRespond     # respond from client udp

from std_msgs.msg import Int16, Int8, Bool

import signal

""" Quy định tất cả dữ liệu được gắn trực tiếp vào biến trong class frame phải ở dạng của server:
 	VD: 
  	1, Tọa độ x, y theo mm
		x,y (ros) = 3,12 m -> x(server) = 3.12*1000 = 312 mm 
 	2, Góc: từ rad -> 360 độ 
		z(ros) = 1.047 (rad) -> z(server) = 59.9 (degree) *100 = 5990 

"""

""" 
1: 27
2-
"""

class UDPReceiver:
    def __init__(self, ip, port):
        rospy.init_node('test_client', anonymous=False)
        self.rate = rospy.Rate(100)

        self.ip = ip
        self.PORT_sento = 8000
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.pub_control = rospy.Publisher("/NN_cmd_a", Int8, queue_size=50)
        self.pub_data = Int8()
        
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

        
    def receive_data(self):
        while True:
            try:
                self.data_received, addr = self.sock.recvfrom(1024)  # Nhận dữ liệu với kích thước tối đa 1024 bytes
                print(f"Received message from {addr}: {self.data_received.decode('utf-8')}")
                print(f"Hellooooooooooooooooooooooooo")
                rospy.loginfo ("lenght frame: %s - Frame: %s", self.byte_to_int(self.data_received[0]), self.data_received[5])
                # self.pub_data = self.data_received[5]
            
                
                self.pub_control.publish(self.pub_data)
            except socket.error as e:
                rospy.logerr(f"Socket error: {e}")
                break

    def byte_to_int(self, byt): # byte to int (standard)
        x = 0
        x = int(byt)
        # print ("byt: ", byt)
        # foo.encode('utf-8').strip()
        # x = int(byt.encode('hex'), 16)
        return x
    
    def int_to_byte(self, val): # int to a bytes
        if val > 255:
            rospy.logerr("int_to_byte: Val error: %s", val)
            val = 255
        elif val < 0:
            rospy.logerr("int_to_byte: Val error: %s", val)
            val = 0
        return struct.pack("B", int(val)) # bytes(chr(int(val)), 'ascii')
	
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

    def shutdown(self, signum, frame):
        rospy.loginfo("Shutting down...")
        self.sock.close()
        rospy.signal_shutdown("Shutdown signal received")

    def bytes_list_to_int(self, list_byte, pos_byte, n_byte): # int (256)
        n = 0
        x = 0
        for t in range(0, n_byte):
            x = int(list_byte[t + pos_byte])
            n += x*pow(256, n_byte - t - 1)
        return int(n)

################################################################
#############################STI################################
################################################################

    def NN_infoBuild(self):
        data_raw = b''

        data_raw += self.coordinates_to_bytes_vs2(self.NN_infoRespond.x)
        # data_raw += self.coordinates_to_bytes_vs2(123) 
        data_raw += self.coordinates_to_bytes_vs2(self.NN_infoRespond.y)
        # data_raw += self.coordinates_to_bytes_vs2(-70.0)
        # - old
        # data_raw += self.direction_to_bytes(self.NN_infoRespond.z) 
        # - new
        data_raw += self.direction_to_bytes_new(self.NN_infoRespond.z)

        data_raw += self.int_to_bytes(self.NN_infoRespond.tag, 2)
        # print ("tag hex: ", self.int_to_bytes(self.NN_infoRespond.tag, 2) )
        
        data_raw += self.int_to_byte(self.NN_infoRespond.battery)
        # data_raw += self.int_to_byte(254)
        
        # -- edit 26/02/2022
        # data_raw += self.int_to_byte(self.NN_infoRespond.status)
        
        error_n = self.getError_new(self.NN_infoRespond.listError)
        # print ("ER: ", self.convert_error(error_n) )
        data_raw += self.int_to_byte(self.convert_error(error_n))

        data_raw += self.int_to_byte(self.NN_infoRespond.mode)
        data_raw += self.int_to_byte(0)
        data_raw += self.int_to_byte(self.NN_infoRespond.task_status)
        return data_raw

################################################################
#############################STI################################
################################################################
    def NN_cmdAnalysis_vs2(self, data_receive):
        
        self.NN_cmdRequest.id_command = self.bytes_list_to_int(data_receive, 2, 4)
        self.NN_cmdRequest.current_pos = self.byte_list

################################################################
#############################STI################################
################################################################

    def run(self):
        if self.process == 0:
            self.process = 1
        
        elif self.process == 1:
            try:
                self.data_received, addr = self.sock.recvfrom(1024)  # Nhận dữ liệu với kích thước tối đa 1024 bytes
                self.HOST_sento = addr[0]
                print(f"Hellooooooooooooooooooooooooo")
                rospy.loginfo ("lenght frame: %s - Frame: %s", self.byte_to_int(self.data_received[0]), self.data_received[5])
            except socket.error:
                self.log_mess("err", "Error -- recv() --!", 0)
                
            if len(self.data_received) != 0:
                self.process = 2
            else:
                self.process = 1
                self.log_mess("warn", "not data receive", 0)
        
        elif self.process == 2: #Check len data
            if self.byte_to_int(self.data_received[0]) != len(self.data_received):
                rospy.logerr ("Error lenght frame: %s - want: %s - current: %s", self.data_received[1], self.byte_to_int(self.data_received[0]), len(self.data_received))
            else:
                self.process = 3
        elif self.process == 3:
            if (self.byte_to_int(self.receive_data[1]) == 85):
                self.process = 4
            elif self.data_received[1] == 67:
                self.process = 5
            else:
                rospy.logwarn("receive data dif frame")
                rospy.logwarn(self.data_received[1])
                
        elif self.process == 4:
            try:
                if self.NN_infoReceived == 1:   #Lấy được data từ topic
                    data_raw_info = self.NN_infoRFline()
                    frame_data = b''
                    frame_data += self.int_to_byte(27) + self.int_to_byte(self.data_received[1]) + self.build_stringByte(self.data_received, 2, 4) + data_raw_info
                    self.udp.sendto(frame_data, (self.HOST_sento, self.PORT_sento))
            except socket.error:
                rospy.logerr ("NN: Can not send Info!")

if __name__ == "__main__":
    udp_ip = "192.168.1.43"
    udp_port = 8888  # Cổng bạn muốn nghe, có thể thay đổi

    receiver = UDPReceiver(udp_ip, udp_port)
    print(f"Listening for UDP packets on {udp_ip}:{udp_port}")
    receiver.receive_data()
