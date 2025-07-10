#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socketio
import json 

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

from sti_msgs.msg import NN_cmdRequest   
from sti_msgs.msg import NN_infoRequest  
from sti_msgs.msg import NN_infoRespond 

# import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sti_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

# is_connected = 0

class Communicate_ROS():
    def __init__(self):
        rospy.init_node('ForwardSimu_node', anonymous=False)
        self.rate = rospy.Rate(30)
        # -

        rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.callback_cmdRequest)	
        self.NN_cmdSub = None
        self.is_cmdRequest = 0

        rospy.Subscriber("/NN_infoRequest", NN_infoRequest, self.callback_infoRequest)	
        self.NN_infoRequest = NN_infoRequest
        self.NN_infoRequest_sio = None
        self.is_infoRequest = 0
		
        # -
        self.NN_infoRespondPub = rospy.Publisher("/NN_infoRespond", NN_infoRespond, queue_size=50)
        self.NN_infoPub = NN_infoRespond()
        self.is_infoRespond = 0
        # -
        # self.NN_infoRequestPub = rospy.Publisher("/NN_infoRequest", NN_infoRequest, queue_size=50)
        # self.server_requestInfor = NN_infoRequest()
        
        # ---------
        self.saveTime_received = rospy.Time.now()

    def callback_cmdRequest(self, data):
        l_point = []
        for i in range(0, len(data.list_id)):
            point = {"id" : data.list_id[i], "x" : data.list_x[i], "y" : data.list_y[i], "speed" : data.list_speed[i]}
            l_point.append(point)
        
        self.NN_cmdSub = {"id_agv" : self.NN_infoRequest.id_agv,
                            "process" : data.process,
                            "target_id" : data.target_id,
                            "target_x" : data.target_x,
                            "target_y" : data.target_y,
                            "target_z" : data.target_z,
                            "tag" : data.tag,
                            "offset" : data.offset,
                            "list_point" : l_point,
                            "before_mission" : data.before_mission,
                            "after_mission" : data.after_mission,
                            "id_command" : data.id_command,
                            "command" : data.command
						  }
		
        # print(self.NN_cmdSub)
		
        self.is_cmdRequest = 1

    def callback_infoRequest(self, data):
        self.NN_infoRequest = data

        self.NN_infoRequest_sio = {"id_agv" : self.NN_infoRequest.id_agv, "name_agv" : self.NN_infoRequest.name_agv}
        self.is_infoRequest = 1
		
    def run(self):
        if self.is_infoRespond:
            self.is_infoRespond = 0

            self.NN_infoRespondPub.publish(self.NN_infoPub)

def main():
    print('Starting main program')
    # - Start the job threads
    myObject = Communicate_ROS()		
    my_socketIO = socketio.Client()

    server_IP = '192.168.1.39'
    server_port = 3001

    # -
    is_connected = 0
    time_save = time.time()

    @my_socketIO.on('Server_send_agv_info')
    def on_message(data):
        print('I received Request from Server!')
        data_json = json.loads(data)
        if data_json['id_agv'] == myObject.NN_infoRequest.id_agv:
            print("have infoRespond from AGV id = ", data_json['id_agv'])
            myObject.is_infoRespond = 1

            myObject.NN_infoPub.x = data_json["x"]
            myObject.NN_infoPub.y = data_json["y"]
            myObject.NN_infoPub.z = data_json["z"]
            myObject.NN_infoPub.offset = data_json["offset"]
            myObject.NN_infoPub.tag = data_json["tag"]
            myObject.NN_infoPub.battery = data_json["battery"]
            myObject.NN_infoPub.status = data_json["status"]
            myObject.NN_infoPub.task_status = data_json["task_status"]
            myObject.NN_infoPub.error_device = data_json["error_device"]
            myObject.NN_infoPub.error_moving = data_json["error_moving"]
            myObject.NN_infoPub.error_perform = data_json["error_perform"]

            le = []
            for e in data_json["listError"]:
                le.append(e)
            myObject.NN_infoPub.listError = le

            myObject.NN_infoPub.process = data_json["process"]

    @my_socketIO.event
    def connect():
        nonlocal is_connected
        print("I'm connected!")
        is_connected = 1

    @my_socketIO.event
    def connect_error(data):
        nonlocal is_connected
        print("The connection failed!")
        is_connected = 0

    @my_socketIO.event 
    def disconnect():
        nonlocal is_connected
        print("I'm disconnected!")
        is_connected = 0

    while not is_connected:
        try:
            print(f"Attempting to connect to server at {server_IP}:{server_port}...")
            my_socketIO.connect('http://' + server_IP +':' + str(server_port))

        except Exception as e:
            print(f"Connection attempt failed: {e}. Retrying in 2 seconds...")
            time.sleep(2)

    # - Keep the main thread running, otherwise signals are ignored.
    while not rospy.is_shutdown():
        # -- 
        myObject.run()
        # -- 
        # print(myObject.is_cmdRequest, is_connected)
        # if myObject.is_cmdRequest and is_connected == 1:
        #     # print("emit to server")
        #     myObject.is_cmdRequest = 0
        #     # my_socketIO.emit("Server_cmdRequest", json.dumps(myObject.NN_cmdSub, indent = 4))
        #     my_socketIO.emit("Server_query_cmd_request", json.dumps(myObject.NN_cmdSub))

        if myObject.is_infoRequest and is_connected == 1:
            # print("emit to server")
            myObject.is_infoRequest = 0
            # my_socketIO.emit("Server_cmdRequest", json.dumps(myObject.NN_cmdSub, indent = 4))
            my_socketIO.emit("Server_query_info_request", json.dumps(myObject.NN_infoRequest_sio))

        # -s
        # if is_connected == 0:
        #     delta_t = (time.time() - time_save)%60
        #     if delta_t > 0.6:
        #         time_save = time.time()
        #         try:
        #             my_socketIO.connect('http://' + server_IP +':' + str(server_port))
        #         except Exception:
                    # pass

        myObject.rate.sleep()
                
    my_socketIO.disconnect()
	
if __name__ == '__main__':
    main()