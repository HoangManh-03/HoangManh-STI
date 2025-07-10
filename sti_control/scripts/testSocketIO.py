#!/usr/bin/env python3

import socketio
import roslib

import json 

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

from geometry_msgs.msg import Twist

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


sio = socketio.Client()
vel_x = 20.
vel_y = 20.
is_connected = 0
mac_agv = ''
ip_agv = ''

def reset():
    global is_connected, vel_x, vel_z
    vel_x = 20.
    vel_z = 20.
    is_connected = 0

@sio.on('Server-request-AGV')
def on_message(data):
    print('I received a message!')
    print(data)

@sio.event
def connect():
    global is_connected
    print("I'm connected!")
    is_connected = 1

@sio.event
def connect_error(data):
    print("The connection failed!")
    reset()

@sio.event 
def disconnect():
    print("I'm disconnected!")
    reset()

def get_macAuto(name_card):
    try:
        mac = re.search(re.compile(r'(?<=link/ether )(.*)(?=\ b)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
        return mac
    except Exception:
        return "-1"
    
def get_ipAuto(name_card): # name_card : str()
    try:
        address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
        return address
    except Exception:
        return "-1"

def callback(data):
    global sio, mac_agv, ip_agv, is_connected, vel_x, vel_z
    # quaternion1 = (data.pose.orientation.x, data.pose.orientation.y,\
    #                 data.pose.orientation.z, data.pose.orientation.w)
    # euler = tf.transformations.euler_from_quaternion(quaternion1)
    # x = round(data.pose.position.x, 3)
    # y = round(data.pose.position.y, 3)
    # z = round(euler[2], 3)
    # data = {"id": 1, "name": "lagv", "enable": 0, "status": "diem1", "x": x, "y": y, "r": z}

    if (vel_x != data.linear.x or vel_z != data.angular.z ) and is_connected:
        vel_x = data.linear.x
        vel_z = data.linear.x

        data = {"ip": ip_agv, "linear": round(vel_x,3), "angular" : round(vel_z,3)}
        print(data)
        json_object = json.dumps(data, indent = 4)
        sio.emit("Update-Velocity", json_object)
    
def listener():
    global sio, mac_agv, ip_agv
    mac_agv = get_macAuto('wlo2')
    ip_agv = get_ipAuto('wlo2')
    sio.connect('http://192.168.1.92:3000')
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('testSocket', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()