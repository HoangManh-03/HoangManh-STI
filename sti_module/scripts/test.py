#!/usr/bin/python3

import threading
import time
import rospy
from std_msgs.msg import String, Bool, Int8

import sys
import struct
import string
import roslib
import serial
import signal

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist, TwistWithCovarianceStamped

from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos

import os

poseRbMa = Pose()
Max_x = 0.0
Min_x = 10000.0
Max_y = 0.0
Min_y = 10000.0
ss_x = 0.0
ss_y = 0.0
is_data_robot = False
        
def getPose(data):
    global poseRbMa, is_data_robot
    is_data_robot = True
    poseRbMa = data.pose
    
# def fnShutDown():
#     rospy.loginfo("Shutting down")

def main():
    global poseRbMa, Max_x, Min_x, Max_y, Min_y, ss_x, ss_y, is_data_robot
    rospy.init_node('test_ss', anonymous=True)
    rate = rospy.Rate(30)
    rospy.Subscriber('/robotPose_nav', PoseStamped, getPose, queue_size = 20)
    # rospy.on_shutdown(fnShutDown)
    
    while not rospy.is_shutdown():
        if is_data_robot == True:
            if Max_x <= poseRbMa.position.x:
                Max_x = poseRbMa.position.x
            if Min_x >= poseRbMa.position.x:
                Min_x = poseRbMa.position.x
            if Max_y <= poseRbMa.position.y:
                Max_y = poseRbMa.position.y
            if Min_y >= poseRbMa.position.y:
                Min_y = poseRbMa.position.y
            ss_x = Max_x - Min_x
            ss_y = Max_y - Min_y
            print("max_x= %s, min_x= %s , ss_x= %s \n" %(Max_x, Min_x, ss_x))
            print("max_y= %s, min_y= %s , ss_y= %s \n" %(Max_y, Min_y, ss_y))
            print("-----------------------------------------------------------")
            is_data_robot = False
            
        rate.sleep()
        
if __name__ == '__main__':
    main()