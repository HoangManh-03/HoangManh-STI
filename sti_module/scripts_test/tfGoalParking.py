#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import signal
import tf
import time
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import sti_msgs.msg
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, fabs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading

class BroadcasterParking(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        
        self.rate = rospy.Rate(30)

        # Tf
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()


        rospy.Subscriber('/parking_request', Parking_request, self.request_callback)
        self.req_parking = Parking_request() 
        self.is_request_parking = False
        
        rospy.Subscriber('/robotPose_nav', PoseStamped, self.cbPose, queue_size = 20)
        self.is_pose_robot = False
        
        self.pubPoseParking = rospy.Publisher("/poseParking", PoseStamped, queue_size= 20)

        self.process = 0
        self.pubTransform = False
        self.time_waitTransfrom = time.time()
        self.is_transform = False

    def cbPose(self, data):
        self.is_pose_robot = True
        if self.is_transform == True:
            self.transformPoseNAV('frame_target', 'frame_robot')

    def sendTransform(self, frame_world, frame_id_pointTarget, point_target):
        self.time_startTransform = rospy.Time.now()
        self.tf_broadcaster.sendTransform((point_target.position.x, point_target.position.y, -1.0),
                                        (0.0, 0.0, point_target.orientation.z, point_target.orientation.w),
                                        self.time_startTransform,
                                        frame_id_pointTarget,
                                        frame_world)
        
    def transformPoseNAV(self, frame_id_pointTarget, frame_agvNAV):
        msgPoseParking = PoseStamped()
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(frame_id_pointTarget, frame_agvNAV, now, rospy.Duration(10))
            pos, orien = self.tf_listener.lookupTransform(frame_id_pointTarget, frame_agvNAV, now)

            msgPoseParking.header.stamp = now
            msgPoseParking.header.frame_id = frame_id_pointTarget

            msgPoseParking.pose.position.x = pos[0]
            msgPoseParking.pose.position.y = pos[1]
            msgPoseParking.pose.position.z = pos[2]
            msgPoseParking.pose.orientation.x = orien[0]
            msgPoseParking.pose.orientation.y = orien[1]
            msgPoseParking.pose.orientation.z = orien[2]
            msgPoseParking.pose.orientation.w = orien[3]

            self.pubPoseParking.publish(msgPoseParking)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            pass
        
    def request_callback(self, data):
        self.req_parking = data
        self.is_request_parking = True

    def run(self):
        while not self.shutdown_flag.is_set(): 
            self.rate.sleep()
 
class ServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass
 
def service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ServiceExit
 
def main():

    rospy.init_node('broadcaster_parking', anonymous=False)
    print("initial node!")

    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:

        br = BroadcasterParking(1)
        br.start()

        # Keep the main thread running, otherwise signals are ignored.
        while not rospy.is_shutdown():
            if br.is_request_parking:
                if br.req_parking.modeRun == 1 or br.req_parking.modeRun == 2:
                    br.sendTransform('frame_map_nav350', 'frame_target', br.req_parking.poseTarget) # -- edit 03/03/2022: frame_map_nav350 frame_global_map
                    br.is_transform = True

                elif br.req_parking.modeRun == 0:
                    br.is_transform = False

            time.sleep(0.01)
 
    except ServiceExit:
        
        br.shutdown_flag.set()
        # Wait for the threads to close...
        br.join()

 
    print('Exiting main program')

if __name__ == '__main__':
    main()