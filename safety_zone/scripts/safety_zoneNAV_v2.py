#!/usr/bin/env python3
# Author : PhucHoang
# EDIT for NAV : 27/10/2021
# Update cho AGV SHIV34 : 19/03/2024 - HOANG AI
    # Them message SafetyZoneNav


from std_msgs.msg import Empty , ColorRGBA, Int8
from geometry_msgs.msg import  Pose , Point
from sensor_msgs.msg import PointCloud2, LaserScan
from decimal import Decimal
from sti_msgs.msg import *
import rospy
import time

from math import atan2, sin, cos, sqrt, fabs
from math import pi as PI

class safety_zone():
    def __init__(self):
        
        rospy.init_node('safety_zone', anonymous = True)
        self.rate = rospy.Rate(50)

        # get param
        self.dis_circle_max = rospy.get_param('~dis_circle_max', 0.2)
        self.dis_circle_min = rospy.get_param('~dis_circle_min', 0.05)

        self.dis_ahead_max = rospy.get_param('~dis_ahead_max', 0.2)
        self.dis_ahead_min = rospy.get_param('~dis_ahead_min', 0.05)

        self.dis_behind_max = rospy.get_param('~dis_behind_max', 0.2)
        self.dis_behind_min = rospy.get_param('~dis_behind_min', 0.05)

        self.dis_width = rospy.get_param('~dis_width', 0.05)

        # topic pub-sub
        rospy.Subscriber("/scan", LaserScan, self.call_sub)
        self.zone_lidarNAV = rospy.Publisher('safety_NAV_test', SafetyZoneNav, queue_size=10) 
        self.dPubZone = SafetyZoneNav()

        # variable
        self.data_scan = LaserScan()
        self.is_scanNav = False


    def call_sub(self,data):
        self.data_scan = data
        self.is_scanNav = True

    def dataProcessing(self):
        numberPointInAreaCircle = 0
        numberPointInAreaAhead = 0
        numberPointInAreaBehind = 0

        for i in range(0, len(self.data_scan.ranges), 1):
            x_point = 0.
            y_point = 0.
            angle_cur = self.data_scan.angle_min + i*self.data_scan.angle_increment

            if fabs(angle_cur) > PI/2.0:
                if angle_cur > 0:
                    x_point = -cos(fabs(PI - angle_cur))*self.data_scan.ranges[i]
                    y_point = -sin(fabs(PI - angle_cur))*self.data_scan.ranges[i]
                else:
                    x_point = -cos(fabs(PI - angle_cur))*self.data_scan.ranges[i]
                    y_point = sin(fabs(PI - angle_cur))*self.data_scan.ranges[i]
  
            # check vung truoc
            else:
                if angle_cur > 0:
                    x_point = cos(fabs(angle_cur))*self.data_scan.ranges[i]
                    y_point = -sin(fabs(angle_cur))*self.data_scan.ranges[i]
                else:
                    x_point = cos(fabs(angle_cur))*self.data_scan.ranges[i]
                    y_point = sin(fabs(angle_cur))*self.data_scan.ranges[i]

            if self.data_scan.ranges[i] >= self.dis_circle_min and self.data_scan.ranges[i] <= self.dis_circle_max:
                numberPointInAreaCircle = numberPointInAreaCircle + 1 

            if x_point >= self.dis_ahead_min and x_point <= self.dis_ahead_max and fabs(y_point) <= self.dis_width/2.:
                numberPointInAreaAhead = numberPointInAreaAhead + 1

            if x_point >= -1*self.dis_behind_max and x_point <= -1*self.dis_behind_min and fabs(y_point) <= self.dis_width/2.:
                numberPointInAreaBehind = numberPointInAreaBehind + 1

        if numberPointInAreaCircle >= 30:
            self.dPubZone.area_circle = 1
        else:
            self.dPubZone.area_circle = 0

        if numberPointInAreaAhead >= 20:
            self.dPubZone.area_ahead = 1
        else:
            self.dPubZone.area_ahead = 0

        if numberPointInAreaBehind >= 20:
            self.dPubZone.area_behind = 1
        else:
            self.dPubZone.area_behind = 0
            
    def raa(self):
        while not rospy.is_shutdown():
            if self.is_scanNav == True:
                self.is_scanNav = False
                try:
                    self.dataProcessing()
                    self.zone_lidarNAV.publish(self.dPubZone) 
                except:
                    print ("Something wrong!!!")

            self.rate.sleep()
        # print('Thread #%s stopped' % self.threadID)

def main():
    
    try:
        m = safety_zone()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()
