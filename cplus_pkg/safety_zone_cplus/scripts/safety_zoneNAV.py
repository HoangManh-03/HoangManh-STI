#!/usr/bin/env python3
# Author : PhucHoang
# EDIT for NAV : 27/10/2021

from std_msgs.msg import Empty , ColorRGBA, Int8
from geometry_msgs.msg import  Pose , Point
from sensor_msgs.msg import PointCloud2, LaserScan
from decimal import Decimal
from sti_msgs.msg import *
import rospy
import time

class safety_zone():
    def __init__(self):
        
        rospy.init_node('safety_zone', anonymous = True)
        self.rate = rospy.Rate(50)

        # get param
        self.dis_max = rospy.get_param('~dis_max', 0.2)
        self.dis_min = rospy.get_param('~dis_min', 0.05)

        # topic pub-sub
        rospy.Subscriber("/scan", LaserScan, self.call_sub)
        self.zone_lidarNAV = rospy.Publisher('safety_NAV', Int8, queue_size=10) 
        self.dPubZone = Int8()

        # variable
        self.data_scan = LaserScan()
        self.is_scanNav = False


    def call_sub(self,data):
        self.data_scan = data
        self.is_scanNav = True
        
            
    def raa(self):
        while not rospy.is_shutdown():
            if self.is_scanNav == True:
                self.is_scanNav = False
                numberPointinCircle = 0
                # print (len(self.data_scan.ranges))
                # for i in range(0,len(self.data_scan.ranges),1):
                try:
                    for i in range(0,len(self.data_scan.ranges),1):
                    # for i in range(40,1400,1):
                        # if (self.data_scan.ranges[i] <= self.dis_max):
                        #     print ("ANG: " + str(i) + " | " + str(self.data_scan.ranges[i]))

                        if (self.data_scan.ranges[i] >= self.dis_min and self.data_scan.ranges[i] <= self.dis_max):
                            numberPointinCircle = numberPointinCircle + 1
                except:
                    numberPointinCircle = 500
                    print ("Something wrong!!!")

                # print(numberPointinCircle)
                if numberPointinCircle >= 15:
                    self.dPubZone.data = 1

                else:
                    self.dPubZone.data = 0

                self.zone_lidarNAV.publish(self.dPubZone) 
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
