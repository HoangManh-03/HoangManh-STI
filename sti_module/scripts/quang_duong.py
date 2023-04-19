#!/usr/bin/env python3

from sti_msgs.msg import  ManageLaunch
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64
import math 


class Quangduong:
    def __init__(self):
        rospy.init_node('quangduong', anonymous=True)
        self.rate = rospy.Rate(1)
        rospy.Subscriber('/robotPose_nav', PoseStamped, self.call_pose, queue_size = 20)
        self.s_pub = rospy.Publisher('/quang_duong', Float64, queue_size=10)
        self.s = 0.
        self.pose_ht = Pose()
        self.pose_tr = Pose()
        self.step = 0
    
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        # print "fnCalcDistPoints"
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def call_pose(self, data): 
        if self.step == 0 : 
            self.pose_tr = self.pose_ht = data.pose
            self.step = 1

        if self.step == 1 :
            self.pose_ht = data.pose
            self.s = self.s + self.fnCalcDistPoints(self.pose_ht.position.x, \
                                                    self.pose_tr.position.x, \
                                                    self.pose_ht.position.y,\
                                                    self.pose_tr.position.y)
            self.pose_tr = self.pose_ht


    def raa(self): 
        while not rospy.is_shutdown():
            self.s_pub.publish(self.s)
            self.rate.sleep()

def main():
    
    try:
        m = Quangduong()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()