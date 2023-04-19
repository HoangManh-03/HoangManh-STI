#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Float64
from math import sqrt 

class Quangduong:
    def __init__(self):
        rospy.init_node('testVel', anonymous=True)
        self.time_tr = rospy.get_time();
        self.rate = rospy.Rate(20)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def pub_cmdVel(self, twist , rate):

        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass

    def phuongTrinhGiaToc(self,denlta_time, time_s, v_s, v_f):
        v_re = 0.0
        denlta_time_now = rospy.Time.now().to_sec() - time_s
        a = (v_f-v_s)/denlta_time
        if denlta_time_now <= denlta_time :
            v_re = v_s + a*denlta_time_now
            # if a > 0:
            #     if v_re >= v_f:
            #         v_re = v_f
            # else:
            #     if v_re <= v_f:
            #         v_re = v_f

        else:
            v_re = v_f

        return v_re


    def raa(self): 
        saveTime = rospy.Time.now().to_sec()
        tw = Twist()
        step = 1
        while not rospy.is_shutdown():
            if step == 1:
                vel_x = self.phuongTrinhGiaToc(3.0, saveTime, 0.0, 0.7)
                tw.linear.x = vel_x
                self.pub_cmdVel(tw, 15)
                if vel_x == 0.7:
                    step = 2
                    saveTime = rospy.Time.now().to_sec()
            
            elif step == 2:
                if rospy.Time.now().to_sec() - saveTime < 5:
                    tw.linear.x = 0.7
                    self.pub_cmdVel(tw, 15)
                else:
                    step = 3
                    saveTime = rospy.Time.now().to_sec()

            elif step == 3:
                vel_x = self.phuongTrinhGiaToc(3.0, saveTime, 0.7, 0.0)
                tw.linear.x = vel_x
                self.pub_cmdVel(tw, 15)

            self.rate.sleep()

def main():
    
    try:
        m = Quangduong()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()