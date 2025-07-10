#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from sti_msgs.msg import  ManageLaunch
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Float64
import math 
from math import degrees, radians, fabs
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Quangduong:
    def __init__(self):
        rospy.init_node('test_runOdom', anonymous=True)
        self.rate = rospy.Rate(50)
        rospy.Subscriber('/odom', Odometry, self.call_odom, queue_size = 20)
        self.data_odom = Odometry()

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.getPose, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.theta_rb_ht = 0.0

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.angle_odom = 0.0

        self.x_nav = 0.0
        self.y_nav = 0.0
        self.angle_nav = 0.0

        self.angle = 0.0

        self.process = 0

        #fnShutDown
        rospy.on_shutdown(self.fnShutDown)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_vel.publish(Twist()) 
    
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        # print "fnCalcDistPoints"
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def call_odom(self, data):
        # print(data)
        self.data_odom = data
        quata = (data.pose.pose.orientation.x,\
                data.pose.pose.orientation.y,\
                data.pose.pose.orientation.z,\
                data.pose.pose.orientation.w )
        
        euler = euler_from_quaternion(quata)
        self.angle = euler[2]

        if self.process == 0:
            self.process = 1

    def getPose(self, data):
        self.is_pose_robot = True
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb_ht = euler[2]

        if self.process == 1:
            self.process = 2

    def raa(self): 
        angle_need = 180.
        s_move_need = 0.1
        vel_linear = 0.1
        vel_rotation = 0.15

        while not rospy.is_shutdown():
            if self.process == 2:
                self.x_odom = self.data_odom.pose.pose.position.x
                self.y_odom = self.data_odom.pose.pose.position.y
                self.angle_odom = self.angle

                self.x_nav = self.poseRbMa.position.x
                self.y_nav = self.poseRbMa.position.y
                self.angle_nav = self.theta_rb_ht

                self.process = 3

            elif self.process == 3:
                x_now = self.data_odom.pose.pose.position.x
                y_now = self.data_odom.pose.pose.position.y

                dis = self.fnCalcDistPoints(self.x_odom, x_now, self.y_odom, y_now)
                angle = degrees(self.angle) - degrees(self.angle_odom)

                dis_nav = self.fnCalcDistPoints(self.x_nav, self.poseRbMa.position.x, self.y_nav, self.poseRbMa.position.y)
                angle_nav = fabs(degrees(self.angle_nav) - degrees(self.theta_rb_ht))

                print("dis = %s, angle = %s" %(dis, angle))

                if dis > s_move_need:
                    print("dung lai, di chuyen - dis nav: ", dis_nav)
                    self.pub_vel.publish(Twist())
                    self.process = 4

                else:
                    twist = Twist()
                    twist.linear.x = vel_linear
                    self.pub_vel.publish(twist)

                # if fabs(angle) > angle_need:
                #     print("dung lai, quay - angle nav: ", angle_nav)
                #     self.pub_vel.publish(Twist())
                #     self.process = 4

                # else:
                #     twist = Twist()
                #     twist.angular.z = vel_rotation
                #     self.pub_vel.publish(twist)


            elif self.process == 4:
                self.pub_vel.publish(Twist())
            
            self.rate.sleep()

def main():
    
    try:
        m = Quangduong()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()