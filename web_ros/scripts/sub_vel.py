#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

# Tỉ lệ điều chỉnh cho vận tốc
LINEAR_SCALE = 0.5
ANGULAR_SCALE = 0.5

class Pub_cmd:
    def __init__(self):
        rospy.init_node("test_joystick", anonymous = True)
        

        self.x = 0
        self.y = 0
        self.rate_pubVel = 30
        self.time_tr = rospy.get_time()       
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.Subscriber("/web_info", Float32MultiArray,self.joyCallback)
        self.joy = Float32MultiArray().data
    def joyCallback(self, msg):
        self.joy = msg.data

    def pub_cmdVel(self, twist, rate):
        if rospy.get_time() - self.time_tr > float(1/rate):
            self.time_tr = rospy.get_time()
            self.pub.publish(twist)
        
    def run(self):
        rate = rospy.Rate(self.rate_pubVel)
        
        while not rospy.is_shutdown():
            print(self.joy)
            # Kiểm tra nếu self.joy có đủ phần tử trước khi truy cập
            if len(self.joy) >= 5:  # Đảm bảo rằng self.joy có ít nhất 5 phần tử
                x = self.joy[2]  # Truy cập đến dữ liệu trong self.joy
                y = self.joy[3]
                w = self.joy[4]
                
                print("x:", x)
                print("y:", y)

                # Tính toán vận tốc tuyến tính và góc
                a = -y / w
                b = -x / w

                vel = Twist()
                vel.linear.x = LINEAR_SCALE * a
                vel.angular.z = ANGULAR_SCALE * b
                
                if abs(vel.linear.x) > 0.0001 or abs(vel.angular.z) > 0.0001:
                	
                	self.pub_cmdVel(vel, self.rate_pubVel)
                else:
                	vel_0 = Twist()
                	self.pub_cmdVel(vel_0,self.rate_pubVel)
            else:
                rospy.logwarn("self.joy không đủ phần tử: {}".format(len(self.joy)))  # In cảnh báo nếu thiếu phần tử

            rate.sleep()


if __name__ == '__main__':
    try:
        a = Pub_cmd()
        a.run()
    except rospy.ROSInterruptException:
        pass

