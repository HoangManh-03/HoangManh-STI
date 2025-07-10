#!/usr/bin/env python3

import rospy
from sti_msgs.msg import Port_status_v2  # Thay 'your_package' bằng tên gói chứa Port_status_v2

class PortSubscriber:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('port_status_subscriber', anonymous=True)
        
        # Đăng ký subscriber với topic '/port_status_v2'
        rospy.Subscriber('/port_status_v2', Port_status_v2, self.callBack_checkPort)
        
        # Giữ cho node hoạt động
        rospy.spin()

    def callBack_checkPort(self, msg):
        # Hàm callback nhận tin nhắn từ topic '/port_status_v2'
        rospy.loginfo("Received Port Status:")
        rospy.loginfo("Port ID: %s", msg.rtc)

        # Bạn có thể thêm xử lý logic khác ở đây

if __name__ == '__main__':
    try:
        # Tạo một instance của PortSubscriber và chạy
        PortSubscriber()
    except rospy.ROSInterruptException:
        pass

