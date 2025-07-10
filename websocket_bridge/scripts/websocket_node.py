#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
import websocket

def talker():
    # Khởi tạo node ROS
    rospy.init_node('talker', anonymous=True)
    
    # Tạo Publisher cho topic '/my_topic'
    pub = rospy.Publisher('/my_topic', String, queue_size=10)

    # Khởi động WebSocket connection
    ws = websocket.create_connection("ws://localhost:9090")  # Đảm bảo rosbridge đã chạy

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        # Tạo message
        message = "hihuhuhu!"
        rospy.loginfo(message)
        
        # Xuất bản message lên ROS topic
        pub.publish(message)

        # Gửi message qua WebSocket
        json_message = {
            'op': 'publish',
            'topic': '/my_topic',
            'msg': {
                'data': message
            }
        }
        ws.send(json.dumps(json_message))

        rate.sleep()

    ws.close()  # Đóng kết nối WebSocket

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
