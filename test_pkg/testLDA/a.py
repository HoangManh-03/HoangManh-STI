#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sti_msgs.msg import NN_cmdRequest

def publish_nn_cmd_request():
    # Khởi tạo nút ROS
    rospy.init_node('stiControl', anonymous=False)
    self.rate = rospy.Rate(30)

    # Tạo một publisher cho topic '/NN_cmdRequest' với message type là NN_cmdRequest
    pub = rospy.Publisher('/NN_cmdRequest', NN_cmdRequest, queue_size=10)

    # Tạo một object NN_cmdRequest và thiết lập các giá trị
    nn_cmd_request = NN_cmdRequest()
    nn_cmd_request.process = 0.0
    nn_cmd_request.target_id = 0
    nn_cmd_request.target_x = 10.00
    nn_cmd_request.target_y = 4.00
    nn_cmd_request.target_z = 3.1416
    nn_cmd_request.tag = 0
    nn_cmd_request.offset = 1.2
    nn_cmd_request.list_id = [1, 2, 3, 0, 0]
    nn_cmd_request.list_x = [11.40, 9.57, 10.00, 0, 0]
    nn_cmd_request.list_y = [-1.81, -1.81, 4.00, 0, 0]
    nn_cmd_request.list_speed = [0, 0, 0, 0, 0]
    nn_cmd_request.before_mission = 0
    nn_cmd_request.after_mission = 1
    nn_cmd_request.id_command = 0
    nn_cmd_request.command = ''

    # Publish message
    pub.publish(nn_cmd_request)

    # rospy.loginfo("NN_cmdRequest message published:\n{}".format(nn_cmd_request))

if __name__ == '__main__':
    try:
        publish_nn_cmd_request()
    except rospy.ROSInterruptException:
        pass
