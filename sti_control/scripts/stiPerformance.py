#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import math
import rospy
from datetime import datetime
# ip
import os
import re  
import subprocess
import json

from message_pkg.msg import *
from sti_msgs.msg import *
from std_msgs.msg import Float64

class InfoError():
    def __init__(self):
        self.code_error = 0
        self.total_time = 0

        self.save_oldTime = time.time()

    def updatetime(self, time_now):
        self.total_time += time_now - self.save_oldTime
        self.save_oldTime = time.time()

    def reset_time(self):
        self.save_oldTime = time.time()


class InfoByDay():
    def __init__(self,):
        self.day = ''
        self.time_running = 0
        self.time_wait = 0
        self.time_error = 0
        # -- 
        self.ls_infoError = []


    def update_error(self, ls_code_error, time_stamp):
        if len(self.ls_infoError) == 0 and len(ls_code_error) != 0:
            for code_error in ls_code_error:
                self.ls_infoError.append(InfoError(code_error))

            return
        
        # -- update hoặc free code cũ
        for infoError in self.ls_infoError:
            for code_error in ls_code_error:
                if code_error == infoError.code_error:
                    infoError.updatetime(time_stamp)

                    break

            infoError.reset_time()

        # -- update
        for code_error in ls_code_error:
            for infoError in self.ls_infoError:
                if code_error == infoError.code_error:
                    break

            self.ls_infoError.append(InfoError(code_error))

#--------------------------------------------------------------------------------- ROS
class Performance():
    def __init__(self):
        rospy.init_node('stiPerformance', anonymous=False)
        self.rate = rospy.Rate(50)
        # SUB - PUB
        # - 
        rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.NN_cmdCallback)
        self.NN_cmdRequest = NN_cmdRequest()
        self.pre_NN_cmdRequest = NN_cmdRequest()
        self.is_request_move = False
        self.timestamp_getNNcmdReq = time.time()
        # - 
        rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.NN_infoCallback)
        self.NN_infoRespond = NN_infoRespond()
        self.pre_NN_infoRespond = NN_infoRespond()
        self.is_NN_infoRespond = False
        self.timestamp_getNNinfoRes = time.time()
        # - 
        # -- move request
        rospy.Subscriber("/request_move", Move_request, self.callback_moveRequest)
        self.req_move = Move_request()
        self.pre_req_move = Move_request()
        self.is_request_move = False
        self.timestamp_getReqMove = time.time()
        # -
        # -- move respond
        rospy.Subscriber("/respond_move", Move_respond, self.callback_moveRespond)
        self.res_move = Move_respond()
        self.pre_req_move = Move_respond()
        self.is_respond_move = False
        self.timestamp_getResMove = time.time()

        # -- đọc file json
        self.link_dataPerformance = '/home/stivietnam/catkin_ws/debug/performance_data.json'
        self.ls_performanceByDay = []
        self.date_update = ''
        try:
            with open(self.link_dataPerformance, 'r') as file:
                data = json.load(file)
                self.date_update = data["date_update"]
                for dt in data["data_byDay"]:
                    iday = InfoByDay()
                    iday.day = dt["date"]
                    iday.time_wait = dt["time_wait"]
                    iday.time_running = dt["time_running"]
                    iday.time_error = dt["time_error"]

                    for error in dt["info_error"]:
                        info_error = InfoError()
                        info_error.code_error = error["code_error"]
                        info_error.total_time = error["total_time"]

                        iday.info_error.append(info_error)

                    self.ls_performanceByDay.append(iday)

        except Exception as e:
            print("Read json file fail")


        self.index_update = -1

    def NN_cmdCallback(self, data):
        self.NN_cmdRequest = data
        self.is_request_move = True
        self.timestamp_getNNcmdReq = time.time()

    def NN_infoCallback(self, data):
        self.NN_infoRespond = data
        self.is_NN_infoRespond = True
        self.timestamp_getNNinfoRes = time.time()

    def callback_moveRequest(self, data):
        self.req_move = data
        self.is_request_move = True
        self.timestamp_getReqMove = time.time()

    def callback_moveRespond(self, data):
        self.res_move = data
        self.is_respond_move = True
        self.timestamp_getResMove = time.time()

    def run(self):
        while not rospy.is_shutdown():
            # khởi tạo ngày mới hoặc update ngày cũ
            today = datetime.today().date()
            
            if str(today) != self.date_update:
                self.date_update = str(today)
                iday = InfoByDay()
                iday.day = self.date_update
                self.ls_performanceByDay.append(iday) 

                # -- tim lai index update
                self.index_update = -1

            # -- tim kiem update day trong list
            if self.index_update == -1:
                for index, info_day in enumerate(self.ls_performanceByDay):
                    if self.date_update == info_day.day:
                        self.index_update = index
                        break

            # -- 

            # -- list error
            if self.is_NN_infoRespond:
                self.is_NN_infoRespond = False

                list_error = self.NN_infoRespond.listError
                self.ls_performanceByDay[index].update_error(list_error)

                for i in self.ls_performanceByDay:
                    print(i)


            self.rate.sleep()



def main():
	class_1 = Performance()		
	class_1.run()

if __name__ == '__main__':
	main()



