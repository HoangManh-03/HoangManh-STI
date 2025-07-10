#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author : PhucHoang 16/05/2024

import roslib
import sys
import signal
import tf
import time
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sti_msgs.msg import APathParking, PointOfPath, Zone_lidar_2head, HC_info
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, tan, fabs, sin, cos, radians, degrees
from math import pi as PI
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
from visualization_msgs.msg import Marker

class Point():
  def __init__(self, _x=0, _y=0):
    self.x = _x
    self.y = _y

class Paraboll():
    def __init__(self, _pointOne=Point(), _pointSecond=Point()):
        # khoi tao 2 bien point
        self.pointOne = _pointOne
        self.pointSecond = _pointSecond

        b = np.array([self.pointOne.y, self.pointSecond.y, 0])
        b = b[:, np.newaxis]
        A = np.array([[pow(self.pointOne.x, 2), self.pointOne.x, 1], [pow(self.pointSecond.x, 2), self.pointSecond.x, 1], [2*self.pointSecond.x, 1, 0]])
        C = np.dot(np.linalg.inv(A), b)
        print(C)
        self.a = C[0,0]
        self.b = C[1,0]
        self.c = C[2,0]

    def calc(self, _x):
        y = self.a*pow(_x, 2) + self.b*_x + self.c
        return y
    
    def tangent(self, _point):
        a = 2*self.a*_point.x + self.b
        if _point.y > 0:
            return -atan(a)
        else:
            return atan(a)
        
    def slip(self, _step):
        listPoint = np.empty((0,2), dtype=float)
        rx = np.arange(self.pointOne.x, self.pointSecond.x, -_step, float)
        for i in rx:
            listPoint = np.append(listPoint, np.array([[i, self.calc(i)]]), axis=0)
        return listPoint
        
class QuadraticBezierCurves():
    def __init__(self, _pointOne=Point(), _pointSecond=Point(), _midpoint=Point(), _typeDefine=0,_angleOne=0., _angleSecond=0., _numberPts=0 ):
        self.pointOne = _pointOne
        self.pointSecond = _pointSecond
        self.midPoint = _midpoint
        if _typeDefine == 1:
            a1, b1, c1 = self.findStraightLineByAngleAndPoint(self.pointOne, _angleOne)
            a2, b2, c2 = self.findStraightLineByAngleAndPoint(self.pointSecond, _angleSecond)
            if b1 == 0:
                x = -c1/a1
                y = (-c2-a2*x)/b2

            else:
                x = ((b2*c1)/b1 - c2)/(a2 - (b2*a1)/b1)
                y = (-c1-a1*x)/b1

            # x = (-c1+b2)/(a1-a2)
            # y = a1*x + c1
            self.midPoint = Point(x,y)
            print(self.midPoint.x ,self.midPoint.y)
        # self.numberPts = _numberPts

        self.numberPts = self.findNumberPts()

        self.t = np.array([i*1/self.numberPts for i in range(0,self.numberPts+1)])

        print("Make a Quadratic Bezier Curves ")

    def findStraightLineByAngleAndPoint(self, _point, _angle):
        if fabs(_angle) == PI/2.:
            return 1, 0, -_point.x
        else:
            k = tan(_angle)
            return k, -1., (-1)*k*_point.x + _point.y


    def findNumberPts(self):
        dis1 = self.calculate_distance(self.pointOne.x, self.pointOne.y, self.midPoint.x, self.midPoint.y)
        dis2 = self.calculate_distance(self.midPoint.x, self.midPoint.y, self.pointSecond.x, self.pointSecond.y)
        dis3 = self.calculate_distance(self.pointOne.x, self.pointOne.y, self.pointSecond.x, self.pointSecond.y)

        return int(max(dis1, dis2, dis3)/0.01)


    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

    def slip(self):
        listPoint = np.empty((0,2), dtype=float)
        for i in self.t:
            _x = (1-i)*((1-i)*self.pointOne.x + i*self.midPoint.x) + i*((1-i)*self.midPoint.x + i*self.pointSecond.x)
            _y = (1-i)*((1-i)*self.pointOne.y + i*self.midPoint.y) + i*((1-i)*self.midPoint.y + i*self.pointSecond.y)

            listPoint = np.append(listPoint, np.array([[_x, _y]]), axis=0)

        return listPoint
    
class PointOfPath():
    def __init__(self):
        self.pose = Pose()
        self.velocity = 0.
    

class InfoPathFollowing():
    def __init__(self):
        self.poseStart = Pose()
        self.poseIntermediary = Pose()
        self.info = []

class InfoPointFollowing():
    def __init__(self):
        self.indexInListpath = 0.
        self.velocity = 0.
        self.radius = 0.
        self.X = 0.
        self.Y = 0.

class ParkingAGV():
    def __init__(self):
        rospy.init_node('parking_agv', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(30)

        self.min_angularVelocity = rospy.get_param('~min_angularVelocity', 0.01)
        self.max_angularVelocity = rospy.get_param('~min_angularVelocity', 1.)

        self.max_linearVelocity = rospy.get_param('~max_linearVelocity', 0.8)
        self.min_linearVelocity = rospy.get_param('~min_linearVelocity', 0.02)

        self.max_lookahead = rospy.get_param('~min_lookahead', 1.49)
        self.min_lookahead = rospy.get_param('~min_lookahead', 0.45)
        self.lookahead_ratio = rospy.get_param('~lookahead_ratio', 8.0)

        self.distanceAlign = rospy.get_param('~distanceAlign', 0.2) 
        self.distanceGoStraight = rospy.get_param('~distanceGoStraight', 0.8)

        self.stepPoint = 0.01

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.callback_poseRobot, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        rospy.Subscriber('/parking_request', Parking_request, self.parkingRequest_callback, queue_size = 20)
        self.req_parking = Parking_request() 
        self.is_request_parking = 0

        rospy.Subscriber("/HC_info", HC_info, self.zone_callback)
        self.zone_lidar = HC_info()
        self.is_check_zone = False	

        # Pub topic
        self.pub_ParkingRespond = rospy.Publisher('/parking_respond', Parking_respond, queue_size=20)
        # self.data_PubRespond = Parking_respond()
        self.timePubRespond = rospy.get_time()
        self.rate_pubRespond = 30

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.time_tr = rospy.get_time()
        self.rate_pubVel = 15

        self.pubMarker = rospy.Publisher('/visualization_markerPoint', Marker, queue_size=10)
        rospy.on_shutdown(self.fnShutDown)

        self.process = 0

        self.min_vel = 0.03
        self.min_velFinish = 0.04
        self.min_rol = 0.1
        self.min_rolFinish = 0.15

        self.max_vel = 0.07
        self.max_rol = 0.3

        self.xFollow = 0.
        self.yFollow = 0.
        self.velFollow = 0.

        self.distDeceleration = 0.3
        self.disgetLocation = 0.15

        self.curr_velocity = 0.

        self.flagFollowPointFinish = False

        self.listVel = [0.02, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]
        self.numTable = len(self.listVel)
        # self.listLookAhead = [0.15, 0.22, 0.3, 0.4, 0.55, 0.65, 0.7, 0.75, 0.8, 0.9, 1.05, 1.28, 1.34, 1.39, 1.44, 1.49]
        self.listLookAhead = [0.45, 0.48, 0.51, 0.54, 0.55, 0.65, 0.7, 0.75, 0.8, 0.9, 1.05, 1.28, 1.34, 1.39, 1.44, 1.49]

        self.infoPathFollow = InfoPathFollowing()
        self.infoPointFollow = InfoPointFollowing()
        self.saveTimeVel = rospy.get_time()
        self.statusVel = 0
        self.velSt = 0.
        self.warn_agv = 0

        self.ss_x = 0.0
        self.ss_y = 0.0
        self.ss_a = 0.0

        self.poseRobot_convert = Pose()
        self.angleRobot_convert = 0.
        self.matrix_origin_to_target = []

    # -- function callback
    def callback_poseRobot(self, data):
        self.poseStampedAGV = data
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_robotNow = euler[2]

        self.is_pose_robot = True

    def parkingRequest_callback(self, data):
        self.req_parking = data
        self.is_request_parking = 1

    def zone_callback(self, data):
        self.zone_lidar = data
        self.is_check_zone = True
    

    # function pub
    def pub_cmdVel(self, twist , rate):
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass

    def pub_Stop(self):
        for i in range(0,3,1):
            self.pub_cmd_vel.publish(Twist())

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def pubMakerPointFollow(self, x, y):
        # Create rviz marker message
        marker = Marker()
        marker.header.frame_id = "frame_target"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'pointfollow'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.
        marker.pose.orientation.w = 1.
        marker.color.r = 1.0
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        self.pubMarker.publish(marker)

    def pub_Status(self, stt, modeRun, pose, offset, ss_x, ss_y, ss_a, meaning, warn):
        mess = Parking_respond()
        mess.status = stt
        mess.warning = warn
        mess.modeRun = modeRun
        mess.poseTarget = pose
        mess.offset = offset
        mess.ss_x = ss_x
        mess.ss_y = ss_y
        mess.ss_a = ss_a
        mess.message = meaning

        if rospy.get_time() - self.timePubRespond > float(1/self.rate_pubRespond) : # < 20hz 
            self.timePubRespond = rospy.get_time()
            self.pub_ParkingRespond.publish(mess)
        else :
            pass

    # function tf
    def get_transform_matrix(self, pose = Pose()):
        trans = [pose.position.x, pose.position.y, pose.position.z]
        rot = [0., 0., pose.orientation.z, pose.orientation.w]
        rotation_matrix = quaternion_matrix(rot)
        translation_vector = np.array(trans).reshape(3, 1)
        rotation_matrix[:3, 3] = translation_vector.flatten()
        return rotation_matrix
    

    def extract_translation_and_euler_angles(self, transform_matrix):
        translation = transform_matrix[:3, 3]
        R = transform_matrix[:3, :3]
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        euler_angles = np.array([x, y, z])
        return translation, euler_angles


    # function 
    def Meaning(self, stt, war):
        mess_step = {
            0: 'Step: Wait All data',
            1: 'Step: Select Mode',
            2: 'Step: make path',
            40: 'Find Goal Nearest',
            41: 'Parking vao ke',
            51: 'Doi reset',
            52: 'ERROER'
        }
        mess_warn = {
            0: 'AGV di chuyen binh thuong :)',
            1: 'AGV gap vat can :(',
            2: 'AGV gap loi chuong trinh',
        }
        mess = mess_step[stt] + ' | ' + mess_warn[war]
        return mess 


    def constrain(self, value_in, value_min, value_max):
        value_out = 0.0
        if value_in < value_min:
            value_out = value_min
        elif value_in > value_max:
            value_out = value_max
        else:
            value_out = value_in
        return value_out
    
    def funcDecelerationByTime(self, denlta_time, time_s, v_s, v_f):
        denlta_time_now = rospy.get_time() - time_s
        a = (v_f-v_s)/denlta_time

    def funcDecelerationByAcc(self, time_s, v_s, v_f, a):
        denlta_time_now = rospy.get_time() - time_s
        v_re = v_s + a*denlta_time_now
        if a > 0.:
            if v_re >= v_f:
                v_re = v_f
        else:
            if v_re <= v_f:
                v_re = v_f

        return v_re

    def getVeloctity(self, _safety, _statusVel):
        if _safety != 0: # trang thai giam toc do vat can
            if _safety == 1:
                self.statusVel = 0
                self.pub_Stop()
                return 0.
            elif _safety == 2: 
                self.statusVel = 0
                return self.infoPointFollow.velocity*(1./3.)
            else:
                self.statusVel = 0
                return self.infoPointFollow.velocity*(2./3.)
            
        else:      
            if _statusVel == 0:
                return self.infoPointFollow.velocity
            elif _statusVel == 1:
                return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, self.infoPointFollow.velocity, 0.2)
            elif _statusVel == 2:
                return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, self.infoPointFollow.velocity, -0.4)
            else:
                return 0.
            
    def findVelByLookAhead(self, l):
        a = 0.
        b = 0.
        if l >= self.max_lookahead:
            l = self.max_lookahead
        elif l <= self.min_lookahead:
            l = self.min_lookahead

        for i in range(self.numTable -1 ):
            if l >= self.listLookAhead[i] and l <= self.listLookAhead[i+1]:
                a = (self.listVel[i] - self.listVel[i+1])/(self.listLookAhead[i] - self.listLookAhead[i+1])
                b = self.listVel[i] - a*self.listLookAhead[i]

        return a*l + b
    
    def findLookAheadByVel(self, curr_velocity):
        a = 0.
        b = 0.
        if curr_velocity >= self.max_linearVelocity:
            curr_velocity = self.max_linearVelocity
        elif curr_velocity <= self.min_linearVelocity:
            curr_velocity = self.min_linearVelocity

        for i in range(self.numTable -1 ):
            if curr_velocity >= self.listVel[i] and curr_velocity <= self.listVel[i+1]:
                a = (self.listLookAhead[i] - self.listLookAhead[i+1])/(self.listVel[i] - self.listVel[i+1])
                b = self.listLookAhead[i] - a*self.listVel[i]

        return a*curr_velocity + b
    
    def findGoalNearest(self, xRb, yRb):
        index = 0
        min = float('inf')
        numpoint = len(self.infoPathFollow.info)
        for i in range(numpoint - 1, -1, -1):
            x = self.infoPathFollow.info[i].pose.position.x
            y = self.infoPathFollow.info[i].pose.position.y
            # vel = self.infoPathFollow.info[i].velocity
            dis = self.calculate_distance(x, y, xRb, yRb)

            if dis <= min:
                min = dis
                index = i

        infoPoint = InfoPointFollowing()
        infoPoint.indexInListpath = index
        infoPoint.velocity = self.infoPathFollow.info[index].velocity
        infoPoint.X = self.infoPathFollow.info[index].pose.position.x
        infoPoint.Y = self.infoPathFollow.info[index].pose.position.y

        return infoPoint, min
    
    def checkPathOutOfRange(self):
        poseX = self.poseRobot_convert.position.x
        poseY = self.poseRobot_convert.position.y
        self.infoPointFollow, dmin = self.findGoalNearest(poseX, poseY)
        return True
    
    def getWaitPoint(self, xRb, yRb, curr_velocity):
        indexSelect = -1
        lookahead = self.findLookAheadByVel(curr_velocity)
        if self.flagFollowPointFinish == False:
            disTarget = self.calculate_distance(0., 0., xRb, yRb)
            if disTarget <= lookahead:
                self.flagFollowPointFinish = True
                self.velSt = curr_velocity
                self.distDeceleration = disTarget

        numpoint = len(self.infoPathFollow.info)
        # find goal to goal finish
        for i in range(numpoint - 1,  -1, -1):
            x = self.infoPathFollow.info[i].pose.position.x
            y = self.infoPathFollow.info[i].pose.position.y
            # vel = self.infoPathFollow.info[i].velocity
            dis = self.calculate_distance(x, y, xRb, yRb)
            if dis <= lookahead:
                indexSelect = i
                break

        if indexSelect == -1:
            return False
        
        else:
            self.infoPointFollow.indexInListpath = indexSelect

            self.infoPointFollow.X = self.infoPathFollow.info[indexSelect].pose.position.x
            self.infoPointFollow.Y = self.infoPathFollow.info[indexSelect].pose.position.y
        
            if self.statusVel != 2:
                vel = self.infoPathFollow.info[indexSelect].velocity
                self.infoPointFollow.velocity = self.constrain(vel, self.min_linearVelocity, self.max_linearVelocity)

            return True
        

    def follow_targetVS2(self):
        twist = Twist()
        poseX = self.poseRobot_convert.position.x
        poseY = self.poseRobot_convert.position.y
        angleAGV = self.angleRobot_convert
        dis = sqrt(poseX*poseX + poseY*poseY)
        print(poseX, poseY, dis)

        if self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2:
            if poseX <= 0.005:
                self.pub_Stop()
                return 1
            
            else:
                if self.zone_lidar.zone_sick_behind == 11 and self.req_parking.modeRun == 1:
                    # print("co vat can")
                    self.pub_Stop()
                    self.statusVel = 0
                    self.warn_agv = 1
                    
                else:
                    self.warn_agv = 0
                    distance_decel = self.findLookAheadByVel(self.infoPointFollow.velocity)
                    distance_decel = 0.4
                    if (fabs(poseY) <= 0.01 and fabs(angleAGV) <= radians(1.)) or fabs(poseX) <= 0.6: # tinh van toc goc
                        vel_x = 0.
                        if dis <= distance_decel: # tinh van toc dai theo khoang cach
                            vel_x = self.infoPointFollow.velocity*(dis/distance_decel)
                            if vel_x >= self.infoPointFollow.velocity:
                                vel_x = self.infoPointFollow.velocity
                            if vel_x <= self.min_linearVelocity:
                                vel_x = self.min_linearVelocity

                        else:
                            vel_x = self.infoPointFollow.velocity

                        kg = 0.35   # 0.7
                        dentaAngle = -angleAGV
                        velAng = kg*fabs(dentaAngle)
                        if velAng >= 0.15:
                            velAng = 0.15
                        
                        if dentaAngle > 0:
                            velAng = velAng
                        else:
                            velAng = -velAng
                        
                        twist.linear.x = -vel_x
                        twist.angular.z = velAng
                        self.pub_cmdVel(twist, self.rate_pubVel)

                    else:
                        if self.flagFollowPointFinish:
                            self.curr_velocity = self.velSt*(dis/self.distDeceleration)
                            self.curr_velocity = self.constrain(self.curr_velocity, self.min_linearVelocity, self.velSt)
                            print(self.curr_velocity, self.infoPointFollow.velocity, "follow Finish Target")
                            velX = -self.curr_velocity
                            if self.getWaitPoint(poseX, poseY, self.curr_velocity):
                                velX = -self.curr_velocity
                                xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPointFollow.X, self.infoPointFollow.Y)
                                velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                                twist.linear.x = velX
                                twist.angular.z = velAng

                                self.pub_cmdVel(twist, self.rate_pubVel)
                            
                            else:
                                pass
                                # self.pub_Stop()
                                # self.warn_agv = 2
                                # print("Im stop here 111 ")
                                # return -1

                        else:
                            # kiem tra dang follow point cuoi chua
                            # them phuong trinh giam toc
                            if self.curr_velocity < self.infoPointFollow.velocity and self.statusVel != 1:
                                self.statusVel = 1
                                self.velSt = self.curr_velocity
                                self.saveTimeVel = rospy.get_time()

                            elif self.curr_velocity > self.infoPointFollow.velocity and self.statusVel != 2:
                                self.statusVel = 2
                                self.velSt = self.curr_velocity
                                self.saveTimeVel = rospy.get_time()

                            elif self.curr_velocity == self.infoPointFollow.velocity:
                                self.statusVel = 0

                            self.curr_velocity = self.getVeloctity(0, self.statusVel)
                            print(self.curr_velocity, self.statusVel, self.infoPointFollow.velocity, "follow Normal Target")

                            if self.curr_velocity != 0. :
                                if self.getWaitPoint(poseX, poseY, self.curr_velocity):
                                    velX = -self.curr_velocity
                                    xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPointFollow.X, self.infoPointFollow.Y)
                                    velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                                    twist.linear.x = velX
                                    twist.angular.z = velAng

                                    self.pub_cmdVel(twist, self.rate_pubVel)
                
                                else:
                                    pass
                                    # self.pub_Stop()
                                    # self.warn_agv = 2
                                    # print("Im stop here 222 ")
                                    # return -1
                            
        else:
            if self.req_parking.modeRun == 3:
                print("Recieve data Stop!")
                self.pub_cmdVel(Twist(), self.rate_pubVel)
            elif self.req_parking.modeRun == 0:
                print("Recieve data Reset!")
                self.pub_Stop()
                self.resetAll()

        return 0
    
    def SIGN(self, num):
        if num > 0:
            return 1
        elif num < 0:
            return -1
        return 0
    
        
    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

        
    # function Navigation 
    def convert_relative_coordinates(self, X_cv, Y_cv):
        angle = -self.angleRobot_convert
        _X_cv = (X_cv - self.poseRobot_convert.position.x)*cos(angle) - (Y_cv - self.poseRobot_convert.position.y)*sin(angle)
        _Y_cv = (X_cv - self.poseRobot_convert.position.x)*sin(angle) + (Y_cv - self.poseRobot_convert.position.y)*cos(angle)
        return _X_cv, _Y_cv

    def control_navigation(self, X_point_goal, Y_point_goal, vel_x): # 1 goal phia truoc | -1 goal phia sau
        vel_th = 0.0
        xGoal = X_point_goal
        yGoal = Y_point_goal

        if fabs(yGoal) <= 0.005:
            vel_th = 0.

        else:
            l = (xGoal*xGoal) + (yGoal*yGoal)
            r = l/(2*fabs(yGoal))
            # print(vel_x, r, l)
            vel = fabs(vel_x)/r
            if yGoal > 0:
                vel_th = vel*self.SIGN(vel_x)
            else:
                vel_th = -vel*self.SIGN(vel_x)

        return vel_th
    

    def resetAll(self):
        self.process = 1
        self.is_request_parking = 0
        self.warn_agv = 0
        self.infoPointFollow = InfoPointFollowing()
        self.infoPathFollow = InfoPathFollowing()
        self.saveTimeVel = rospy.get_time()
        self.statusVel = 0
        self.velSt = 0.
        self.flagFollowPointFinish = False
        
    def run(self):
        while not rospy.is_shutdown():    
            if self.process == 0:
                ck = 0
                if self.is_pose_robot:
                    ck += 1
                if self.is_check_zone:
                    ck += 1
                if ck == 2:
                    self.process = 1
                    print("Done receive all need data <(^-^)> ")

            # chờ data parking
            elif self.process == 1:
                if self.is_request_parking:
                    modeRun = self.req_parking.modeRun
                    if modeRun == 1 or modeRun == 2:
                        # convert goc
                        poseTarget = Pose()
                        poseTarget.position.x = self.req_parking.poseTarget.position.x
                        poseTarget.position.y = self.req_parking.poseTarget.position.y
                        poseTarget.position.z = self.req_parking.poseTarget.position.z

                        euler = euler_from_quaternion((0.0, 0.0, self.req_parking.poseTarget.orientation.z, self.req_parking.poseTarget.orientation.w))
                        # ----- fix ------
                        angle_target = 0.
                        if (euler[2] >= 0):
                            angle_target = euler[2] - PI
                        else:
                            angle_target = PI + euler[2]

                        # -- close fix

                        quat = quaternion_from_euler(0., 0., angle_target)
                        poseTarget.orientation.z = quat[2]
                        poseTarget.orientation.w = quat[3]
                        # tính vị trí AGV trong frame Target
                        self.matrix_origin_to_target = self.get_transform_matrix(poseTarget)
                        matrix_origin_to_robot = self.get_transform_matrix(self.poseRbMa)
                        matrix_out = np.dot(np.linalg.inv(self.matrix_origin_to_target), matrix_origin_to_robot)
                        translation, euler_angles = self.extract_translation_and_euler_angles(matrix_out)
                        print(translation[0], translation[1], translation[2], euler_angles)
                        self.poseRobot_convert.position.x = translation[0]
                        self.poseRobot_convert.position.y = translation[1]
                        self.poseRobot_convert.position.z = translation[2]
                        quat = quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
                        self.poseRobot_convert.orientation.x = quat[0]
                        self.poseRobot_convert.orientation.y = quat[1]
                        self.poseRobot_convert.orientation.z = quat[2]
                        self.poseRobot_convert.orientation.w = quat[3]

                        self.process = 2

            # tạo đường dẫn
            elif self.process == 2:
                try:
                    listPoint = np.empty((0,2), dtype=float)
                    pointOne = Point(self.poseRobot_convert.position.x, self.poseRobot_convert.position.y) # vi tri agv hien tai
                    pointSecond = Point(self.distanceGoStraight + self.distanceAlign, 0.)
                    if pointOne.x > pointSecond.x:
                        myPath = QuadraticBezierCurves(pointOne, pointSecond, Point(pointOne.x, 0.))
                        listPoint = myPath.slip()


                    rx_strline = np.arange(pointSecond.x - self.stepPoint, -1., -self.stepPoint, float)
                    for i in rx_strline:
                        listPoint = np.append(listPoint, np.array([[i, 0.]]), axis=0)

                    del self.infoPathFollow.info[:]
                    for i in listPoint:
                        pointPath = PointOfPath()
                        pointPath.pose.position.x = i[0]
                        pointPath.pose.position.y = i[1]
                        pointPath.pose.orientation.w = 1.0

                        pointPath.velocity = 0.15

                        self.infoPathFollow.info.append(pointPath)

                    self.process = 40
                    print("make path done")
                
                except:
                    print ("make path fail")
                    self.process = 52
            
            # follow path
            elif self.process == 40:
                matrix_origin_to_robot = self.get_transform_matrix(self.poseRbMa)
                matrix_out = np.dot(np.linalg.inv(self.matrix_origin_to_target), matrix_origin_to_robot)
                translation, euler_angles = self.extract_translation_and_euler_angles(matrix_out)
                self.poseRobot_convert.position.x = translation[0]
                self.poseRobot_convert.position.y = translation[1]
                self.poseRobot_convert.position.z = translation[2]
                quat = quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
                self.poseRobot_convert.orientation.x = quat[0]
                self.poseRobot_convert.orientation.y = quat[1]
                self.poseRobot_convert.orientation.z = quat[2]
                self.poseRobot_convert.orientation.w = quat[3]

                if self.checkPathOutOfRange():
                    print("Done find Goal Nearest")
                    self.process = 41

            elif self.process == 41: # follow vao ke
                # tinh toa do robot convert
                matrix_origin_to_robot = self.get_transform_matrix(self.poseRbMa)
                matrix_out = np.dot(np.linalg.inv(self.matrix_origin_to_target), matrix_origin_to_robot)
                translation, euler_angles = self.extract_translation_and_euler_angles(matrix_out)
                self.poseRobot_convert.position.x = translation[0]
                self.poseRobot_convert.position.y = translation[1]
                self.poseRobot_convert.position.z = translation[2]
                quat = quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
                self.poseRobot_convert.orientation.x = quat[0]
                self.poseRobot_convert.orientation.y = quat[1]
                self.poseRobot_convert.orientation.z = quat[2]
                self.poseRobot_convert.orientation.w = quat[3]
                self.angleRobot_convert = euler_angles[2]

                stt = self.follow_targetVS2()
                if stt == 1:
                    print("Arrived at the Target location")
                    time.sleep(0.5)
                    self.process = 51

                elif stt == -1:
                    self.pub_Stop()
                    print("Something went wrong (T-T)")
                    self.process = 52

            elif self.process == 51: # Reset
                if self.req_parking.modeRun == 0: #Reset
                    self.resetAll()
                    print("RESET")

            elif self.process == 52:
                if self.req_parking.modeRun == 0: # loi - doi reset
                    self.resetAll()
                    print("RESET - ERROR")

            mess_pub = self.Meaning(self.process, self.warn_agv)
            self.pub_Status(self.process, self.req_parking.modeRun, self.req_parking.poseTarget, self.req_parking.offset, self.ss_x, self.ss_y, self.ss_a, mess_pub, self.warn_agv)
            # self.pubMakerPointFollow(self.infoPointFollow.X, self.infoPointFollow.Y)  
            self.rate.sleep()


def main():
    # Start the job threads
    class_1 = ParkingAGV()
    # class_1.run()
    class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()