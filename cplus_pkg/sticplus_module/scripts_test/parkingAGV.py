#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sti_msgs.msg import APathParking, PointOfPath, HC_info
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, fabs, cos, sin
from math import pi as PI
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker


class InfoPathFollowing():
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
        self.min_linearVelocity = rospy.get_param('~min_linearVelocity', 0.012)

        self.max_lookahead = rospy.get_param('~min_lookahead', 0.3)
        self.min_lookahead = rospy.get_param('~min_lookahead', 0.05)
        self.lookahead_ratio = rospy.get_param('~lookahead_ratio', 8.0)

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.callback_poseRobot, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        rospy.Subscriber('/parking_request', Parking_request, self.parkingRequest_callback)
        self.req_parking = Parking_request() 
        self.is_request_parking = False

        rospy.Subscriber('/poseParking', PoseStamped, self.poseParking_callback)
        self.is_poseParking = False
        self.poseParking = Pose()
        self.poseParkingStamped = PoseStamped()

        rospy.Subscriber("/path_parking", APathParking, self.path_callback)
        self.is_path = False
        self.startPoint = Pose()
        self.intermediaryPoint = Pose()
        self.dataPath = APathParking()
        self.theta_AngentFisrtPoint = 0.

        rospy.Subscriber("/HC_info", HC_info, self.zone_callback)
        self.zone_lidar = HC_info()
        self.is_check_zone = False	

        # Pub topic
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

        self.listVel = [0.012, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]
        self.numTable = len(self.listVel)
        self.listLookAhead = [0.12, 0.22, 0.3, 0.4, 0.55, 0.65, 0.7, 0.75, 0.8, 0.9, 1.05, 1.28, 1.34, 1.39, 1.44, 1.49]

        self.infoPathFollow = InfoPathFollowing()
        self.saveTimeVel = rospy.get_time()
        self.statusVel = 0
        self.velSt = 0.
        self.warn_agv = 0

    # function callback
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
        self.is_request_parking = True

    def poseParking_callback(self, data):
        self.poseParkingStamped = data
        self.poseParking = data.pose
        quata = ( self.poseParking.orientation.x,\
                self.poseParking.orientation.y,\
                self.poseParking.orientation.z,\
                self.poseParking.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_robotParking = euler[2]
        self.is_poseParking = True

    def path_callback(self, data):
        self.dataPath = data
        quata = ( self.dataPath.poseStart.orientation.x,\
                self.dataPath.poseStart.orientation.y,\
                self.dataPath.poseStart.orientation.z,\
                self.dataPath.poseStart.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_AngentFisrtPoint = euler[2]
        self.startPoint = self.dataPath.poseStart
        self.intermediaryPoint = self.dataPath.poseIntermediary
        self.is_path = True

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
        marker.pose.position.z = -1.
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

    # function 
    def Meaning(self, stt, war):
        mess_step = {
            0: 'Step: Wait All data',
            1: 'Step: Select Mode',
            2: 'Find Goal Nearest',
            3: 'Parking vao ke',
            5: 'Doi reset',
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

    def rotary_around(self, thetaTarget, angular_velocity, permission_tolerance):
        twist = Twist()
        ss_theta = thetaTarget - self.theta_robotParking
        if fabs(ss_theta) > PI:
            newss_theta = 2*PI - fabs(ss_theta)
            if ss_theta > 0.:
                ss_theta = - newss_theta
            else:
                ss_theta = newss_theta

        if fabs(ss_theta) > permission_tolerance:
            if ss_theta > 0.: # quay trai
                twist.angular.z = angular_velocity
            else:
                twist.angular.z = -angular_velocity
            
            self.pub_cmdVel(twist, self.rate_pubVel)
            return 0
        
        self.pub_Stop()
        return 1
    
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
                return self.infoPathFollow.velocity*(1./3.)
            else:
                self.statusVel = 0
                return self.infoPathFollow.velocity*(2./3.)
            
        else:      
            if _statusVel == 0:
                return self.infoPathFollow.velocity
            elif _statusVel == 1:
                return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, self.infoPathFollow.velocity, 0.12)
            elif _statusVel == 2:
                return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, self.infoPathFollow.velocity, -0.4)
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
        numpoint = len(self.dataPath.info)
        for i in range(numpoint - 1, -1, -1):
            x = self.dataPath.info[i].pose.position.x
            y = self.dataPath.info[i].pose.position.y
            # vel = self.dataPath.info[i].velocity
            dis = self.calculate_distance(x, y, xRb, yRb)

            if dis <= min:
                min = dis
                index = i

        infoPoint = InfoPathFollowing()
        infoPoint.indexInListpath = index
        infoPoint.velocity = self.dataPath.info[index].velocity
        infoPoint.X = self.dataPath.info[index].pose.position.x
        infoPoint.Y = self.dataPath.info[index].pose.position.y

        return infoPoint, min
    
    def checkPathOutOfRange(self):
        poseX = self.poseParking.position.x
        poseY = self.poseParking.position.y
        self.infoPathFollow, dmin = self.findGoalNearest(poseX, poseY)
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

        numpoint = len(self.dataPath.info)
        # find goal to goal finish
        for i in range(numpoint - 1,  -1, -1):
            x = self.dataPath.info[i].pose.position.x
            y = self.dataPath.info[i].pose.position.y
            # vel = self.dataPath.info[i].velocity
            dis = self.calculate_distance(x, y, xRb, yRb)
            if dis <= lookahead:
                indexSelect = i
                break

        if indexSelect == -1:
            return False
        
        else:
            self.infoPathFollow.indexInListpath = indexSelect

            self.infoPathFollow.X = self.dataPath.info[indexSelect].pose.position.x
            self.infoPathFollow.Y = self.dataPath.info[indexSelect].pose.position.y
        
            if self.statusVel != 2:
                vel = self.dataPath.info[indexSelect].velocity
                self.infoPathFollow.velocity = self.constrain(vel, self.min_linearVelocity, self.max_linearVelocity)

            return True
    
    def follow_target(self):
        twist = Twist()
        poseX = self.poseParking.position.x
        poseY = self.poseParking.position.y
        # angleAGV = self.theta_robotParking
        dis = sqrt(poseX*poseX + poseY*poseY)
        print(poseX, poseY, dis)

        if self.dataPath.modeRun == 1 or self.dataPath.modeRun == 2:
            if poseX <= 0.:
                self.pub_Stop()
                return 1
            
            else:
                if self.zone_lidar.zone_sick_behind == 1 and self.req_parking.modeRun == 1:
                    # print("co vat can")
                    self.pub_Stop()
                    self.statusVel = 0
                    self.warn_agv = 1
                    
                else:
                    self.warn_agv = 0
                    if self.flagFollowPointFinish:
                        self.curr_velocity = self.velSt*(dis/self.distDeceleration)
                        self.curr_velocity = self.constrain(self.curr_velocity, self.min_linearVelocity, self.velSt)
                        print(self.curr_velocity, self.infoPathFollow.velocity, "follow Finish Target")
                        velX = -self.curr_velocity
                        if self.getWaitPoint(poseX, poseY, self.curr_velocity):
                            velX = -self.curr_velocity
                            xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                            velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                            twist.linear.x = velX
                            twist.angular.z = velAng

                            self.pub_cmdVel(twist, self.rate_pubVel)
                        
                        else:
                            self.pub_Stop()
                            self.warn_agv = 2
                            return -1

                    else:
                        # kiem tra dang follow point cuoi chua
                        # them phuong trinh giam toc
                        if self.curr_velocity < self.infoPathFollow.velocity and self.statusVel != 1:
                            self.statusVel = 1
                            self.velSt = self.curr_velocity
                            self.saveTimeVel = rospy.get_time()

                        elif self.curr_velocity > self.infoPathFollow.velocity and self.statusVel != 2:
                            self.statusVel = 2
                            self.velSt = self.curr_velocity
                            self.saveTimeVel = rospy.get_time()

                        elif self.curr_velocity == self.infoPathFollow.velocity:
                            self.statusVel = 0

                        self.curr_velocity = self.getVeloctity(0, self.statusVel)
                        print(self.curr_velocity, self.statusVel, self.infoPathFollow.velocity, "follow Normal Target")

                        if self.curr_velocity != 0. :
                            if self.getWaitPoint(poseX, poseY, self.curr_velocity):
                                velX = -self.curr_velocity
                                xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                                velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                                twist.linear.x = velX
                                twist.angular.z = velAng

                                self.pub_cmdVel(twist, self.rate_pubVel)
               
                            else:
                                self.pub_Stop()
                                self.warn_agv = 2
                                return -1
                            
        else:
            if self.req_parking.modeRun == 3:
                # print("Recieve data Stop!")
                self.pub_cmdVel(Twist(), self.rate_pubVel)
            elif self.req_parking.modeRun == 0:
                # print("Recieve data Reset!")
                self.resetAll()
                self.pub_Stop()

        return 0
                
    def resetAll(self):
        self.process = 1
        self.is_request_parking = False
        self.is_poseParking = False
        self.is_path = False
        self.warn_agv = 0
        self.infoPathFollow = InfoPathFollowing()
        self.saveTimeVel = rospy.get_time()
        self.statusVel = 0
        self.velSt = 0.
        self.flagFollowPointFinish = False


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
        angle = -self.theta_robotParking
        _X_cv = (X_cv - self.poseParking.position.x)*cos(angle) - (Y_cv - self.poseParking.position.y)*sin(angle)
        _Y_cv = (X_cv - self.poseParking.position.x)*sin(angle) + (Y_cv - self.poseParking.position.y)*cos(angle)
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

    def run(self):
        self.process = 1
        while not rospy.is_shutdown():    
            if self.process == 0:
                ck = 0
                if self.is_pose_robot:
                    ck += 1
                if self.is_check_zone:
                    ck += 1
                if ck == 2:
                    self.process = 1
                    rospy.loginfo("Done receive all need data <(^-^)> ")

            elif self.process == 1:
                if self.is_request_parking == True and (self.dataPath.modeRun == 1 or self.dataPath.modeRun == 2):
                    print("recieve data Run Parking")
                    if  self.is_poseParking:
                        print("recieve data transfrom pose target")
                        if self.is_path:
                            print("recieve data local goal")
                            self.process = 2

            elif self.process == 2: # tim goal gan nhat
                if self.checkPathOutOfRange():
                    print("Done find Goal Nearest")
                    self.process = 3

            elif self.process == 3: # follow vao ke
                stt = self.follow_target()
                if stt == 1:
                    print("Arrived at the Target location")
                    time.sleep(0.5)
                    self.process = 5

                elif stt == -1:
                    self.pub_Stop()
                    print("Something went wrong (T-T)")
                    self.process = 4

            elif self.process == 5: # Reset
                if self.req_parking.modeRun == 0: #Reset
                    self.resetAll()

            elif self.process == 4:
                if self.req_parking.modeRun == 0: # loi - doi reset
                    self.resetAll()

            mess_pub = self.Meaning(self.process, self.warn_agv)
            self.pub_Status(self.process, self.dataPath.modeRun, self.req_parking.poseTarget, self.req_parking.offset, self.ss_x, self.ss_y, self.ss_a, mess_pub, self.warn_agv)
            self.pubMakerPointFollow(self.infoPathFollow.X, self.infoPathFollow.Y)  
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