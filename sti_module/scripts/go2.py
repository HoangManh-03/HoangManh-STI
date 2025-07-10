#!/usr/bin/python3

import threading
import time
import rospy
from std_msgs.msg import String, Bool, Int8

import sys
import struct
import string
import roslib
import serial
import signal

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sti_msgs.msg import Move_request, Move_respond, Zone_lidar_2head, POWER_info, Status_goalControl, Velocities, Status_goal_control, HC_info

import os 
from nav_msgs.msg import Path
import math


class PID:
    def __init__(self):
        self.kp = 0.1
        self.ki = 0.001
        self.kd = 0.00001
        self.previous_error = 0
        self.integral = 0

    def calculate(self, error):
        
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output


class bezier_curve_control:
    def __init__(self):
        self.d1 = 0.5
        self.d2 = 0.5
        self.P1 = []
        self.P2 = []
    def bezier_curve(self, t, p0, p1, p2, p3):
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
        y = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] + 3 * (1 - t) * t**2 * p2[1] + t**3 * p3[1]
        return [x, y]
    def calculate_velocity_and_angle(self, points, dt=0.1):
        velocities = []
        for i in range(len(points) - 1):
            dx = points[i + 1][0] - points[i][0]
            dy = points[i + 1][1] - points[i][1]
            distance = math.sqrt(dx**2 + dy**2)
            linear_vel = distance / dt
            angular_vel = math.atan2(dy, dx)
            velocities.append((linear_vel, angular_vel))
        return linear_vel, angular_vel
    
    def caculateP1P2(self, P0, P3, theta):
        P1 = [P0[0] + self.d1 * math.cos(theta), P0[1] + self.d1 * math.sin(theta)]
        P2 = [P3[0] - self.d2 * math.cos(theta), P3[1] - self.d2 * math.sin(theta)]
        return P1, P2

class goalControl():
    def __init__(self):


        rospy.init_node('goal_control_v2', anonymous=True)

        self.targetx = 8.325
        self.targety = 1.000
        self.target_z = 3.14
        self.kc_backward = 1.2
        self.rate = rospy.Rate(50)
        self.time_tr = rospy.get_time()
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.check_theta = 0
        
        self.bezier_curve = bezier_curve_control()
        self.pid = PID()
        rospy.Subscriber('/odometry', Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.is_odom_rb = False
        self.odom_rb = Odometry()
        rospy.Subscriber("/raw_vel", TwistWithCovarianceStamped, self.rawvel_callback)	
        self.is_raw_vel = False
        self.vel_raw = TwistWithCovarianceStamped()
        rospy.Subscriber('/robotPose_nav', PoseStamped, self.getPose, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.theta_rb_ht = 0.0
        self.status = 999
        self.rate_cmdVel = 30
        self.angle_giam_toc = 45.0*PI/180.0
        self.vel_rot_step1 = 0.45
        self.tolerance_rot_step1 = 0.005

        self.point_goal_start_x = 0.0
        self.point_goal_start_y = 0.0
        self.is_target = 1
        self.dist_ahead_min = 0.05
        self.dist_ahead_max = 1.4
        self.angle_find_vel = 25.0*PI/180.0
        # self.angle_find_vel
        self.vel_x_control = 0
        self.min_vel_x_gh = 0.2
        self.is_need_pttt = 0
        self.is_pre_pttt = 0
        self.dis_gt = 1.15
        self.vel_x_max = 1.0
        self.min_vel_x = 0.04
        self.vel_x_control = 0.3
        self.cur_goal_is = 3    #1 diem thuong, 2 diem dac biet, 3 diem dich
        self.tol_target = 0.01
        self.tol_simple = 0.04
        self.distance_hc = 0.0 
        
        self.dis_gt_khilui = 0.3
        
        rospy.on_shutdown(self.fnShutDown)

    def getPose(self, data):
        self.is_pose_robot = True
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb_ht = euler[2]

    def rawvel_callback(self, data):
        self.vel_raw = data
        self.is_raw_vel = True
        
    def cbGetRobotOdom(self, data):
        self.odom_rb = data
        self.is_odom_rb = True

    def Cal_angle(self, x_position, y_position, x_target, y_target, x_theta):
        
        theta = atan2(y_target - y_position, x_target - x_position)
        # print("Góc theta",theta)
        return x_theta - theta
        
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def pub_cmdVel(self, twist , rate):

        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass

    def distanceAB(self, xA, yA, xB, yB):
        return sqrt((xA - xB)**2 + (yA - yB)**2)

    def turn_ar(self, theta, tol_theta, vel_rot):
        if fabs(theta) > tol_theta:
            if theta > 0: 
                if fabs(theta) <= self.angle_giam_toc:
                    vel_th = (fabs(theta)/self.angle_giam_toc)*vel_rot
                else:
                    vel_th = vel_rot

                if vel_th < 0.1:
                    vel_th = 0.1

                return -vel_th

            if theta < 0: #quay phai , vel_z < 0
                # print "a"
                if fabs(theta) <= self.angle_giam_toc:
                    vel_th = (fabs(theta)/self.angle_giam_toc)*(-vel_rot)
                else:
                    vel_th = -vel_rot
                if vel_th > -0.1:
                    vel_th = -0.1
                return -vel_th
        else: 
            return -10 

    def round_precision(self, x1, x2):
        precision = 0.01
        return 0 if abs(x2 - x1) >= precision else 1

    def stop(self):
        for i in range(2):
            self.pub_cmd_vel.publish(Twist())

    def control_navigation(self, X_point_goal, Y_point_goal, vel_x, theta, dis):
        vel_th = 0.0
        l = (X_point_goal*X_point_goal) + (Y_point_goal*Y_point_goal)
        if Y_point_goal == 0:
            print(Y_point_goal)
            Y_point_goal = 0.0001

        r = l/(2*fabs(Y_point_goal))
        vel = vel_x/r

        if Y_point_goal > 0:
            vel_th = vel
        else:
            vel_th = -vel

        return vel_th
    
    def control_naviTarget(self, X_point_goal, Y_point_goal):
        vel_th = 0.0
        vel = 0.0
        
        if round(fabs(Y_point_goal), 3) == 0.0:
            vel = 0.0
        else:
            if round(fabs(X_point_goal), 3) == 0.0:
                X_point_goal = 0.0001
            angle = atan2(fabs(Y_point_goal), X_point_goal)
            vel = 0.45*angle
            
        if Y_point_goal > 0:
            vel_th = vel
        else:
            vel_th = -vel
        
        return vel_th

    def find_hc(self, X_s, Y_s, X_f, Y_f):
        X_n = Y_n = 0.0
        kc_hinh_chieu = 0.0
        # pt duong thang quy dao
        a_qd = Y_s - Y_f
        b_qd = X_f - X_s
        c_qd = -X_s*a_qd -Y_s*b_qd

        # pt tu RB to Goal
        a_rg = self.poseRbMa.position.y - Y_f
        b_rg = X_f - self.poseRbMa.position.x

        # diem hinh chieu
        if self.poseRbMa.position.x == X_s and self.poseRbMa.position.y == Y_s :
            # print('VAO DAY ROI')
            X_n = X_s
            Y_n = Y_s
        else:
            #pt duong thang hinh chieu
            a_hc = b_qd
            b_hc = -a_qd
            c_hc = -self.poseRbMa.position.x*a_hc -self.poseRbMa.position.y*b_hc

            X_n = ((c_hc*b_qd)-(c_qd*b_hc))/((a_qd*b_hc)-(b_qd*a_hc))
            Y_n = ((c_hc*a_qd)-(c_qd*a_hc))/((a_hc*b_qd)-(b_hc*a_qd))

        kc_hinh_chieu = sqrt((X_n - self.poseRbMa.position.x)*(X_n - self.poseRbMa.position.x) + (Y_n - self.poseRbMa.position.y)*(Y_n - self.poseRbMa.position.y))

        return X_n, Y_n, a_qd, b_qd, c_qd, kc_hinh_chieu, a_rg, b_rg



    def find_point_goal(self, X_s, Y_s, X_f, Y_f, a_qd, b_qd, c_qd, X_n, Y_n, is_target):
        X_g = Y_g =  X_g1 = Y_g1 = X_g2 = Y_g2 = 0.0
        dis_ahead = 0.0
        vector_point1_x = vector_point1_y = vector_point2_x = vector_point2_x =0.0
        v_a = v_b = 0.0
        vector_qd_x = X_s - X_f
        vector_qd_y = Y_s - Y_f
        x_cv = y_cv = 0.0
        kc_g1 = kc_g2 = 0.0
        kc_ns = kc_nf = 0.0
        is_over = False
        kc_sf = sqrt((X_f - X_s)*(X_f - X_s) + (Y_f - Y_s)*(Y_f - Y_s))
        # pt duong thang quy dao

        kc_nf = sqrt((X_n - X_f)*(X_n - X_f) + (Y_n - Y_f)*(Y_n - Y_f))
        kc_ns = sqrt((X_n - X_s)*(X_n - X_s) + (Y_n - Y_s)*(Y_n - Y_s))

        if kc_ns >= kc_sf and kc_nf <= kc_sf:
            is_over = True
        else:
            is_over = False
            
        if is_target == 1:
            dis_ahead = self.dist_ahead_min 
        else:
            dis_ahead = self.dist_ahead_max

        if is_target == 1 and kc_nf < dis_ahead:
            X_g = X_f
            Y_g = Y_f
            
        else:
            if  b_qd == 0.0:
                X_g1 = X_g2 = -c_qd/a_qd
                Y_g1 = -sqrt(dis_ahead*dis_ahead - (X_g1 - X_n)*(X_g1 - X_n)) + Y_n
                Y_g2 = sqrt(dis_ahead*dis_ahead - (X_g2 - X_n)*(X_g2 - X_n)) + Y_n
            else:
                la = (1.0 + (a_qd/b_qd)*(a_qd/b_qd))
                lb = -2.0*(X_n - (a_qd/b_qd)*((c_qd/b_qd) + Y_n))
                lc = X_n*X_n + ((c_qd/b_qd) + Y_n)*((c_qd/b_qd) + Y_n) - dis_ahead*dis_ahead
                denlta = lb*lb - 4.0*la*lc

                X_g1 = (-lb + sqrt(denlta))/(2.0*la)
                X_g2 = (-lb - sqrt(denlta))/(2.0*la)

                Y_g1 = (-c_qd - a_qd*X_g1)/b_qd
                Y_g2 = (-c_qd - a_qd*X_g2)/b_qd

            # loai nghiem bang vector
            vector_qd_x = X_s - X_f
            vector_qd_y = Y_s - Y_f

            vector_point1_x = X_n - X_g1
            vector_point1_y = Y_n - Y_g1

            if vector_qd_x == 0.0:
                if vector_qd_y*vector_point1_y > 0.0:
                    X_g = X_g1
                    Y_g = Y_g1
                else:
                    X_g = X_g2
                    Y_g = Y_g2
            elif vector_qd_y == 0.0:
                if vector_qd_x*vector_point1_x > 0.0:
                    X_g = X_g1
                    Y_g = Y_g1
                else:
                    X_g = X_g2
                    Y_g = Y_g2

            else:
                v_a = vector_qd_x/vector_point1_x
                v_b = vector_qd_y/vector_point1_y
                if v_a*v_b > 0.0 and v_a > 0.0:
                    X_g = X_g1
                    Y_g = Y_g1
                else:
                    X_g = X_g2
                    Y_g = Y_g2

        # print(X_g, Y_g)
        x_cv, y_cv = self.convert_relative_coordinates(X_g, Y_g)
        return x_cv, y_cv, kc_nf, kc_sf, is_over

    def convert_relative_coordinates(self, X_cv, Y_cv):
        angle = -self.theta_rb_ht
        _X_cv = (X_cv - self.poseRbMa.position.x)*cos(angle) - (Y_cv - self.poseRbMa.position.y)*sin(angle)
        _Y_cv = (X_cv - self.poseRbMa.position.x)*sin(angle) + (Y_cv - self.poseRbMa.position.y)*cos(angle)
        
        return _X_cv, _Y_cv


#####################################################################################################################################
#################################################Bat dau quy trinh###################################################################
#####################################################################################################################################

    def run(self):
        while not rospy.is_shutdown():
            if self.status == 1:
                self.theta = self.Cal_angle(self.poseRbMa.position.x, self.poseRbMa.position.y, self.targetx, self.targety, self.theta_rb_ht)
                print("theta", self.theta)
                # print(self.theta)
                twist = Twist()
                gt = self.turn_ar(self.theta, self.tolerance_rot_step1, self.vel_rot_step1)
                if gt == -10:
                    print("aaaaaaaaa")
                    self.stop()
                    rospy.sleep(0.3)
                    self.status = 1000
                    self.point_goal_start_x = self.poseRbMa.position.x
                    self.point_goal_start_y = self.poseRbMa.position.y

                else:
                    twist = Twist()
                    twist.angular.z = gt
                    self.pub_cmdVel(twist, self.rate_cmdVel)
                    
            elif self.status == 2:
                
                # print("distance", self.distanceAB(self.poseRbMa.position.x, self.poseRbMa.position.y, self.targetx, self.targety))
                
                self.X_n, self.Y_n, self.a_qd, self.b_qd, self.c_qd, self.dis_hc, arg, b_rg = self.find_hc(self.point_goal_start_x,\
                                                                                                self.point_goal_start_y,\
                                                                                                self.targetx,\
                                                                                                self.targety)
                
                self.is_target = 1

                self.x_td_goal, self.y_td_goal, self.kc_con_lai, self.kc_qd, self.is_over_goal = self.find_point_goal(self.point_goal_start_x,\
                                                                                                                self.point_goal_start_y,\
                                                                                                                self.targetx,\
                                                                                                                self.targety,\
                                                                                                                self.a_qd,self.b_qd,self.c_qd,\
                                                                                                                self.X_n,self.Y_n,\
                                                                                                                self.is_target)
                
                distance_hc = self.distanceAB(self.poseRbMa.position.x, self.poseRbMa.position.y, self.X_n, self.Y_n)
                
                self.distance_goal = self.distanceAB(self.poseRbMa.position.x, self.poseRbMa.position.y, self.targetx, self.targety)
                
                if fabs(self.theta) > self.angle_find_vel:
                    v_x = self.min_vel_x_gh

                elif round(fabs(self.theta),3) == 0.0:
                    v_x = self.vel_x_control

                else:
                    v_x = self.min_vel_x_gh + ((self.angle_find_vel - fabs(self.theta))/self.angle_find_vel)*(self.vel_x_control - self.min_vel_x_gh)                

                v_x_send = 0.0
                if self.is_need_pttt == 1 and self.distance_goal > self.dis_gt:
                    self.is_pre_pttt = 1
                    self.vel_x_now = self.ptgt(4.0,self.time_start_navi,0.0, v_x)
                    v_x_send = self.vel_x_now
                    if v_x_send >= v_x:
                        self.is_need_pttt = 0
                        self.is_pre_pttt = 0
                        v_x_send = v_x
                        # print("1")

                else:
                    self.is_need_pttt = 0
                    if self.is_pre_pttt == 1 and self.vel_x_now >= 0.3:
                        v_x_send = self.vel_x_now*(self.distance_goal/self.dis_gt)
                        # print("2")

                    else:
                        self.is_pre_pttt = 0
                        v_x_send = v_x*(self.distance_goal/self.dis_gt)    
                        # print("3")
                        
                    if v_x_send > v_x:
                        v_x_send = v_x

                if self.distance_goal <= self.tol_target or self.kc_con_lai <= self.tol_target or self.is_over_goal == True :
                    #----- them xoay dap ung goc cuoi
                    self.is_target = 0
                    self.is_over_goal = False
                    self.is_need_pttt = 0
                    self.is_pre_pttt = 0
                    rospy.loginfo('da den goal cuoi roi roi!!!!!!')
                    self.stop()
                    self.completed_simple = 1
                    self.status = 3
                    time.sleep(0.6)

                vel_x = 0.0
                self.war_agv = 0
                vel_x = v_x_send
                    
                if vel_x >= self.vel_x_max:
                    vel_x = self.vel_x_max

                if vel_x <= self.min_vel_x:
                    vel_x = self.min_vel_x

                    
                else:
                    self.war_agv = 0
                    vel_x = v_x_send

                if vel_x >= self.vel_x_max:
                    vel_x = self.vel_x_max

                if vel_x <= self.min_vel_x:
                    vel_x = self.min_vel_x

                # v_th_send = 0.0

                # if self.distance_goal <= 0.02:
                #     self.is_target = 0
                # else:
                #     self.is_target = 1
                    
                # if self.is_target == 0:     #Nếu đã đến đích 
                #     v_th_send = self.control_navigation(self.x_td_goal, self.y_td_goal,vel_x,self.theta, self.dis_hc) 
                #     self.status = 3
                # else:
                #     v_th_send = self.control_naviTarget(self.x_td_goal, self.y_td_goal)                                                                                                                
                
                v_th_send = 0.0
                v_th_send = self.pid.calculate(self.distance_hc)
                
                twist = Twist()
                twist.linear.x = vel_x
                twist.angular.z = v_th_send
                self.pub_cmdVel(twist,self.rate_cmdVel)                    
                    
            elif self.status == 3:
                self.theta = self.theta_rb_ht - self.target_z

                twist = Twist()
                gt = self.turn_ar(self.theta, self.tolerance_rot_step1, self.vel_rot_step1)
                print("theta", self.theta)
                # print(gt)
                if gt == -10:
                    print("aaaaaaaaa")
                    self.stop()
                    rospy.sleep(0.3)
                    self.status = 4
                    self.XRobotStart = self.poseRbMa.position.x
                    self.YRobotStart = self.poseRbMa.position.y

                else:
                    twist = Twist()
                    twist.angular.z = gt
                    self.pub_cmdVel(twist, self.rate_cmdVel)
                
            elif self.status == 4:
                print("lui vao lay ke")
                s_nav = self.distanceAB(self.poseRbMa.position.x, self.poseRbMa.position.y, self.XRobotStart, self.YRobotStart)
                print("s_nav", s_nav)
                print("backward", self.kc_backward)
                if s_nav < fabs(self.kc_backward):
                    vel_lui = (fabs((fabs(self.kc_backward)- s_nav))/self.dis_gt_khilui)*0.16
                    if vel_lui > 0.16:
                        vel_lui = 0.16 
                    if vel_lui < 0.1:
                        vel_lui = 0.1
                    twist.linear.x = -vel_lui
                    self.pub_cmdVel(twist, self.rate_cmdVel)
                else: 
                    self.stop()
                    self.status = 5

            elif self.status == 5:
                print("Completed")
            
            
            elif self.status == 999:
                self.point_goal_start_x = self.poseRbMa.position.x
                self.point_goal_start_y = self.poseRbMa.position.y
                self.theta = self.Cal_angle(self.poseRbMa.position.x, self.poseRbMa.position.y, self.targetx, self.targety, self.theta_rb_ht)

                self.status = 1000
                
            elif self.status == 1000:
                P0 = [self.point_goal_start_x, self.point_goal_start_y]
                P3 = [self.targetx, self.targety]
                print("P000000000000:", P0)
                print("P333333333333:", P3)
                P1, P2 = self.bezier_curve.caculateP1P2(P0, P3, self.theta)
                
                print("P111111111111: ", P1)
                print("P222222222222: ", P2)
                points = [self.bezier_curve.bezier_curve(t/100.0, P0, P1, P2, P3) for t in range(101)]
                self.distance_goal = self.distanceAB(self.poseRbMa.position.x, self.poseRbMa.position.y, self.targetx, self.targety)
                print("distance goal: ", self.distance_goal)
                if self.distance_goal > self.tol_target:
                    linear_vel, angular_vel = self.bezier_curve.calculate_velocity_and_angle(points)
                    # print("pointtttttttttttttttt", points)
                    twist = Twist()
                    twist.linear.x = linear_vel
                    twist.angular.z = angular_vel
                    self.pub_cmdVel(twist, self.rate_cmdVel)
                    print("linear x", linear_vel)
                    print("angular z", angular_vel)
                else: 
                    #----- them xoay dap ung goc cuoi
                    self.is_target = 0
                    self.is_over_goal = False
                    self.is_need_pttt = 0
                    self.is_pre_pttt = 0
                    rospy.loginfo('da den goal cuoi roi roi!!!!!!')
                    self.stop()
                    self.completed_simple = 1
                    self.status = 3
                    time.sleep(0.6)

            self.rate.sleep()

def main():

    program = goalControl()
    program.run()
        #pass

if __name__ == '__main__':
    main()
