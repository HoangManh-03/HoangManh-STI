#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dev: Archie Phung
Date Modify: 16/8/2024

Advantage:
  From tf coef -> position tolerance < 1 cm

Function:
  - Spin AGV to precicise angle with odometry data
  - Exec command from keyboard
  - Intergrate with remote function of AGV

"""

import rospy
from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos, degrees

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_pkg.msg import *
from sti_msgs.msg import *
from ros_canBus.msg import *

class GetTFCoef():
    def __init__(self):
        print("ROS Initial: GetTFCoef !")
        rospy.init_node('get_tf_coef', anonymous = False, disable_signals=True) # False

        self.rate = rospy.Rate(40)

        # self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1',0.02)
        # self.vel_rot_step1 = rospy.get_param('~vel_rot_step1',0.32)      # 0.45
        self.vel_rot_step1 = 0.2

        # -- SUBSCRIBER NODE -- 
        # -- Keyboard command
        rospy.Subscriber("/Keyboard_cmd", Keyboard_command, self.callback_Keyboard) # 
        self.keyboard_cmd = Keyboard_command()	

        # -- App
        rospy.Subscriber("/app_button", App_button, self.callback_appButton) # lay thong tin trang thai nut nhan tren man hinh HMI.
        self.app_button = App_button()

        # -- Safety zone
        rospy.Subscriber("/HC_info", HC_info, self.zone_callback)
        self.zone_lidar = HC_info()

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.getPose, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.theta_rb_ht = 0.0

        rospy.Subscriber('/nav350laser/odom', Odometry, self.getPose_Odometry, queue_size = 20)
        self.poseRbMa_odom = Pose()
        self.theta_rb_odom = 0.0

        # -- PUBLISH TOPIC -- 
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # -- CONST --
        self.ANGLE1 = 358.20*PI/180.0
        self.ANGLE2 = 88.20*PI/180.0
        self.ANGLE3 = 178.20*PI/180.0
        self.ANGLE4 = 268.2*PI/180.0
        self.DELTA_ANGLE_TARGET = 0.015
        self.VEL_RB_MIN = 0.05
        
        # -- VAR --
        self.total_x = 0.0
        self.total_y = 0.0
        self.total_z = 0.0
        self.step_simple = 0
        self.number_coorSample = 0
        self.coorAverage_x = 0.0
        self.coorAverage_y = 0.0
        self.coorAverage_z = 0.0
        self.coor_deviation_x = 0.0
        self.coor_deviation_y = 0.0

        self.reference_point_x = 0
        self.reference_point_y = 0
        self.number_spin_simple = 0
        self.allow_spin_simple = 0

        self.angle_target = 0
        self.pre_angle_target = 0
        self.angle_giam_toc = 45.0*PI/180.0

        self.flag_reset = 0
        self.flag_restart = 0
        self.flag_spin = 0
        self.flag_stop = 0
        self.flag_get_coor_simple = 0
        self.flag_auto = 0
        self.is_exit = 0

        self.rate_cmdvel = 30
        self.time_tr = rospy.get_time()

    ############################################ CALLBACK FUNCTION ########################################################################3
    def callback_Keyboard(self, data):
        self.keyboard_cmd = data
        if self.keyboard_cmd.value == "":
            self.flag_reset = 0	

        elif self.keyboard_cmd.value == "spin0":     # spin forward
            self.flag_spin = 1
            self.angle_target = self.ANGLE1

        elif self.keyboard_cmd.value == "spin90":
            self.flag_spin = 1
            self.angle_target = self.ANGLE2

        elif self.keyboard_cmd.value == "spin180":
            self.flag_spin = 1
            self.angle_target = self.ANGLE3

        elif self.keyboard_cmd.value == "spin270":
            self.flag_spin = 1
            self.angle_target = self.ANGLE4

        elif self.keyboard_cmd.value == "get":
            self.flag_get_coor_simple = 1

        elif self.keyboard_cmd.value == "stop":
            self.flag_spin = 0
            self.flag_stop = 1

        elif self.keyboard_cmd.value == "run":
            self.flag_auto = 1

        elif self.keyboard_cmd.value == "reset":
            self.flag_reset = 1

        elif self.keyboard_cmd.value == "restart":
            self.flag_restart = 1

    def callback_appButton(self, dat):
        self.app_button = dat

    def zone_callback(self, data):
        self.zone_lidar = data

    def getPose(self, data):
        self.is_pose_robot = True
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb_ht = euler[2]
        if self.theta_rb_ht < 0:
            self.theta_rb_ht = self.theta_rb_ht + 2*PI

    def getPose_Odometry(self, data):
        self.poseRbMa_odom = data.pose.pose
        quata = ( self.poseRbMa_odom.orientation.x,\
                self.poseRbMa_odom.orientation.y,\
                self.poseRbMa_odom.orientation.z,\
                self.poseRbMa_odom.orientation.w )
        euler = euler_from_quaternion(quata)

        self.theta_rb_odom = euler[2]
        if self.theta_rb_odom <= 0:
            self.theta_rb_odom = self.theta_rb_odom + 2*PI
        
        # print(round(degrees(self.theta_rb_odom)),3)

        # # - Lấy tọa độ trung bình
        # if self.step_simple == 1:
        #     self.total_x += self.poseRbMa.position.x
        #     self.total_y += self.poseRbMa.position.y
        #     self.number_coorSample += 1
        #     if self.number_coorSample == 50:
        #         self.step_simple = 0
        #         self.coorAverage_x = self.total_x/ self.number_coorSample
        #         self.coorAverage_y = self.total_y/ self.number_coorSample
        #         self.total_x = 0
        #         self.total_y = 0

    ############################################ DEF FUNCTION ########################################################################3
    def pub_cmdVel(self, twist , rate):

        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass
            
    def log_mess(self, mess):
        if self.enb_log == 1:
            if self.step_log == 0:
                self.ct_log = rospy.get_time()
                self.step_log = 1
                print(mess)
            else:
                if rospy.get_time() - self.ct_log >= self.TIME_LOG:
                    print(mess)
                    self.ct_log = rospy.get_time()
        else:
            self.step_log = 0

    def turn_ar(self, theta, target_theta, vel_rot):
        delta_theta = theta - target_theta
        # print("Góc và độ lệch hiện tại là: {x}, {y}".format(x=round(degrees(theta), 3), y = round(degrees(delta_theta), 3)))

        if -self.DELTA_ANGLE_TARGET/2 > delta_theta or delta_theta > self.DELTA_ANGLE_TARGET/2: # +- 10 do
            if target_theta >= 0: #quay trai
                # print "b"
                if fabs(delta_theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhh')
                    vel_th = (fabs(delta_theta)/self.angle_giam_toc)*vel_rot
                else:
                    vel_th = vel_rot

                if vel_th < self.VEL_RB_MIN:
                    vel_th = self.VEL_RB_MIN

                # vel_th = fabs(theta) + 0.1
                # if vel_th > vel_rot : vel_th = vel_rot
                return vel_th

            elif target_theta < 0: #quay phai , vel_z < 0
                # print "a"
                if fabs(delta_theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhhh')
                    vel_th = (fabs(delta_theta)/self.angle_giam_toc)*(-vel_rot)
                else:
                    vel_th = -vel_rot

                if vel_th > -self.VEL_RB_MIN:
                    vel_th = -self.VEL_RB_MIN

                # vel_th = -fabs(theta) - 0.1
                # if vel_th < -vel_rot : vel_th = -vel_rot
                return vel_th
                # buoc = 1

        else : 
            return -10

    def stop(self):
        for i in range(2):
            self.pub_cmd_vel.publish(Twist())

    def reset(self):
        pass

    def restart(self):
        pass	

    def shutdown(self):
        self.is_exit = 1
        
    ############################################ LOOP FUNCTION ########################################################################3
    def run(self):
        try:
            if self.is_exit == 0:
                while not rospy.is_shutdown():
                    if self.app_button.bt_remote == 1:
                        if self.flag_restart == 1:
                            self.restart()
                        
                        if self.flag_reset == 1:
                            self.number_spin_simple = 0
                            print("############################## Reset hệ thống! ##################################################")
                            self.flag_reset = 0

                        if self.flag_spin == 1:
                            if self.number_spin_simple == 0:
                                # if self.zone_lidar.zone_sick_ahead == 1 or self.zone_lidar.zone_sick_behind != 0:
                                #     self.war_agv = 1
                                #     rospy.logwarn('co vat can o vung tron truoc sau')
                                #     self.stop()
                                # else:
                                gt = self.turn_ar(self.theta_rb_odom, self.angle_target, self.vel_rot_step1)
                                if gt == -10:
                                    self.stop()
                                    rospy.sleep(0.3)
                                    self.flag_spin = 0
                                    self.step_simple = 1
                                    self.number_spin_simple += 1
                                    self.pre_angle_target = self.angle_target
                                    print("AGV đã tới góc đích")

                                else:
                                    self.step_simple = 0
                                    twist = Twist()
                                    twist.angular.z = gt
                                    self.pub_cmdVel(twist, self.rate_cmdvel)

                            else:
                                if self.pre_angle_target != self.angle_target:
                                    # if self.zone_lidar.zone_sick_ahead == 1 or self.zone_lidar.zone_sick_behind != 0:
                                    #     self.war_agv = 1
                                    #     rospy.logwarn('co vat can o vung tron truoc sau')
                                    #     self.stop()
                                    # else:
                                    gt = self.turn_ar(self.theta_rb_odom, self.angle_target, self.vel_rot_step1)
                                    if gt == -10:
                                        self.stop()
                                        rospy.sleep(0.3)
                                        self.flag_spin = 0
                                        self.step_simple = 1
                                        self.number_spin_simple += 1
                                        self.pre_angle_target = self.angle_target
                                        print("AGV đã tới góc đích")

                                    else:
                                        self.step_simple = 0
                                        twist = Twist()
                                        twist.angular.z = gt
                                        self.pub_cmdVel(twist, self.rate_cmdvel)
                                else:
                                    if self.enb_log == 1:
                                        rospy.logwarn("AGV đã tới góc đích. Vui lòng đổi góc khác")
                                        self.enb_log = 0

                        else:
                            self.enb_log = 1
                            if self.flag_get_coor_simple == 1:
                                # - Lấy tọa độ trung bình
                                if self.step_simple == 1:
                                    self.total_x += self.poseRbMa.position.x
                                    self.total_y += self.poseRbMa.position.y
                                    self.total_z += self.theta_rb_ht
                                    self.number_coorSample += 1
                                    if self.number_coorSample == 50:
                                        self.step_simple = 2
                                        self.coorAverage_x = self.total_x/ self.number_coorSample
                                        self.coorAverage_y = self.total_y/ self.number_coorSample
                                        self.coorAverage_z = self.total_z/ self.number_coorSample

                                        print("Tọa độ AGV odom ở góc {z} là: {x}, {y}, {r}".format(z = round(degrees(self.angle_target)), x = self.poseRbMa_odom.position.x, y = self.poseRbMa_odom.position.y, r = round(degrees(self.theta_rb_odom), 3)))
                                        print("Tọa độ AGV từ lidar ở góc {z} là: {x}, {y}, {r}".format(z = round(degrees(self.angle_target)), x = self.coorAverage_x, y = self.coorAverage_y, r = round(degrees(self.coorAverage_z), 3)))
                                        self.total_x = 0
                                        self.total_y = 0
                                        self.total_z = 0
                                        self.number_coorSample = 0

                                        if self.number_spin_simple == 1:
                                            self.reference_point_x = self.coorAverage_x
                                            self.reference_point_y = self.coorAverage_y
                                            print("Tọa độ AGV tham chiếu là: {x}, {y}".format(x = self.reference_point_x, y = self.reference_point_y))
                                            print("--------------------------------------------------------------")
                                            
                                # - Hiển thị sai số giữa các hướng        
                                elif self.step_simple == 2:
                                    if self.number_spin_simple > 1:
                                        self.coor_deviation_x = self.coorAverage_x - self.reference_point_x
                                        self.coor_deviation_y = self.coorAverage_y - self.reference_point_y
                                        print("Sai số tọa độ AGV với điểm tham chiếu sau lần {d} là: {x}, {y}".format(d = self.number_spin_simple, x = self.coor_deviation_x, y = self.coor_deviation_y))
                                        print("--------------------------------------------------------------")
                                    
                                    self.step_simple = 0
                                    self.flag_get_coor_simple = 0

                                else:
                                    self.flag_get_coor_simple = 0
                            
                            else:
                                self.stop()

                    self.rate.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(self.shutdown)
            self.is_exit = 1
            print('!!FINISH!!')

def main():
	print('Starting main program')
	program = GetTFCoef()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()
    