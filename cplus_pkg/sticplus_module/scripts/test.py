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

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees, acos, fabs

class TEST():
    def __init__(self):
        print("ROS Initial: navigation!")
        rospy.init_node('rickshaw_navigation', anonymous = False) # False
        self.rate = rospy.Rate(30)
        rospy.Subscriber('/robotPose_nav', PoseStamped, self.getPose, queue_size = 20)

        self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size = 60)

        self.poseRbMa = Pose()
        self.is_data_robot = False

        self.step = 0
        
    def getPose(self, data):
        self.is_data_robot = True
        self.poseRbMa = data.pose

    def calculate_distance(self,p1, p2): # p1, p2 | geometry_msgs/Point
        x = p2.x - p1.x
        y = p2.y - p1.y
        return sqrt(x*x + y*y)

    def angleLine_AB(self, pointA, pointB): # -- Angle Line Point A to Point B. | Point()
        a = pointA.y - pointB.y
        b = pointB.x - pointA.x
        ang = 0.0
        if b == 0:
            if a < 0:
                ang = pi/2.0
            elif a > 0:
                ang = -pi/2.0
        elif a == 0:
            if -b < 0:
                ang = 0.0
            elif -b > 0:
                ang = pi
        else:
            ang = acos(b/sqrt(b*b + a*a))
            if -a/b > 0:
                if fabs(ang) > pi/2:
                    ang = -ang
                else:
                    ang = ang
            else:
                if fabs(ang) > pi/2:
                    ang = ang
                else:
                    ang = -ang

        return ang

    def quaternion_to_euler(self, qua):
        quat = (qua.x, qua.y, qua.z, qua.w )
        a, b, euler = euler_from_quaternion(quat)
        return euler

    def limitAngle(self, angle_in): # - rad
        qua_in = self.euler_to_quaternion(angle_in)
        angle_out = self.quaternion_to_euler(qua_in)
        return angle_out

    def euler_to_quaternion(self, euler):
        quat = Quaternion()
        odom_quat = quaternion_from_euler(0, 0, euler)
        quat.x = odom_quat[0]
        quat.y = odom_quat[1]
        quat.z = odom_quat[2]
        quat.w = odom_quat[3]
        return quat

    def constrain(self, value_in, value_min, value_max):
        value_out = 0.0
        if value_in < value_min:
            value_out = value_min
        elif value_in > value_max:
            value_out = value_max
        else:
            value_out = value_in
        return value_out

    def fakePose_robotFollowGoal(self, robotPose, goalPose): # -- goal -> main.
        distancePoint = self.calculate_distance(robotPose.position, goalPose.position)
        # --
        angle_goalToRobot = self.angleLine_AB(goalPose.position, robotPose.position)
        # angle_robotToGoal = self.angleLine_AB(robotPose.position, goalPose.position)
        # --
        angleGoal = self.quaternion_to_euler(goalPose.orientation)
        angleRobot = self.quaternion_to_euler(robotPose.orientation)
        # -- 
        # print ("angleRobot: ", degrees(angleRobot))
        # print ("angleGoal: ", degrees(angleGoal))
        # print ("angle_goalToRobot: ", degrees(angle_goalToRobot))
        # print ("angle_robotToGoal: ", degrees(angle_robotToGoal))
        # -- 
        angleNew_goalToRobot = angle_goalToRobot - angleGoal
        angleNew_goalToRobot = self.limitAngle(angleNew_goalToRobot)
        # -- 
        # print ("angleNew_goalToRobot: ", degrees(angleNew_goalToRobot) )
        # -- 
        angleRobot_new = angleRobot - angleGoal
        angleRobot_new = self.limitAngle(angleRobot_new)
        # -- 
        poseRobot_new = Pose()
        poseRobot_new.position.x = cos(angleNew_goalToRobot)*distancePoint
        poseRobot_new.position.y = sin(angleNew_goalToRobot)*distancePoint
        poseRobot_new.orientation = self.euler_to_quaternion(angleRobot_new)
        # print ("poseRobot_new X: ", poseRobot_new.position.x)
        # print ("poseRobot_new Y: ", poseRobot_new.position.y)
        # print ("poseRobot_new R: ", degrees(angleRobot_new) )
        # print (str(round(poseGoal_new.position.x, 3)) + " | " + str(round(poseGoal_new.position.y, 3)))

        poseGoal_new = Pose()
        poseGoal_new.orientation = self.euler_to_quaternion(0)

        return poseRobot_new, poseGoal_new


    def fakePose_robotFollowGoalVS2(self, robotPose, goalPose):
        angleGoal = self.quaternion_to_euler(goalPose.orientation)
        angleRobot = self.quaternion_to_euler(robotPose.orientation)
        angleRobot_new = angleRobot - angleGoal
        angleRobot_new = self.limitAngle(angleRobot_new)

        poseRobot_new = Pose()
        poseRobot_new.position.x = (robotPose.position.x - goalPose.position.x)*cos(-angleGoal) - (robotPose.position.y - goalPose.position.y)*sin(-angleGoal)
        poseRobot_new.position.y = (robotPose.position.x - goalPose.position.x)*sin(-angleGoal) + (robotPose.position.y - goalPose.position.y)*cos(-angleGoal)
        poseRobot_new.orientation = self.euler_to_quaternion(angleRobot_new)

        poseGoal_new = Pose()
        poseGoal_new.orientation = self.euler_to_quaternion(0)

        return poseRobot_new, poseGoal_new

    def move(self, goalSimple, robotPose):
        twist = Twist()
        pose_extraGoal = Pose()
        poseGoal = goalSimple
        # poseGoal.position = goalSimple.position
        # poseGoal.orientation = self.euler_to_quaternion(goalSimple.angleLine)
        # -- Chuyển đổi hệ tọa độ.
        poseRobot_fake, poseGoal_fake = self.fakePose_robotFollowGoal(robotPose, poseGoal)

        # --------- Phát hiện lỗi --------- #
        angle_robotFake = self.quaternion_to_euler(poseRobot_fake.orientation)
        pose_extraGoal.orientation = poseGoal_fake.orientation

        # -- Kiểm tra Khoảng cách lệch an toàn.
        # if abs(poseRobot_fake.position.y) > (goalSimple.roadWidth/2.):
        #     self.flag_errorSoFar_distance = 1
        # else:
        #     self.flag_errorSoFar_distance = 0

        # -- Kiểm tra Góc lệch an toàn.
        # if abs(angle_robotFake) < (goalSimple.deltaAngle/2.):
        #     self.flag_errorSoFar_angle = 0
        # else:
        #     self.flag_errorSoFar_angle = 1
        
        # - Tạm thời loại bỏ lỗi.
        # self.flag_errorSoFar_angle = 0
        # self.flag_errorSoFar_distance = 0

        # -- Phát hiện lỗi
        # if self.step_moveSpecial == 0:
        #     if self.flag_errorSoFar_distance != 0 or self.flag_errorSoFar_angle != 0:
        #         if self.flag_errorSoFar_distance != 0 and self.flag_errorSoFar_angle == 0:
        #             sts_run = -1

        #         elif self.flag_errorSoFar_distance == 0 and self.flag_errorSoFar_angle != 0:
        #             sts_run = -2
        #         else:
        #             sts_run = -3
        #     else:
        #         self.step_moveSpecial = 1

            # - Xác định hướng di chuyển. (Đi đầu, đi đuôi).
        if self.step == 0:
            self.step = 1
            if poseRobot_fake.position.x < 0: # - 
                self.moveBy = 1
            else:
                self.moveBy = 2

        if self.step == 1:
            if self.moveBy == 1:
                # -- Hien thi kieu di chuyen.
                print("----- MoveSpecial Ahead: Spining AngleFirst -----")
                # -
                pose_extraGoal.position.x = poseRobot_fake.position.x + 0.4
                angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
                # -
                angle_robot_ahead = self.quaternion_to_euler(poseRobot_fake.orientation)
                delta_angle = angle_robotToGoal - angle_robot_ahead

                # -- Xác nhận hoàn thành.
                delta_dis = self.calculate_distance(poseRobot_fake.position, Point() )
                if abs(delta_dis) < 0.01 or poseRobot_fake.position.x > 0.01:
                # if poseRobot_fake.position.x > accuracy_distance:
                    self.step = 3 #
                    self.pub_cmdVel.publish(Twist())
                    print("----- MoveSpecial Ahead F: Go Completed -----")

                # print ("Ahead Delta: " + str(round(degrees(delta_angle), 2)) + " Dis_x: " + str(round(poseRobot_fake.position.x, 3)) + " Dis_y: " + str(round(poseRobot_fake.position.y, 3)) + " Dis: " + str(round(delta_dis, 3) ) )

            else:
                # -- Hien thi kieu di chuyen.
                # --
                print("----- MoveSpecial Tail: Spining AngleFirst -----")
                # -
                pose_extraGoal.position.x = poseRobot_fake.position.x - 0.4
                angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
                # -
                angle_robot_tail = self.quaternion_to_euler(poseRobot_fake.orientation) + pi
                delta_angle = angle_robotToGoal - angle_robot_tail

                # -- Xác nhận hoàn thành.
                delta_dis = self.calculate_distance(poseRobot_fake.position, Point() )
                if abs(delta_dis) < 0.01 or poseRobot_fake.position.x < 0.01:
                # if poseRobot_fake.position.x < accuracy_distance:
                    self.step = 3 #
                    self.pub_cmdVel.publish(Twist())
                    print("----- MoveSpecial Tail F: Go Completed -----")
                # print ("Ahead Delta: " + str(round(degrees(delta_angle), 2)) + " Dis_x: " + str(round(poseRobot_fake.position.x, 3)) + " Dis_y: " + str(round(poseRobot_fake.position.y, 3)) + " Dis: " + str(round(delta_dis, 3)) )
            # -
            delta_angle = self.limitAngle(delta_angle)
            twist.linear.x = 0
            if abs(delta_angle) > radians(1):
                coefficient_angle = delta_angle/radians(32)
                coefficient_angle = self.constrain(coefficient_angle, -1, 1)
                twist.angular.z = coefficient_angle*0.3 # 0.4

                if abs(twist.angular.z) < 0.01 and twist.angular.z != 0:
                    if twist.angular.z > 0:
                        twist.angular.z = 0.01
                    else:
                        twist.angular.z = -0.01

                self.pub_cmdVel.publish(twist)
            else:
                twist.angular.z = 0.0
                self.pub_cmdVel.publish(twist)
                self.step = 2

        if self.step == 2:
            if self.moveBy == 1:
                # -- Hien thi kieu di chuyen.
                # --
                print("----- MoveSpecial Ahead: Go Strange -----")
                distance_offset = 0.6 # 0.4
                if abs(poseRobot_fake.position.y) > distance_offset:
                    dif_x = 0.0
                else:
                    dif_x = distance_offset - pow(poseRobot_fake.position.y, 2)
                # -
                pose_extraGoal.position.x = poseRobot_fake.position.x + dif_x
                pose_extraGoal.position.y = 0
                delta_distance = self.calculate_distance(poseRobot_fake.position, pose_extraGoal.position)
                # --
                deltaDistance_real = self.calculate_distance(poseRobot_fake.position, Point())
                
                # -- 
                angle_robot_ahead = self.quaternion_to_euler(poseRobot_fake.orientation)
                angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
                delta_angle = angle_robotToGoal - angle_robot_ahead
                delta_angle = self.limitAngle(delta_angle)
                # -
                speedMax_linear = 0.2
                if abs(deltaDistance_real) < 0.2:
                    # print ("deltaDistance_real: ", deltaDistance_real)
                    speedMax_linear = 0.06
                # -- X
                coefficient_distance = abs(delta_distance)/speedMax_linear
                coefficient_distance = self.constrain(coefficient_distance, 0, 1)
                twist.linear.x = coefficient_distance*speedMax_linear
                twist.linear.x = self.constrain(twist.linear.x, 0.02, 0.3)
                # -- R
                if abs(delta_angle) > radians(0.4): # do lech du lon thi cho phep quay.
                    coefficient_angle = delta_angle/radians(40) # self.deceleration_angle
                    coefficient_angle = self.constrain(coefficient_angle, -1, 1)
                    twist.angular.z = coefficient_angle*0.3
                else:
                    twist.angular.z = 0.0

                # --
                # self.string_debug1 = 'Rx: '+ str(round(poseRobot_fake.position.x, 3)) + ' |Ry: ' + str(round(poseRobot_fake.position.y, 3)) + ' |Gx: ' + str(round(pose_extraGoal.position.x, 3))
                # self.string_debug2 = 'RG: '+ str(round(angle_robotToGoal, 2)) + ' |R: ' + str(round(angle_robot_ahead, 2)) + ' |D: ' + str(round(delta_angle, 2)) + ' |V: ' + str(round(vel_level.r, 3))

                # - Bat vat can
                # -- Góc lệch lớn quá. Phải dừng lại. Xoay lại góc.
                if abs(delta_angle) > radians(80):
                    self.step = 1
                    self.pub_cmdVel.publish(Twist())
                    print("----- MoveSpecail Ahead: Delta_Angle larg -> Spining ----- Dis: " + str(abs(delta_distance)))
                    
                # -- Xác nhận hoàn thành.
                # if poseRobot_fake.position.x > accuracy_distance:
                if poseRobot_fake.position.x > 0.01:
                    self.step = 3 #
                    self.pub_cmdVel.publish(Twist())
                    print("----- MoveSpecial Ahead: Completed -----", 4)
                    return 1
                # -
                # delta_distance_2 = self.calculate_distance(robotPose.position, goalSimple.pose.position)
                # -- Khoảng cách gần tới đích.
                # if abs(delta_distance_2) < distance_near:
                #     is_nearGoal = 1

                self.pub_cmdVel.publish(twist)

            else:
                # -- Hien thi kieu di chuyen.
                # --
                print("----- MoveSpecial Tail: Go Strange -----")
                distance_offset = 0.6 # 0.4
                if abs(poseRobot_fake.position.y) > distance_offset:
                    dif_x = 0.0
                else:
                    dif_x = distance_offset - pow(poseRobot_fake.position.y, 2)
                # -
                pose_extraGoal.position.x = poseRobot_fake.position.x - dif_x
                pose_extraGoal.position.y = 0
                delta_distance = self.calculate_distance(poseRobot_fake.position, pose_extraGoal.position)
                # --
                deltaDistance_real = self.calculate_distance(poseRobot_fake.position, Point())
                # -- 
                angle_robot_tail = self.quaternion_to_euler(poseRobot_fake.orientation) + pi
                angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
                delta_angle = angle_robotToGoal - angle_robot_tail
                delta_angle = self.limitAngle(delta_angle)
                # -
                speedMax_linear = 0.2
                if abs(deltaDistance_real) < 0.2:
                    speedMax_linear = 0.06
                # -- X
                coefficient_distance = abs(delta_distance)/speedMax_linear
                coefficient_distance = self.constrain(coefficient_distance, 0, 1)
                twist.linear.x = coefficient_distance*speedMax_linear*-1
                twist.linear.x = self.constrain(twist.linear.x, 0.3*-1, 0.02*-1)
                # -- R
                if abs(delta_angle) > radians(0.4): # do lech du lon thi cho phep quay.
                    coefficient_angle = delta_angle/radians(30) # self.deceleration_angle
                    coefficient_angle = self.constrain(coefficient_angle, -1, 1)
                    twist.angular.z = coefficient_angle*0.3 # 0.4
                else:
                    twist.angular.z = 0.0

                # --
                # self.string_debug1 = 'Rx: '+ str(round(poseRobot_fake.position.x, 3)) + ' |Ry: ' + str(round(poseRobot_fake.position.y, 3)) + ' |Gx: ' + str(round(pose_extraGoal.position.x, 3))
                # self.string_debug2 = 'RG: '+ str(round(angle_robotToGoal, 2)) + ' |R: ' + str(round(angle_robot_tail, 2)) + ' |D: ' + str(round(delta_angle, 2)) + ' |V: ' + str(round(vel_level.r, 3))

                # - Bat vat can
                # if self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
                #     vel_level = Velocities()
                #     # print ("VAT CAN LUI")
                #     stopRun = 1

                # -- Góc lệch lớn quá. Phải dừng lại. Xoay lại góc.
                if abs(delta_angle) > radians(80):
                    self.step = 1
                    self.pub_cmdVel.publish(Twist())
                    # self.show_message_smart("----- MoveSpecail Tail: Delta_Angle larg -> Spining ----- Dis: " + str(abs(delta_distance)), 4)

                # -- Xác nhận hoàn thành.
                if poseRobot_fake.position.x < 0.01:
                    self.step = 3 #
                    self.pub_cmdVel.publish(Twist())
                    print("----- MoveSpecial Tail: Completed -----")
                    return 1
                
                self.pub_cmdVel.publish(twist)
                # print ("SPS: ", poseRobot_fake.position.x)

                # delta_distance_2 = self.calculate_distance(robotPose.position, goalSimple.pose.position)
                # -- Xác nhận gần đích. -> Để chuyển sang điểm tiếp theo.
                # if abs(delta_distance_2) < distance_near:
                #     is_nearGoal = 1
            
        return 0
    
    def anotherMove(self, goalSimple, robotPose):
        twist = Twist()
        pose_extraGoal = Pose()
        poseGoal = goalSimple
        # poseGoal.position = goalSimple.position
        # poseGoal.orientation = self.euler_to_quaternion(goalSimple.angleLine)
        # -- Chuyển đổi hệ tọa độ.
        print("[----------------------------------1-------------------------------------]")
        poseRobot_fake, poseGoal_fake = self.fakePose_robotFollowGoal(robotPose, poseGoal)
        print(poseRobot_fake)
        print("[-----------------------------------2------------------------------------]")
        poseRobot_fake, poseGoal_fake = self.fakePose_robotFollowGoalVS2(robotPose, poseGoal)
        print(poseRobot_fake)

        return 0, 0

        # --------- Phát hiện lỗi --------- #
        angle_robotFake = self.quaternion_to_euler(poseRobot_fake.orientation)
        pose_extraGoal.orientation = poseGoal_fake.orientation
        print(poseRobot_fake, poseGoal_fake)

        self.step = 20

        if self.step == 0:
            self.step = 1
            if poseRobot_fake.position.x < 0: # - 
                self.moveBy = 1
            else:
                self.moveBy = 2

        if self.step == 1:
            if self.moveBy == 1:
                # -- Hien thi kieu di chuyen.
                # self.navigationRespond.typeRun = 21
                # --
                print("----- MoveSpecial Target Ahead: Spining AngleFirst -----")
                # -
                pose_extraGoal.position.x = poseRobot_fake.position.x + 0.4
                angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
                # -
                angle_robot_ahead = self.quaternion_to_euler(poseRobot_fake.orientation)
                delta_angle = angle_robotToGoal - angle_robot_ahead
                
                # -- Xác nhận hoàn thành.
                # if poseRobot_fake.position.x > 0.01:
                #     # self.pub_cmdVel.publish(Twist())
                #     twist = Twist()
                #     print("----- MoveSpecial Target Ahead F: Go Completed -----")
                #     self.step = 2
                    # print ("Delta: " + str(degrees(delta_angle)) + "Dis_x: " + str(poseRobot_fake.position.x) + "Dis_y: " + str(poseRobot_fake.position.y) )
            else:
                # -- Hien thi kieu di chuyen.
                # self.navigationRespond.typeRun = -21
                # --
                print("----- MoveSpecial Target Tail: Spining AngleFirst -----")
                # -
                pose_extraGoal.position.x = poseRobot_fake.position.x - 0.4
                angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
                # -
                angle_robot_tail = self.quaternion_to_euler(poseRobot_fake.orientation) + pi
                delta_angle = angle_robotToGoal - angle_robot_tail
                
                # -- Xác nhận hoàn thành.
                # if poseRobot_fake.position.x < 0.01:
                #     # self.pub_cmdVel.publish(Twist())
                #     twist = Twist()
                #     print("----- MoveSpecial Target Tail F: Go Completed -----")
                #     self.step = 2
                    # print ("Delta: " + str(degrees(delta_angle)) + "Dis_x: " + str(poseRobot_fake.position.x) + "Dis_y: " + str(poseRobot_fake.position.y) )
            # -
            delta_angle = self.limitAngle(delta_angle)
            twist.linear.x = 0.
            if abs(delta_angle) > radians(1):
                coefficient_angle = delta_angle/radians(40)
                coefficient_angle = self.constrain(coefficient_angle, -1, 1)
                twist.angular.z = coefficient_angle*0.3 
                # -
                if abs(twist.angular.z) < 0.01 and twist.angular.z != 0:
                    if twist.angular.z > 0:
                        twist.angular.z = 0.01
                    else:
                        twist.angular.z = -0.01
                self.pub_cmdVel.publish(twist)

            else:
                twist.angular.z = 0.0
                # self.pub_cmdVel.publish(twist)
                self.step = 2

            print(delta_angle, twist.angular.z, poseRobot_fake.position.x, self.moveBy)
            
        return twist, self.step

    def run(self):
        step = 0
        pose = Pose()
        pose.position.x = 7.822
        pose.position.y = 6.09
        pose.orientation.x = 0.
        pose.orientation.y = 0.
        pose.orientation.z = 0.0044
        pose.orientation.w = 0.9999
        while not rospy.is_shutdown():
            if step == 0:
                if self.is_data_robot:
                    # if self.move(pose, self.poseRbMa):
                    #     print("DONE")
                    #     step = 10
                    twist, st = self.anotherMove(pose, self.poseRbMa)
                    # self.pub_cmdVel.publish(twist)

            self.rate.sleep()

def main():
    a = TEST()
    a.run()
        
if __name__ == '__main__':
    main()