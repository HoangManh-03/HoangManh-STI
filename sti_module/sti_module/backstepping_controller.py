#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import json
import sys
import signal

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from decimal import *
from math import degrees, radians

from message_pkg.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

class LineFollowerBackstepping(LifecycleNode):
    def __init__(self):
        super().__init__('line_controller')
        self.get_logger().warning("ROS 2 Node Line Controller Initialized!")
        self.killnode = 0

        # -- Publisher 
        # -- cmd_vel
        self.pub_cmdVel = self.create_publisher(Twist, 'cmd_vel', qos_profile = QoSProfile(
                depth = 10,
                reliability = QoSReliabilityPolicy.RELIABLE,    # BEST_EFFORT: sensor | RELIABLE: control
                durability  = QoSDurabilityPolicy.VOLATILE     # TRANSIENT_LOCAL: latch | VOLATILE: no latch
            ))
        self.rate_cmdvel = 20. 
        self.saveTime_pubVel = time.time()

        # -- line controller respond
        self.pub_respond_controller = self.create_publisher(LineControllerRespond, '/line_controller_respond', 10)
        self.pub_respond = LineControllerRespond()

        # -- Sub
        '''
            enable: 0
            direction_move: 0
            velocity: 0.0
        '''
        self.sub_controller = self.create_subscription(
            LineControllerRequest,
            "line_controller_request",
            self.callBack_lineCtl,
            10)
        self.sub_controller

        self.is_lineCtl = False
        self.data_lineCtlRequest = LineControllerRequest()

        # -- front magline
        self.sub_maglineFront = self.create_subscription(
            MagneticLine,
            "magneticLine_front",
            self.callBack_maglineFront,
            10)
        self.sub_maglineFront

        self.is_magline_front = False
        self.data_mangline_front = MagneticLine()

        # -- behind magline
        self.sub_maglineBehind = self.create_subscription(
            MagneticLine,
            "magneticLine_behind",
            self.callBack_maglineBehind,
            10)
        self.sub_maglineBehind

        self.is_magline_behind = False
        self.data_mangline_behind = MagneticLine()

        # -- hc
        self.sub_hcInfo = self.create_subscription(
            HcInfo,
            "hc_info",
            self.callback_zone,
            10)
        self.sub_hcInfo

        self.zone_lidar = HcInfo()
        self.is_check_zone = False

        # -- 
        self.process = 0

        self.SET_POINT = 7.5
        self.move_byHead = 1
        self.move_byTail = 2
        
        # Hệ số điều khiển
        self.k1 = 0.01  # Hệ số Backstepping cho lỗi vị trí
        self.k2 = 0.01  # Hệ số Backstepping cho lỗi vận tốc góc
        self.v_max = 0.3  # Vận tốc tối đa
        self.lambda_v = 0.05  # Giảm tốc khi lệch line


        # -- param control
        self.savetime1 = time.time()
        self.recieve_dataLine = 0
        self.save_enable = -1

        # -- param velocity
        self.isDecelerationObstacles = 0
        self.saveVelWhenDecObs = 0.
        self.statusVel = 0
        self.curr_velocity = 0.
        self.velSt = 0.
        self.target_velNow = 0.

        # -- 
        self.vel_in_zone3 = 0.2
        self.vel_in_zone2 = 0.1
        self.vel_min = 0.1

        # -- Create timer run 
        self.rate_main = 15
        self.timer_period_main = 1/self.rate_main
        self.timer_main = self.create_timer(self.timer_period_main, self.run)

    # -- 
    def shutdown_hook(self):
        for i in range(2):
            self.pub_cmdVel.publish(Twist())

        self.get_logger().warning("Shutting down. cmd_vel will be 0")

    def on_shutdown(self, state):
        self.killnode = 1
        # self.shutdown_hook()
        self.get_logger().warning("Shutting down node by lifecycle! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    def callBack_lineCtl(self, data):
        self.data_lineCtlRequest = data
        self.is_lineCtl = True

    def callBack_maglineFront(self, data):
        self.data_mangline_front = data
        self.is_magline_front = True

    def callBack_maglineBehind(self, data):
        self.data_mangline_behind = data
        self.is_magline_behind = True

    def callback_zone(self, data):
        self.zone_lidar = data
        self.is_check_zone = True

    def stop(self):
        # reset van toc
        self.curr_velocity = 0.
        self.saveTimeVel = time.time()
        self.velSt = 0.
        self.statusVel = 0
        self.isDecelerationObstacles = 0
        self.saveVelWhenDecObs = 0.

        for i in range(3):
            self.pub_cmdVel.publish(Twist())

    def funcDecelerationByAcc(self, time_s, v_s, v_f, a):
        denlta_time_now = time.time()- time_s
        v_re = v_s + a*denlta_time_now
        if a > 0.:
            if v_re >= v_f:
                v_re = v_f
        else:
            if v_re <= v_f:
                v_re = v_f

        return v_re

    def getVeloctity(self, velCmd, _statusVel):
        if _statusVel == 0:
            return velCmd
        elif _statusVel == 1:
            return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, velCmd, 0.1)
        elif _statusVel == 2:
            return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, velCmd, -2.5)
        else:
            return 0.


    def backstepping_ctl(self, velocity):
        v = 0.
        vel = 0.
        status = 0

        value_line = -1
        value_line_s = -1
        dir_move  = self.data_lineCtlRequest.direction_move
        if dir_move == self.move_byHead:
            value_line = self.data_mangline_front.value
            value_line_s = self.data_mangline_behind.value

        elif dir_move == self.move_byTail:
            value_line = self.data_mangline_behind.value
            value_line_s = self.data_mangline_front.value
            
        error = -1.
        if value_line == 20:
            status = 1
            vel = 0.

        elif value_line > -1 and value_line < 16:
            status = 1
            self.recieve_dataLine = 1
            self.savetime1 = time.time()

            current_time = time.time()
            dt = current_time - self.last_time
            error = self.SET_POINT - value_line

            # Điều khiển Backstepping
            alpha = -self.k1 * error
            vel = -self.k1 * error - self.k2 * alpha
            
            # Điều chỉnh vận tốc tuyến tính
            v = velocity * (2.718 ** (-self.lambda_v * abs(error)))
        
        else:
            if self.recieve_dataLine:
                denta_t = time.time() - self.savetime1
                if denta_t < 0.5:
                    status = 2

        return status, v, vel, error

    def reset_pid(self):
        self.previous_error = 0.0
        self.integral = 0.0

        self.previous_error_2 = 0.0
        self.integral_2 = 0.0

        self.last_time = time.time()

        self.recieve_dataLine = 0
        self.savetime1 = time.time()

    def Pub_cmdVel(self, twist , rate):
        if 1 or time.time() - self.saveTime_pubVel > float(1/rate): # < 20hz 
            self.saveTime_pubVel = time.time()
            self.pub_cmdVel.publish(twist)

    def run(self):
        # -- kiem tra du lieu
        if self.process == 0:
            ck = 0
            if self.is_magline_front:
                ck += 1
            if self.is_magline_behind:
                ck += 1
            if self.is_check_zone:
                ck += 1
            if self.is_lineCtl:
                ck += 1

            if ck == 4:
                print("recieved all data need!")
                self.process = 1
        
        elif self.process == 1:
            if self.data_lineCtlRequest.enable != 2:
                if self.save_enable != self.data_lineCtlRequest.enable or self.data_lineCtlRequest.enable == 0:
                    print("STOP!")
                    self.stop()
                    self.save_enable = self.data_lineCtlRequest.enable

                self.reset_pid()
                self.pub_respond_controller.publish(LineControllerRespond())

            else:
                # -- safety
                safety = 0
                if self.data_lineCtlRequest.direction_move == self.move_byHead:
                    safety = self.zone_lidar.zone_sick_ahead if self.data_lineCtlRequest.safety else 0
                elif self.data_lineCtlRequest.direction_move == self.move_byTail:
                    safety = self.zone_lidar.zone_sick_behind if self.data_lineCtlRequest.safety else 0

                velocity = self.data_lineCtlRequest.velocity
                velCmd = 0. 
                if safety == 1:
                    velCmd = 0.
                    print('not safety', velCmd)
                    
                elif safety == 2:
                    if self.curr_velocity == 0. or self.curr_velocity >= self.vel_in_zone2:
                        velCmd = self.vel_in_zone2
                    else:
                        velCmd = self.curr_velocity

                    if velCmd <= self.vel_min:
                        velCmd = self.vel_min

                    print('in safety zone 2!', velCmd)

                elif safety == 3:
                    if self.curr_velocity == 0. or self.curr_velocity >= self.vel_in_zone3:
                        velCmd = self.vel_in_zone3
                    else:
                        velCmd = self.curr_velocity

                    if velCmd <= self.vel_min:
                        velCmd = self.vel_min

                    print('in safety zone 3!', velCmd)

                else:
                    velCmd = velocity

                if velCmd == 0.:
                    self.stop()
                    self.reset_pid()

                else:
                    if self.curr_velocity < velCmd and self.statusVel != 1:
                        self.statusVel = 1
                        self.velSt = self.curr_velocity
                        self.saveTimeVel = time.time()

                    elif self.curr_velocity > velCmd and self.statusVel != 2:
                        self.statusVel = 2
                        self.velSt = self.curr_velocity
                        self.saveTimeVel = time.time()

                    elif self.curr_velocity == velCmd:
                        self.statusVel = 0

                    # self.curr_velocity = self.getVeloctity(velCmd, self.statusVel)

                    self.curr_velocity = self.data_lineCtlRequest.velocity

                    try:
                        self.save_enable = 2
                        status, vx, vel, error = self.backstepping_ctl(self.curr_velocity)

                        if status == 0:
                            self.stop()
                            self.reset_pid()
                            print("agv lech khoi line tu")

                        elif status == 1:
                            twist = Twist()
                            vel_linear = vx
                            # vel_linear = vel_x
                            if self.data_lineCtlRequest.direction_move == self.move_byTail:
                                vel_linear = -1 * vel_linear

                            twist.linear.x = vel_linear
                            twist.angular.z = vel
                            self.Pub_cmdVel(twist, self.rate_cmdvel)

                            print("linear_x: %s, angular_z: %s" %(vel_linear, vel))
                        
                        # -- pub data respond
                        _dRespond = LineControllerRespond()
                        _dRespond.status = status
                        _dRespond.error_line = error

                        self.pub_respond_controller.publish(_dRespond)

                    except Exception as e:
                        print("ERROR!")

        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)


# Hàm xử lý khi nhấn Ctrl+C
def signal_handler(node):
    node.killnode = 1

def main():
    # -- Khoi tao ROS
    rclpy.init()
    node = LineFollowerBackstepping()

    # Đăng ký Ctrl+C, truyền hàm `publish_final_message` vào `signal_handler`
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(node))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')

if __name__ == '__main__':
    main()