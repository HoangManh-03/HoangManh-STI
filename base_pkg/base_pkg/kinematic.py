import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

from message_pkg.msg import *
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from std_msgs.msg import Int16

from math import sin , cos , pi , atan2

class RPM:
	motor1 = 0
	motor2 = 0

class kinematic(LifecycleNode):
    def __init__(self):
        super().__init__('kinematic')
        self.get_logger().warn("ROS 2 Node Kinematic Initialized!")
        self.killnode = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_circumference',         0.471),
                ('transmission_ratio',          20),
                ('distanceBetwentWheels',       0.44),
                ('max_rpm',                     2800),
                ('linear_max',                  0.8),
                ('angular_max',                 0.4),
                ('topicControl_vel',            'cmd_vel'),
                ('topicGet_vel',                'raw_vel'),
                ('topicControl_driverAll',      'mc_request'),
                ('topicRespond_driverAll',      'mc_info'),
                ('frame_id',                    'frame_robot'),
                ('isRevert1',                   0),
                ('isRevert2',                   1)
            ]
        )

        # -- Get param
        self.wheel_circumference = self.get_parameter('wheel_circumference').value
        self.transmission_ratio = self.get_parameter('transmission_ratio').value
        self.distanceBetwentWheels = self.get_parameter('distanceBetwentWheels').value

        self.max_rpm = self.get_parameter('max_rpm').value
        self.linear_max = self.get_parameter('linear_max').value
        self.angular_max = self.get_parameter('angular_max').value

        self.topicControl_vel = self.get_parameter('topicControl_vel').value
        self.topicGet_vel = self.get_parameter('topicGet_vel').value

        self.topicControl_driver = self.get_parameter('topicControl_driverAll').value
        self.topicRespond_driver = self.get_parameter('topicRespond_driverAll').value

        self.frame_id = self.get_parameter('frame_id').value
        self.isRevert1 = self.get_parameter('isRevert1').value
        self.isRevert2 = self.get_parameter('isRevert2').value

        # -- Publisher 
        # -
        self.pub_rawVel = self.create_publisher(TwistWithCovarianceStamped, self.topicGet_vel, 10)
        self.raw_vel = TwistWithCovarianceStamped()
        # -
        self.pub_driverControl = self.create_publisher(McRequest, self.topicControl_driver, 10)
        self.driver_query = McRequest()
        # -

        # -- Subcriber
        # -
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            self.topicControl_vel,
            self.cmdVel_callback,
            10)
        self.subscription_cmd_vel
        self.cmd_vel = Twist()

        self.subscription_task_driver = self.create_subscription(
            Int16,
            "/task_driver",
            self.taskDriver_callback,
            10)
        self.subscription_task_driver
        self.task_driver = Int16()

        self.subscription_driver_respond = self.create_subscription(
            McInfo,
            self.topicRespond_driver,
            self.driverRespond_callback,
            10)
        self.subscription_driver_respond
        self.driver_respond = McInfo()

        # -- 
        # -- find main driver.
        self.is_finded = 0
        self.mainDriver = 0
        # -- frequence pub raw vel.
        self.fre_rawVel = 25.
        self.cycle_rawVel = 1/self.fre_rawVel
        self.timeout = self.cycle_rawVel*4
        self.isNew_driver = 0
        # -- 
        self.is_exit = 1
        #-- 
        self.nowTime_cmdVel = time.time()
        self.timeout_cmdVel = 0.4
        self.is_timeout = 0
        # -- 
        self.max_deltaTime = 0.0
        # -- 
        self.max_rotation = 0.6 # rad/s

        # -- Loop
        self.rate_main = 30
        self.timer_period_main = 1/self.rate_main
        self.timer_main = self.create_timer(self.timer_period_main, self.run)

        self.rate_getVel = 30
        self.timer_period_getVel = 2.
        self.timer_getVel = self.create_timer(self.timer_period_getVel, self.getVel)

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    # -- Callback function
    def taskDriver_callback(self, data):
        self.task_driver = data

    def cmdVel_callback(self, data):
        self.cmd_vel = data
        self.nowTime_cmdVel = time.time()

    def driverRespond_callback(self, data):
        self.driver_respond = data
        self.nowTime_speed = time.time()
        self.isNew_driver = 1

    def constrain(self, val_in, val_compare1, val_compare2):
        if val_in >= val_compare2:
            return val_compare2
        if val_in <= val_compare1:
            return val_compare1
        return val_in

    def calculateRPM(self, linear_x, angular_z):
        linear_x_right = 0
        angular_x_right = 0
        linear_vel_x_mins = 0.0
        angular_vel_z_mins = 0.0
        tangential_vel = 0.0
        x_rpm = 0.0
        tan_rpm = 0.0

        # -- Limit
        linear_x_right = self.constrain(linear_x, -1.*self.linear_max, self.linear_max)
        angular_x_right = self.constrain(angular_z, -1.*self.angular_max, self.angular_max)

        # convert m/s to m/min
        linear_vel_x_mins = linear_x_right * 60
        # convert rad/s to rad/min
        angular_vel_z_mins = angular_x_right * 60
        tangential_vel = angular_vel_z_mins * (self.distanceBetwentWheels / 2)

        x_rpm = linear_vel_x_mins / self.wheel_circumference
        tan_rpm = tangential_vel / self.wheel_circumference

        rpm_query = RPM()

        # calculate for the target motor RPM and direction
        # front-left motor
        rpm_query.motor1 = (x_rpm - tan_rpm)*self.transmission_ratio
        rpm_query.motor1 = self.constrain(rpm_query.motor1, -self.max_rpm, self.max_rpm)
        # front-right motor
        rpm_query.motor2 = (x_rpm + tan_rpm)*self.transmission_ratio
        rpm_query.motor2 = self.constrain(rpm_query.motor2, -self.max_rpm, self.max_rpm)

        return rpm_query

    def convert_RPM_8bit(self, rpm_input):
        return int((rpm_input/3000.)*255)

    def calculate_rawVel(self, rp1, rp2): # rp1,rp2: RPM | out: Velocities(m/s;rad/s)
        vel = TwistWithCovarianceStamped()
        average_rps = ((rp2 + rp1)/2.)/60./self.transmission_ratio
        angular_rps = ((rp2 - rp1)/2.)/60./self.transmission_ratio

        vel.twist.twist.linear.x = average_rps*self.wheel_circumference
        vel.twist.twist.angular.z = (angular_rps*self.wheel_circumference)/(self.distanceBetwentWheels/2.)

        vel.twist.covariance[0] = 0.001
        vel.twist.covariance[7] = 0.001
        vel.twist.covariance[35] = 0.001

        vel.header.stamp = self.get_clock().now().to_msg()
        vel.header.frame_id =  self.frame_id

        return vel

    def getVel(self):
        if (self.isNew_driver == 1):
            vel_1 = self.driver_respond.speed1
            vel_2 = self.driver_respond.speed2
            self.raw_vel = self.calculate_rawVel(vel_1, vel_2)
            self.pub_rawVel.publish(self.raw_vel)
            self.isNew_driver1 = 0

    def run(self):
        # - Reset Alamp.
        if (self.task_driver.data == 1):
            self.driver_query.reset = 1
        else:
            self.driver_query.reset = 0

        if (self.cmd_vel.linear.x == 0. and self.cmd_vel.angular.z == 0.):
            self.driver_query.mode_operate = 2
            self.driver_query.speed1 = 0
            self.driver_query.speed2 = 0

        else:
            self.driverRPM_query = self.calculateRPM(self.cmd_vel.linear.x, self.cmd_vel.angular.z)
            self.driver_query.mode_operate = 1

            # - Speed
            self.driver_query.speed1 = self.convert_RPM_8bit(abs(self.driverRPM_query.motor1))
            self.driver_query.speed2 = self.convert_RPM_8bit(abs(self.driverRPM_query.motor2))

            # - Driection 1
            if (self.driverRPM_query.motor1 >= 0):
                if (self.isRevert1 == 0):
                    self.driver_query.dir1 = 0
                else:
                    self.driver_query.dir1 = 1
            else:
                if (self.isRevert1 == 0):
                    self.driver_query.dir1 = 1
                else:
                    self.driver_query.dir1 = 0

            # - Driection 2
            if (self.driverRPM_query.motor2 >= 0):
                if (self.isRevert2 == 0):
                    self.driver_query.dir2 = 0
                else:
                    self.driver_query.dir2 = 1
            else:
                if (self.isRevert2 == 0):
                    self.driver_query.dir2 = 1
                else:
                    self.driver_query.dir2 = 0

        if (self.is_timeout):
            self.pub_driverControl.publish(McRequest())
        else: # -- ok
            self.pub_driverControl.publish(self.driver_query)


        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)


def main():
    # -- Khoi tao ROS
    rclpy.init()
    node = kinematic()

    # Dùng MultiThreadedExecutor để chạy song song
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')

if __name__ == '__main__':
    main()
