#include<iostream>
#include<ros/ros.h>
#include<thread>
#include <std_msgs/String.h>    // Thay cho from std_msgs.msg import String
#include <std_msgs/Bool.h>      // Thay cho from std_msgs.msg import Bool
#include <std_msgs/Int8.h>      // Thay cho from std_msgs.msg import Int8

#include <cstdlib>         // Thay cho import sys
#include <cstdint>         // Thay cho import struct
#include <string>          // Thay cho import string
#include <ros/package.h>   // Thay cho import roslib
#include <serial/serial.h> // Thay cho import serial (Cần thư viện bổ sung: `sudo apt-get install ros-<ros_distro>-serial`)
#include <csignal>         // Thay cho import signal

#include <geometry_msgs/PoseWithCovarianceStamped.h>   // Thay cho from geometry_msgs.msg import PoseWithCovarianceStamped
#include <geometry_msgs/PoseStamped.h>                 // Thay cho from geometry_msgs.msg import PoseStamped
#include <geometry_msgs/Quaternion.h>                  // Thay cho from geometry_msgs.msg import Quaternion
#include <geometry_msgs/Pose.h>                        // Thay cho from geometry_msgs.msg import Pose
#include <geometry_msgs/Twist.h>                       // Thay cho from geometry_msgs.msg import Twist
#include <geometry_msgs/TwistWithCovarianceStamped.h>  // Thay cho from geometry_msgs.msg import TwistWithCovarianceStamped
#include <sensor_msgs/LaserScan.h>                     // Thay cho from sensor_msgs.msg import LaserScan
#include <nav_msgs/Odometry.h>                         // Thay cho from nav_msgs.msg import Odometry
#include <visualization_msgs/Marker.h>                 // Thay cho from visualization_msgs.msg import Marker
#include <visualization_msgs/MarkerArray.h>            // Thay cho from visualization_msgs.msg import MarkerArray

#include <cmath>                                       // Thay cho from math import pi as PI, atan2, sin, cos, sqrt, fabs, acos
#include <tf/tf.h>                                     // Thay cho from tf.transformations import euler_from_quaternion, quaternion_from_euler
#include <sti_msgs/Move_request.h>                     // Thay cho from sti_msgs.msg import Move_request
#include <sti_msgs/Move_respond.h>                     // Thay cho from sti_msgs.msg import Move_respond
#include <sti_msgs/Zone_lidar_2head.h>                 // Thay cho from sti_msgs.msg import Zone_lidar_2head
#include <sti_msgs/POWER_info.h>                       // Thay cho from sti_msgs.msg import POWER_info
#include <sti_msgs/Status_goalControl.h>               // Thay cho from sti_msgs.msg import Status_goalControl
#include <sti_msgs/Velocities.h>                       // Thay cho from sti_msgs.msg import Velocities
#include <sti_msgs/Status_goal_control.h>              // Thay cho from sti_msgs.msg import Status_goal_control
#include <sti_msgs/HC_info.h>                          // Thay cho from sti_msgs.msg import HC_info

#include <cstdlib>     // Thay cho import os
#include <nav_msgs/Path.h>  // Thay cho from nav_msgs.msg import Path
#include <cmath>       // Thay cho import math
#include <tuple>

class PID{
public:
    PID() : kp(0.1), ki(0.001), kd(0.00001), previous_error(0), integral(0) {}

    double calculate(double error) {
        integral += error;
        double derivative = error - previous_error;
        previous_error = error;
        double output = kp * error + ki * integral + kd * derivative;
        return output;
    }

private:
    double kp;
    double ki;
    double kd;
    double previous_error;
    double integral;
};


class goalControl {
public:
    GoalControl();  // Constructor declaration
    void getPose(const geometry_msgs::Pose::ConstPtr& data);
    void zone_callback(const sti_msgs::HC_info& data);
    void pub_cmdVel(const geometry_msgs::Twist& cmd_vel, int rate);

    ros::NodeHandle nh;  // NodeHandle
    ros::Subscriber sub_robotPose_nav;  // Subscriber for robotPose_nav
    ros::Subscriber sub_HC_info;  // Subscriber for HC_info
    ros::Publisher pub_respond;  // Publisher for Move_respond
    ros::Publisher pub_requestFields;  // Publisher for HC_fieldRequest
    ros::Publisher pub_stt_goal;  // Publisher for Status_goal_control
    // Declare other public members and variables if needed
    geometry_msgs::Pose poseRbMa;
    ros::Rate rate;
    PID pid;
    double time_tr = ros::Time::now().toSec();

    double M_PI = 3.1416;
    double targetx = 8.325;
    double targety = 1.001;
    double target_z = 3.14;
    double kc_backward = 1.2;
    int check_theta = 0;
    bool is_odom_rb = false;
    bool is_raw_vel = false;
    bool is_pose_robot = false;
    int status = 1;
    int rate_cmdVel = 30;
    double angle_giam_toc = 45.0 * M_PI / 180;
    double vel_rot_step1 = 0.45;
    double toerance_rot_step1 = 0.005;
    double theta_rb_ht;
    double point_goal_start_x;
    double point_goal_start_y;
    double Cal_angle(double x1, double y1, double x2, double y2, double theta_rb_ht)

};

    goalControl::GoalControl()
    {
        ros::NodeHandle nh;
        pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
        pub_respond = nh.advertise<sti_msgs::Move_respond>("/respond_move", 20);
        pub_requestFields = nh.advertise<std_msgs::Int8>("/HC_fieldRequest", 20);
        pub_stt_goal = nh.advertise<sti_msgs::Status_goal_control>("/status_goal_control", 10)
        

        
        sub_raw_vel = nh.subscribe("/raw_vel", 1, &GoalControl::rawvel_callback, this);
        sub_robotPose_nav = nh.subscribe("/robotPose_nav", 20, &GoalControl::getPose, this);
        sub_HC_info = nh.subscribe("/HC_info", 20, &GoalControl::zone_callback, this);
    }

    goalControl::Cal_angle(double x1, double y1, double x2, double y2, double theta_rb_ht){
        return atan2(y2 - y1, x2 - x1) - theta_rb_ht;
    }
    void getPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
        is_pose_robot = true;
        poseRbMa = msg->pose;
        tf::Quaternion quat(
            poseRbMa.orientation.x,
            poseRbMa.orientation.y,
            poseRbMa.orientation.z,
            poseRbMa.orientation.w
        );
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta_rb_ht = yaw;
        std::cout<<"theta_rb_ht:" <<theta_rb_ht;

    }
    void stop(){    // Stop robot is all 
        geometry_msgs::Twist twist;
        for(int i = 0; i < 2; i++){
            pub_cmd_vel.publish(twist);
        }
    }

    void pub_cmdVel(geometry_msgs::Twist twist, int rate){     // pub cmd_vel, control robot by linear.x, angular.z
        if(ros::Time.toSec() - time_tr > float(1 / rate)){
            time_tr = ros::Time.toSec();
            pub_cmd_vel.publish(twist);
        }
    }
    
    int distance_two_point(double x1, double y1, double x2, double y2){
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    tuple<double, double, double> find_hc(double X_s, double Y_s, double X_f, double Y_f){
        double X_projection = 0.0
        double Y_projection = 0.0
        double distance_projection = 0.0

        // Tham so phuong trinh duong thang quy dao
        double a_ = Y_s - Y_f;
        double b_ = X_f - X_s;
        double c_ = -X_s * a_ - Y_s * b_;

        // Diem bat dau robot
        if (poseRbMa.posision.x == X_s poseRbMa.posision.y == Y_s){
            X_n = X_s;
            Y_n = Y_s;
        }
        else{
            // tham so pt duong thang hinh chieu
            c_hc = -poseRbMa.posision.x * b_ - poseRbMa.posision.y * a_;
        }

        X_projection = (b_*(b_*poseRbMa.posision.x - a_*poseRbMa.posision.y) - a_*c_)/(a_*a_ + b_*b_);
        Y_projection = (a_*(-b_*poseRbMa.posision.x + a_*poseRbMa.posision.y) - b_*c_)/(a_*a_ + b_*b_);

        distance_projection = distance_two_point(poseRbMa.posision.x, poseRbMa.posision.y, X_projection, Y_projection);
        return X_projection, Y_projection, distance_projection;
    }

    int turn_ar(double theta, double tol_theta, double vel_rot){
        if (fabs(theta) > tol_theta){
            if (theta > 0){
                if(fabs(theta) <= angle_giam_toc){
                    vel_th = (fabs(theta) / angle_giam_toc)*vel_rot;
                }
                else{
                    vel_th = vel_rot;
                }
                if (vel_th < 0.1){
                    vel_th = 0.1;
                }
                return -vel_th;
            }
            if (theta < 0){
                if (fabs(theta) <= angle_giamtoc) {
                    vel_th = (fabs(theta) / angle_giam_toc)*vel_rot;
                }
                else{
                    vel_th = vel_rot;
                }
                if (vel_th < 0.1){
                    vel_th = 0.1;
                }
                return -vel_th;
            }

        }
        else{
            return -10;
        }
    }

    int round_precision(double x1, double x2){
        double precision = 0.01;
        if (x2 - x1 >= precision){
            return 0;
        }
        else{
            return 1;
        }
    }

    void run(){
        while (ros::ok()){
            if (status == 1){
                double theta = Cal_angle(poseRbMa.position.x, poseRbMa.position.y, targetx, targety, theta_rb_ht);
                std::cout << "theta: " << theta;
                geometry_msgs::Twist twist;
                double gt = turn_ar(theta, tolerance_rot_step1, vel_rot_step1);
                if (gt == 10){
                    std::cout << "da quay dung goc";
                    stop();
                    ros::Duration(0.3).sleep();
                    status = 2;
                    point_goal_start_x = poseRbMa.position.x;
                    point_goal_start_y = poseRbMa.position.y;
                }
                else{
                    twist.angular.z = gt;
                    pub_cmdVel(twist, rate_cmdVel);
                }
            }
            rate.sleep();
        }
    }


int main(int argc, char** argv){
    ros::init(argc, argv, "goal_control_v2");
    goalControl goalControl;
    // goalControl.run();
    return 0;
}
