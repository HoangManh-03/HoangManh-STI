#include<iostream>
#include<ros/ros.h>
#include<vector>
#include <geometry_msgs/PoseWithCovarianceStamped.h>   // Thay cho from geometry_msgs.msg import PoseWithCovarianceStamped
#include <geometry_msgs/PoseStamped.h>                 // Thay cho from geometry_msgs.msg import PoseStamped
#include <geometry_msgs/Quaternion.h>                  // Thay cho from geometry_msgs.msg import Quaternion
#include <geometry_msgs/Pose.h>                        // Thay cho from geometry_msgs.msg import Pose
#include <geometry_msgs/Twist.h>                       // Thay cho from geometry_msgs.msg import Twist
#include <geometry_msgs/TwistWithCovarianceStamped.h>  // Thay cho from geometry_msgs.msg import TwistWithCovarianceStamped

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

#include "testMotionPlanning/move_task.h"
#include <string.h>
#include <thread>
#include <signal.h>
#include <vector>

namespace GoalControl{

class goalControl {
    public:
    // goalControl();
    
    goalControl(ros::NodeHandle& nh);

    ~goalControl() {
        stop2();
    }

    geometry_msgs::Pose poseRbMa;
    double theta_rb_ht;
    
    // void initialize(std::string name);
    bool shutdown_requested_;
    int is_shutdown = 0;

    double distance_two_point(double x1, double x2, double y1, double y2);
    void stop();
    void stop2();
    void exit_f();

    static void shutdownCallback();

    double Cal_angle(double x1, double y1, double x2, double y2, double theta);
    void pub_cmdVel(const geometry_msgs::Twist twist, int cmd_rate);
    void getPose(const geometry_msgs::PoseStamped::ConstPtr& data);
    void zone_callback(const sti_msgs::HC_info& data);
    void rawvel_callback(const geometry_msgs::TwistWithCovarianceStamped& data);
    double turn_ar(double theta, double tol_theta, double vel_rot);
    void readpose();
    void requestStop();
    int check_point_position(double x1, double y1, double x2, double y2, double x, double y);
    
    static void signalHandler(int signal);
    double accellometer(double delta_time, double time_s, double v_s, double v_f);
    std::tuple<double, double, double> find_hc(double startx, double starty, double finishx, double finishy); 
    void run();

////////////////////////////////////////////////////////////////////////////
    private:
    bool stopRequested;
    int status;
    int stop_AGV;

    double targetx, targety;

    double vel_th;
    double vel_rot;
    ros::Rate rate;
    double angle_giam_toc;
    double vel_rot_step1;
    double tolerance_rot_step1;
    
    int rate_cmdVel;

    std::string name_;
        // thoat chuong trinh 

    ros::NodeHandle nh_;

    // khai bao pub sub
    ros::Publisher pub_cmd_vel;
    ros::Publisher notification;

    ros::Subscriber robot_pose;
    ros::Subscriber safety_zone;
    ros::Time time_tr; // Khai báo biến thời gian
};

}
