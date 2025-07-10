#include<iostream>
#include<ros/ros.h>
#include<vector>

namespace GoalControl{

class goalControl {
    public:
    
    void distance_two_point(double x1, double x2, double y1, double y2);
    void stop();
    void pub_cmdVel(const geometry_msgs::Twist twist, int rate);
    void getPose(const geometry_msgs::Pose::ConstPtr& data);
    void zone_callback(const sti_msgs::HC_info& data);
    std::vector<double> find_hc(double X_s, double Y_s, double X_f, double Y_f);
    int turn_ar(double theta, double tol_theta, double vel_rot);

    private:
    
    // khai bao pub sub
    ros::Publisher pub_cmd_vel;
    ros::Publisher notification;

    ros::Subscriber robot_pose;
    ros::Subscriber safety_zone;
};

}