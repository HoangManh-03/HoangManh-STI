#include<iostream>
#include<ros/ros.h>
#include<testMotionPlanning/goalControl.h>

namespace GoalControl{
    int status = 0;
    
    int stop_AGV = 0;

    geometry_msgs::Pose poseRbMa;

    goalControl::goalControl(ros::NodeHandle& nh) : nh_(nh), stopRequested(false), status(0), rate(20), angle_giam_toc(1.0), vel_rot(0.1), tolerance_rot_step1(0.02), vel_rot_step1(0.02),
    targetx(1.000),targety(2.000), theta_rb_ht(0), vel_th(0), rate_cmdVel(20)
    {
        // Khởi tạo Publisher trong constructor
        signal(SIGINT, signalHandler);
        pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        robot_pose = nh_.subscribe("/robotPose_nav", 20, &goalControl::getPose, this);
        time_tr = ros::Time::now();
    }

    double goalControl::distance_two_point(double x1, double y1, double x2, double y2){
        return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }

    std::tuple<double, double, double> goalControl::find_hc(const double startx, double starty, double finishx, double finishy){
        double X_projection = 0.0;
        double Y_projection = 0.0;
        double distance_projection = 0.0;
        double X_n = 0;
        double Y_n = 0;
        
        // phuong trinh duong thang quy dao ax + by + c = 0
        double a = starty - finishy;
        double b = finishx - startx;
        double c = -startx * a - starty * b;

        if (poseRbMa.position.x == startx && poseRbMa.position.y == starty){
            X_projection = startx;
            Y_projection = starty;
        }
        else{
            double a_hc = b;
            double b_hc = -a;
            double c_hc = -poseRbMa.position.x * a_hc - poseRbMa.position.y * b_hc;

            X_projection = ((c_hc*b)-(c*b_hc))/((a*b_hc)-(b*a_hc));
            Y_projection = ((c_hc*a)-(c*a_hc))/((a_hc*b)-(b_hc*a));
        }


        distance_projection = distance_two_point(poseRbMa.position.x, poseRbMa.position.y, X_projection, Y_projection);
        if (check_point_position(startx, starty, finishx, finishy, poseRbMa.position.x, poseRbMa.position.y) == 1){
            distance_projection = distance_projection;
        }
        else if (check_point_position(startx, starty, finishx, finishy, poseRbMa.position.x, poseRbMa.position.y) == -1){
            distance_projection = -distance_projection;
        }
        else{
            distance_projection = 0;
        }

        // distance_projection = (a*poseRbMa.position.x + b*poseRbMa.position.y + c)/std::sqrt((a*a + b*b));
        return std::make_tuple(X_projection, Y_projection, distance_projection);
    }

    int goalControl::check_point_position(double x1, double y1, double x2, double y2, double x, double y) {

        double cross_product = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1);
        if (cross_product > 0) {
            return 1; // Điểm P nằm bên trái đoạn thẳng AB
        } else if (cross_product < 0) {
            return -1; // Điểm P nằm bên phải đoạn thẳng AB
        } else {
            return 0; // Điểm P nằm trên đoạn thẳng AB
        }

    }


    // std::tuple<double, double, double> goalControl::find_hc(const double startx, const double starty, const double finishx, const double finishy) {
    //     double X_projection = 0.0;
    //     double Y_projection = 0.0;
    //     double distance_projection = 0.0;

    //     // Phương trình đường thẳng quy đạo ax + by + c = 0
    //     double a = starty - finishy;
    //     double b = finishx - startx;
    //     double c = -startx * a - starty * b;

    //     // Tính tọa độ hình chiếu của poseRbMa.position lên đường thẳng
    //     if (poseRbMa.position.x == startx && poseRbMa.position.y == starty) {
    //         // Nếu poseRbMa.position trùng với điểm đầu
    //         X_projection = startx;
    //         Y_projection = starty;
    //     } else {
    //         // Tính toán tọa độ hình chiếu
    //         X_projection = (b * (b * poseRbMa.position.x - a * poseRbMa.position.y) - a * c) / (a * a + b * b);
    //         Y_projection = (a * (-b * poseRbMa.position.x + a * poseRbMa.position.y) - b * c) / (a * a + b * b);
    //     }

    //     // Tính khoảng cách giữa poseRbMa.position và hình chiếu
    //     distance_projection = distance_two_point(poseRbMa.position.x, poseRbMa.position.y, X_projection, Y_projection);

    //     return std::make_tuple(X_projection, Y_projection, distance_projection);
    // }

    void goalControl::stop(){    // dung lai cai con AGV nayyyy
        geometry_msgs::Twist twist;
        ROS_INFO("bbbbbbbbbbbbbbbbbbbbbbbbbbbb");
        twist.linear.x = 0.0;
        twist.angular.z = 0.1;
        for(int i = 0; i < 2; i++){
            pub_cmd_vel.publish(twist);
        }  
    }

    void goalControl::stop2(){    // dung lai cai con AGV nayyyy
        stopRequested = true;
        geometry_msgs::Twist twist;
        ROS_INFO("truoc pub cmd vel ............");
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;

        for(int i = 0; i < 2; i++){
        pub_cmd_vel.publish(twist);      
        ROS_INFO("OKEEEEEEEEEE");
        }
    }

    void goalControl::requestStop() {
        stopRequested = true; // Đặt biến điều kiện để dừng chương trình
    }

    void goalControl::pub_cmdVel(const geometry_msgs::Twist twist, int rate){
        if(ros::Time::now() - time_tr > ros::Duration(1.0 / rate)){
            time_tr = ros::Time::now();
            pub_cmd_vel.publish(twist);
        }
    }

    void goalControl::getPose(const geometry_msgs::PoseStamped::ConstPtr& data) {
        poseRbMa = data->pose;

        // Chuyển đổi quaternion sang góc Euler
        tf::Quaternion quat(
            poseRbMa.orientation.x,
            poseRbMa.orientation.y,
            poseRbMa.orientation.z,
            poseRbMa.orientation.w
        );

        tf::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        theta_rb_ht = yaw; // yaw tương ứng với góc quay z
    }

    void goalControl::readpose(){

    }

    double goalControl::turn_ar(double theta, double tol_theta, double vel_rot){
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

                ROS_INFO("vel th %f", vel_th);
                return -vel_th;
            }
            if (theta < 0){
                if (fabs(theta) <= angle_giam_toc) {
                    vel_th = (fabs(theta) / angle_giam_toc)*vel_rot;
                }
                else{
                    vel_th = vel_rot;
                }
                if (vel_th < 0.1){
                    vel_th = 0.1;
                }
                ROS_INFO("vel th %f", vel_th);
                return -vel_th;
            }

        }
        else{
            return -10;
        }
    }

    void goalControl::signalHandler(int signal) {
        ROS_INFO("Signal received: %d", signal);
        // Thực hiện các thao tác cần thiết khi nhận tín hiệu
        // Ví dụ: goalControlInstance.requestStop();
        ros::shutdown(); // Tắt ROS khi nhận tín hiệu
    }

    double goalControl::Cal_angle(double x1, double y1, double x2, double y2, double theta_rb_ht){
        return atan2(y2 - y1, x2 - x1) - theta_rb_ht;
    }


    double goalControl::accellometer(double delta_time, double time_s, double v_s, double v_f) {
        double v_re = 0.0;
        double delta_time_now = ros::Time::now().toSec() - time_s;
        double a = (v_f - v_s) / delta_time;

        if (delta_time_now <= delta_time) {
            v_re = v_s + a * delta_time_now;
        } else {
            v_re = v_f;
        }

        return v_re;
    }

    void goalControl::run(){

    }
}
