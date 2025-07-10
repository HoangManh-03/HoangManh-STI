// Author : Phùng Quý Dương 
// Date: 21-09-2023

/*
    - Node Parking_nav_cpp 
    - function:
        + Điều khiển AGV di chuyển qua các điểm
*/

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/Int8.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "sti_msgs/HC_info.h"
#include "sti_msgs/Move_request.h"
#include "sti_msgs/Move_respond.h"
#include "sti_msgs/Status_goal_control.h"

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <bits/stdc++.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <signal.h>
#include <csignal>
#include <tuple>
#include <limits>
#include <chrono>
using namespace std;

class goalControl{
    public:
    float vel_x_max;
    float vel_x_min; 
    double vel_theta_max;
    double vel_theta_min;
    double tolerance_xy_max;
    double tolerance_xy_min;
    double tolerance_theta_max;
    double tolerance_theta_min;

    double tolerance_theta; 
    double tolerance_rot_step1;

    float vel_rot_step1;
    float vel_rot_step_f;

    double gioihan_lui;
    float K_x;
    double dist_ahead_max;
    double dist_ahead_min;
    double khoang_offset;
    double distance_goArc;
    bool needRotatyFinish;
    bool need_check_goal = false;

    ros::Subscriber sub_request_move;
    sti_msgs::Move_request req_move;
    bool is_request_move = false;

    ros::Subscriber sub_robotPose;
    bool is_pose_robot = false;
    double theta_rb_ht = 0.0;
    geometry_msgs::Pose poseRbMa;

    ros::Subscriber sub_GetRobotOdom;
    bool is_odom_rb = false;
    nav_msgs::Odometry odom_rb;

    ros::Subscriber sub_HCinfo;
    sti_msgs::HC_info zone_lidar;
    bool is_check_zone = false;

    ros::Subscriber sub_rawvel;
    bool is_raw_vel = false;
    geometry_msgs::TwistWithCovarianceStamped vel_raw;

    ros::Subscriber sub_safetyNav;
    bool is_ZoneNav = false;
    std_msgs::Int8 dataZoneNav;

    ros::Publisher pub_respond;
    sti_msgs::Move_respond pub_move;

    ros::Publisher pub_requestFields;
    uint8_t oldselectfield = 0;

    ros::Publisher pub_stt_goal;
    ros::Publisher pub_path_local;
    ros::Publisher pub_path_global;
    ros::Publisher pub_cmd_vel;

    uint8_t auto_reset = 0;
    bool is_slowly = false;
    uint8_t check_goArc = 0;

    // cac bien khac
    int process = 0;
    string pre_mess = "";      // cout<< just one time.
    uint8_t war_agv = 0; // 0: agv di chuyen binh thuong | 1: agv gap vat can

    double coordinate_unknown = 500.0;
    vector<double> list_unknow {0.0, 0.0, 0.0, 0.0, 0.0};
    bool is_target_change = false;

    double target_x = coordinate_unknown;
    double target_y = coordinate_unknown;
    double target_z = 0;
    int32_t tag;
    double offset;
    vector<double> list_x {0.0, 0.0, 0.0, 0.0, 0.0};
    vector<double> list_y {0.0, 0.0, 0.0, 0.0, 0.0};
    vector<int32_t> list_id {0, 0, 0, 0, 0};
    vector<double> list_vel {0.0, 0.0, 0.0, 0.0, 0.0};
    vector<double> old_list_x = list_unknow;
    vector<double> old_list_y = list_unknow;
    double old_target_x = coordinate_unknown;
    double old_target_y = coordinate_unknown;
    int32_t old_id_follow = 0.0;
    int32_t mission = 0;

    uint8_t completed_simple = 0;      // bao da den dich.
    uint8_t completed_backward = 0;     // bao da den aruco.
    uint8_t completed_all = 0;
    uint8_t completed_list = 0;
    uint8_t completed_reset = 0;
    uint8_t stt_agv = 0;
    uint8_t error = 0;

    bool end_of_list = false;

    double cur_goal_x = 0.0;
    double cur_goal_y = 0.0;

    double point_goal_start_x = 0.0;
    double point_goal_start_y = 0.0;

    bool is_need_turn_step1 = 0;
    bool is_need_pttt = 0;
    bool is_pre_pttt = 0;
    double x_td_goal = 0.0;
    double y_td_goal = 0.0;
    float dis_hc = 0.0;

    uint8_t cur_goal_is = 0; //1: diem thuong, 2: diem dac biet, 3: diem dich

    double distance_goal = 0.0;

    double tol_simple = 0.04;      // do chinh xac theo truc x,y
    double tol_target = 0.01;
    double ss_luivsnextGoal = 0.15;

    float vel_x_now = 0.0;

    float vel_x_control = 0.0;   // toc do theo truc x
    double theta_max = 0.6;

    float vel_x1 = 0.45;		// level 1 slowless
    double theta_max_1 = 0.4;

    float vel_x2 = 0.55;  		// level 2 
    double theta_max_2 = 0.5;

    float vel_x3 = 0.65;  		// level 3 fastless
    double theta_max_3 = 0.6;

    float min_vel_x = 0.04;

    double odom_x_ht = 0.0;
    double odom_y_ht = 0.0;
    float kc_backward = 0.0;

    double XRobotStart = 0.0;
    double YRobotStart = 0.0;

    float min_vel_x_gh = 0.2;
    double theta = 0.0;

    double angle_find_vel = 25.0*M_PI/180.0;
    double time_start_navi = ros::Time::now().toSec();

    double angle_giam_toc = 45.0*M_PI/180.0;

    float dis_gt = 1.15;
    float dis_gt_khilui = 0.3;
    float kc_con_lai = 0.0;
    float kc_qd = 0.0;
    bool is_over_goal = false;

    double X_n = 0.0;
    double Y_n = 0.0;
    double a_qd = 0.0;
    double b_qd = 0.0;
    double c_qd = 0.0;

    int32_t id_fl = 0;
    float vel_fl = 0.0;

    uint8_t rate_cmdvel = 30;
    double time_tr = ros::Time::now().toSec();

    nav_msgs::Path path_plan;

    double timeRecieveNAV = ros::Time::now().toSec();
    double timeRecieveTIM = ros::Time::now().toSec();

    uint8_t timeZone3TIM = 0;
    uint8_t timeZone2TIM = 0;

    float timeWaitTIM = 0.1;
    float timeWaitNAV = 0.5;

    uint8_t oldzone = 0;
    
    uint8_t is_target = 0;

    goalControl(ros::NodeHandle *nh, ros::NodeHandle *npr){
        ros::param::get("~vel_x_max", vel_x_max);
        ros::param::get("~vel_x_min", vel_x_min);
        ros::param::get("~vel_theta_max", vel_theta_max);
        ros::param::get("~vel_theta_min", vel_theta_min);
        ros::param::get("~tolerance_xy_max", tolerance_xy_max);
        ros::param::get("~tolerance_xy_min", tolerance_xy_min);
        ros::param::get("~tolerance_theta_max", tolerance_theta_max);
        ros::param::get("~tolerance_theta_min", tolerance_theta_min);
        ros::param::get("~tolerance_theta", tolerance_theta);
        ros::param::get("~tolerance_rot_step1", tolerance_rot_step1);
        ros::param::get("~vel_rot_step1", vel_rot_step1);
        ros::param::get("~vel_rot_step_f", vel_rot_step_f);
        ros::param::get("~gioihan_lui", gioihan_lui);
        ros::param::get("~he_so", K_x);
        ros::param::get("~khoang_nhin_truoc_max", dist_ahead_max);
        ros::param::get("~khoang_nhin_truoc_min", dist_ahead_min);
        ros::param::get("~khoang_offset", khoang_offset);
        ros::param::get("~distance_goArc", distance_goArc);
        
        // subscriber
        sub_request_move = nh->subscribe("/request_move", 100, &goalControl::move_callback, this);
        sub_robotPose = nh->subscribe("/robotPose_nav", 20, &goalControl::getPose, this);
        sub_GetRobotOdom = nh->subscribe("/odometry", 10, &goalControl::cbGetRobotOdom, this);
        sub_HCinfo = nh->subscribe("/HC_info", 1000, &goalControl::zone_callback, this);
        sub_rawvel = nh->subscribe("/raw_vel", 100, &goalControl::rawvel_callback, this);
        sub_safetyNav = nh->subscribe("/safety_NAV", 100, &goalControl::cbZoneNAV, this);   

        // publisher
        pub_respond = nh->advertise<sti_msgs::Move_respond>("/respond_move", 20);
        pub_requestFields = nh->advertise<std_msgs::Int8>("/HC_fieldRequest", 20);
        pub_stt_goal = nh->advertise<sti_msgs::Status_goal_control>("/status_goal_control", 10);
        pub_path_local = nh->advertise<nav_msgs::Path>("/path_plan_local", 20);
        pub_path_global = nh->advertise<nav_msgs::Path>("/path_plan_global", 20);
        pub_cmd_vel = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 20);

        path_plan.header.frame_id = "frame_map_nav350";
        path_plan.header.stamp = ros::Time::now();
    }

    ~goalControl(){}

	geometry_msgs::Quaternion euler_to_quaternion(double euler){
		geometry_msgs::Quaternion quat;
        tf::Quaternion odom_quat;
        odom_quat.setRPY(0,0, euler);
        odom_quat = odom_quat.normalize();
		// odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat.x();
		quat.y = odom_quat.y();
		quat.z = odom_quat.z();
		quat.w = odom_quat.w();
		return quat;
    }

    float quaternion_to_euler(geometry_msgs::Quaternion qua){
        tf::Quaternion quat(qua.x, qua.y, qua.z, qua.w);
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
		return yaw;
    }

    void move_callback(const sti_msgs::Move_request& data){
        req_move = data;
        is_request_move = true;
    }

    void getPose(const geometry_msgs::PoseStamped& data){
        is_pose_robot = true;
        poseRbMa = data.pose;
        theta_rb_ht = quaternion_to_euler(poseRbMa.orientation);
    }

    void zone_callback(const sti_msgs::HC_info& data){
        zone_lidar = data;
        is_check_zone = true;
        // timeRecieveTIM = time(NULL);
        // auto timeRecieveTIM = chrono::high_resolution_clock::now();
        timeRecieveTIM = ros::Time::now().toSec();
    }

    void rawvel_callback(const geometry_msgs::TwistWithCovarianceStamped& data){
        vel_raw = data;
        is_raw_vel = true;
    }

    void cbGetRobotOdom(const nav_msgs::Odometry& data){
        odom_rb = data;
        is_odom_rb = true;
    }

    void cbZoneNAV(const std_msgs::Int8& data){
        dataZoneNav = data;
        is_ZoneNav = true;
        // auto timeRecieveNAV = chrono::high_resolution_clock::now();
        timeRecieveNAV = ros::Time::now().toSec();

        // cout << "TimeReceiveNav is " << timeRecieveNAV << endl;
    }

    void pub_status(int16_t misson, int16_t status_now, int16_t error, int16_t safety, int16_t complete_misson , int16_t id){
        string stt_1 = "AGV dang thuc hien nhiem vu - khong co loi :)";
        string stt_2 = "AGV dang thuc hien nhiem vu - loi co vat can khi di thang :(";
        string stt_3 = "AGV dang thuc hien nhiem vu - loi co vat can khi di lui :(";
        string stt_4 = "AGV dang thuc hien nhiem vu - loi co vat can khi quay tai cho :(";
        string stt_5 = "AGV dang thuc hien nhiem vu - loi target khong hop le ";
        string stt_6 = "AGV dang thuc hien nhiem vu - loi goal khong hop le :(";
        string stt_7 = "AGV da hoan thanh nhiem vu - dang doi nhiem vu tiep theo :)";
        string stt_8 = "AGV khong co nhiem vu gi :)";
        string stt_9 = "AGV da hoan thanh het goal trong list - dang doi list tiep theo :)";
        string stt_10 = "AGV dang thuc hien nhiem vu - loi mat cam bien an toan :(";
        sti_msgs::Status_goal_control status;
        status.misson = misson;
        status.status_now = status_now;
        status.ID_follow = int(id);
        status.error = error;
        status.safety = safety;
        if ((misson == 1 or misson == 3) && complete_misson == 0 && end_of_list == true){
            status.complete_misson = 2;
        }
        else{
            status.complete_misson = complete_misson;
        }

        if (complete_misson == 1){
            status.meaning = stt_7;
        }
        else if (misson == 0){
            status.meaning = stt_8;
        }
        else{
            if (status_now == 1 && safety == 1)
                status.meaning = stt_2;
            else if (status_now == 2 && safety == 1)
                status.meaning = stt_3;
            else if (status_now == 3 && safety == 1)
                status.meaning = stt_4;
            else if (error == 1)
                status.meaning = stt_5;
            else if (error == 2)
                status.meaning = stt_6;
            else if (error == 3)
                status.meaning = stt_10;
            else if (end_of_list == true)
                status.meaning = stt_9;
            else{
                status.meaning = stt_1;
            }
        }

        pub_stt_goal.publish(status);
    }

    geometry_msgs::PoseStamped point_path(double x, double y){
        geometry_msgs::PoseStamped point;
        point.header.frame_id = "frame_map_nav350";
        point.header.stamp = ros::Time::now();
        point.pose.position.x = x;
        point.pose.position.y = y;
        point.pose.position.z = -1.0;
        point.pose.orientation.w = 1.0;
        return point;
    }

    double fnCalcDistPoints(double x1, double x2, double y1, double y2){
        return sqrt((x1 - x2) * (x1- x2) + (y1 - y2) * (y1 - y2));
    }

    double constrain(double val, double min_val, double max_val){
        if (val < min_val) 
            return min_val;
        if (val > max_val)
            return max_val;
        return val;
    }

    //double round_1decimal(double var)
    //{
    //    double value = (int)(var * 10 + .5);
    //    return (double)value / 10;
    //}
    //double round_2decimal(double var)
    //{
    //    double value = (int)(var * 100 + .5);
    //    return (double)value / 100;
    //}
    
    double round_1decimal(double var)
    {   
        double value = round(var * 10) / 10;
        return value;
    }
    
    double round_2decimal(double var)
    {   
        double value = round(var * 100) / 100;
        return value;
    }
    
    double round_3decimal(double var)
    {   
        double value = round(var * 1000) / 1000;
        return value;
    }

    int find_xy(double p_x, double p_y, vector<double> lis_x, vector<double> lis_y){
        for (int i = 0; i < lis_x.size(); i++){
            if (round_3decimal(p_x) == round_3decimal(lis_x[i])){
                if (round_3decimal(p_y) == round_3decimal(lis_y[i])){
                    return i;
                }
            }
        }
        return -1;
    }

    double calAngleThreePoint(double x1, double y1, double x2, double y2, double x3, double y3){
        double dx1 = x1 - x2;
        double dy1 = y1 - y2;
        double dx2 = x3 - x2;
        double dy2 = y3 - y2;
        double c_goc = (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-12);
        double goc = acos(c_goc);
        
        return goc;
    }

    int find_point_special(double x_c_goal, double y_c_goal, double x_n_goal, double y_n_goal, int dk_check){
        double dx2 = x_n_goal - x_c_goal;
        double dy2 = y_n_goal - y_c_goal;
        double d2 = sqrt(dx2*dx2 + dy2*dy2);
        double goc = calAngleThreePoint(poseRbMa.position.x, poseRbMa.position.y, x_c_goal, y_c_goal, x_n_goal, y_n_goal);
        
        if (dk_check == 1){
            if (goc >= 75.0*M_PI/180.0 && goc < 160.0*M_PI/180.0){
                if (d2 > distance_goArc){
                    // ROS_INFO('x = %lf, y = %lf khong la diem dac biet mode CHECK',x_c_goal,y_c_goal);
                    cout << "x = " << x_c_goal << " y= " << y_c_goal << "khong la diem dac biet mode CHECK" << endl;
                    return 1;
                }
                else{
                    ROS_INFO("CHECK NEXT GOAL!!!");
                    return 3;
                }
            }
            else{
                // ROS_INFO('x = %lf, y = %lf la diem dac biet mode CHECK',x_c_goal,y_c_goal);
                cout << "x = " << x_c_goal << " y= " << y_c_goal << " la diem dac biet mode CHECK" << endl;
                return -1;
            }
        }
            
        else{
            if(goc >= 165.0*M_PI/180.0){
                // ROS_INFO('x = %lf, y = %lf khong la diem dac biet',x_c_goal,y_c_goal);
                cout << "x = " << x_c_goal << " y= " << y_c_goal << " khong la diem dac biet" << endl;
                return 1;
            }
            else{
                // ROS_INFO('x = %lf, y = %lf la diem dac biet',x_c_goal,y_c_goal);
                cout << "x = " << x_c_goal << " y= " << y_c_goal << " la diem dac biet" << endl;
                return 2;
            }
        }
    }

    uint8_t update_all(){
        // cur_goal = PoseStamped() // --
        if (req_move.list_x.size() == 5 && req_move.list_y.size() == 5 && req_move.list_id.size() == 5 && req_move.list_speed.size() == 5 && req_move.list_id[0] != 0.0){
            // cout<<"Im here! -------------------- process 3 - update all --------------" << endl;
            cur_goal_x = req_move.list_x[0];
            cur_goal_y = req_move.list_y[0];
            id_fl = req_move.list_id[0];
            vel_fl = req_move.list_speed[0];
            uint8_t flag_error_goal_update = 0;
            target_x = round_3decimal(req_move.target_x);
            target_y = round_3decimal(req_move.target_y);
            target_z = round_3decimal(req_move.target_z);

            // cout << "request move message " << req_move.target_x << " " << req_move.target_y << endl;
            // cout << "target msg " << target_x << " " << target_y << endl;

            stt_agv = 0;
            completed_all = 0;
            completed_simple = 0;
            completed_backward = 0;
            end_of_list = false;
            completed_reset = 0;
            mission = req_move.mission;
            is_target_change = true;
            is_need_turn_step1 = 0;
            is_need_pttt = 0;
            is_pre_pttt = 0;
            cur_goal_is = 0;
            timeZone3TIM = 0;
            timeZone2TIM = 0;
            check_goArc = 0;
            is_target = 0;
            return 1;
        }
        
        else{
            return 2;
        }
    }

    void reset_all(){
        completed_all = 0;
        completed_simple = 0;
        completed_backward = 0;
        end_of_list = false;
        completed_reset = 0;
        stt_agv = 0;
        is_need_turn_step1 = 0;
        is_need_pttt = 0;
        is_pre_pttt = 0;
        error = 0;
        cur_goal_is = 0;
        target_x = coordinate_unknown;
        target_y = coordinate_unknown;
        id_fl = 0.0;
        vel_fl = 0.0;
        timeZone3TIM = 0;
        timeZone2TIM = 0;
        check_goArc = 0;
        selectfield(0);
        is_target = 0;
    }

    void stop(){
        geometry_msgs::Twist twist;
        for(int i = 0; i < 2; i++){
            pub_cmd_vel.publish(twist);
        }
    }

    void pub_cmdVel(geometry_msgs::Twist twist , int rate){

        if (ros::Time::now().toSec() - time_tr > float(1/rate)){ // < 20hz 
            time_tr = ros::Time::now().toSec();
            pub_cmd_vel.publish(twist);
        }
    }


    tuple<double, double, double, double, double, double, double, double> find_hc(double X_s, double Y_s, double X_f, double Y_f){
        double X_n = 0.0;
        double Y_n = 0.0;
        double kc_hinh_chieu = 0.0;
        // pt duong thang quy dao
        double a_qd = Y_s - Y_f;
        double b_qd = X_f - X_s;
        double c_qd = -X_s*a_qd -Y_s*b_qd;

        // pt tu RB to Goal
        double a_rg = poseRbMa.position.y - Y_f;
        double b_rg = X_f - poseRbMa.position.x;
        //pt duong thang hinh chieu
        // a_hc = b_qd
        // b_hc = -a_qd
        // c_hc = -poseRbMa.position.x*a_hc -poseRbMa.position.y*b_hc
        // diem hinh chieu
        if (poseRbMa.position.x == X_s && poseRbMa.position.y == Y_s){
            // cout<<('VAO DAY ROI')
            X_n = X_s;
            Y_n = Y_s;
        }
        else{
            //pt duong thang hinh chieu
            double a_hc = b_qd;
            double b_hc = -a_qd;
            double c_hc = -poseRbMa.position.x*a_hc - poseRbMa.position.y*b_hc;

            X_n = ((c_hc*b_qd)-(c_qd*b_hc))/((a_qd*b_hc)-(b_qd*a_hc));
            Y_n = ((c_hc*a_qd)-(c_qd*a_hc))/((a_hc*b_qd)-(b_hc*a_qd));
        }

        kc_hinh_chieu = sqrt((X_n - poseRbMa.position.x)*(X_n - poseRbMa.position.x) + (Y_n - poseRbMa.position.y)*(Y_n - poseRbMa.position.y));

        return make_tuple(X_n, Y_n, a_qd, b_qd, c_qd, kc_hinh_chieu, a_rg, b_rg);
    }

    tuple<double, double> convert_relative_coordinates(double X_cv, double Y_cv){
        double angle = -theta_rb_ht;
        double _X_cv = (X_cv - poseRbMa.position.x)*cos(angle) - (Y_cv - poseRbMa.position.y)*sin(angle);
        double _Y_cv = (X_cv - poseRbMa.position.x)*sin(angle) + (Y_cv - poseRbMa.position.y)*cos(angle);
        
        return make_tuple(_X_cv, _Y_cv);
    }

    tuple<double, double, double, double, bool> find_point_goal(double X_s, double Y_s, double X_f, double Y_f, double a_qd, double b_qd, double c_qd, double X_n, double Y_n, bool is_target){
        double X_g = 0.0;
        double Y_g = 0.0;
        double X_g1 = 0.0;
        double Y_g1 = 0.0;
        double X_g2 = 0.0;
        double Y_g2 = 0.0;
        double dis_ahead = 0.0;
        double vector_point1_x = 0.0;
        double vector_point1_y = 0.0;
        double vector_point2_x = 0.0;
        double vector_point2_y = 0.0;
        double v_a = 0.0;
        double v_b = 0.0;
        double vector_qd_x = X_s - X_f;
        double vector_qd_y = Y_s - Y_f;
        double x_cv = 0.0;
        double y_cv = 0.0;
        double kc_g1 = 0.0;
        double kc_g2 = 0.0;
        double kc_ns = 0.0;
        double kc_nf = 0.0;
        bool is_over = false;
        double kc_sf = sqrt((X_f - X_s)*(X_f - X_s) + (Y_f - Y_s)*(Y_f - Y_s));

        // pt duong thang quy dao
        kc_nf = sqrt((X_n - X_f)*(X_n - X_f) + (Y_n - Y_f)*(Y_n - Y_f));
        kc_ns = sqrt((X_n - X_s)*(X_n - X_s) + (Y_n - Y_s)*(Y_n - Y_s));

        if (kc_ns >= kc_sf && kc_nf <= kc_sf){
            is_over = true;
        }
        else{
            is_over = false;
        }
            
        if (is_target == 1){
            dis_ahead = dist_ahead_min;
        }
        else{
            dis_ahead = dist_ahead_max;
        }

        if (is_target == 1 && kc_nf < dis_ahead){
            X_g = X_f;
            Y_g = Y_f;
        }
            
        else{
            if(b_qd == 0.0){
                X_g1 = X_g2 = -c_qd/a_qd;
                Y_g1 = -sqrt(dis_ahead*dis_ahead - (X_g1 - X_n)*(X_g1 - X_n)) + Y_n;
                Y_g2 = sqrt(dis_ahead*dis_ahead - (X_g2 - X_n)*(X_g2 - X_n)) + Y_n;
            }
            else{
                double la = (1.0 + (a_qd/b_qd)*(a_qd/b_qd));
                double lb = -2.0*(X_n - (a_qd/b_qd)*((c_qd/b_qd) + Y_n));
                double lc = X_n*X_n + ((c_qd/b_qd) + Y_n)*((c_qd/b_qd) + Y_n) - dis_ahead*dis_ahead;
                double denlta = lb*lb - 4.0*la*lc;
                // cout<<(la,lb,lc,denlta)

                X_g1 = (-lb + sqrt(denlta))/(2.0*la);
                X_g2 = (-lb - sqrt(denlta))/(2.0*la);

                Y_g1 = (-c_qd - a_qd*X_g1)/b_qd;
                Y_g2 = (-c_qd - a_qd*X_g2)/b_qd;
            }

            // loai nghiem bang vector
            vector_qd_x = X_s - X_f;
            vector_qd_y = Y_s - Y_f;

            vector_point1_x = X_n - X_g1;
            vector_point1_y = Y_n - Y_g1;

            if (vector_qd_x == 0.0){
                if (vector_qd_y*vector_point1_y > 0.0){
                    X_g = X_g1;
                    Y_g = Y_g1;
                }
                else{
                    X_g = X_g2;
                    Y_g = Y_g2;
                }
            }
            else if (vector_qd_y == 0.0){
                if (vector_qd_x*vector_point1_x > 0.0){
                    X_g = X_g1;
                    Y_g = Y_g1;
                }
                else{
                    X_g = X_g2;
                    Y_g = Y_g2;
                }
            }

            else{
                v_a = vector_qd_x/vector_point1_x;
                v_b = vector_qd_y/vector_point1_y;
                if (v_a*v_b > 0.0 && v_a > 0.0){
                    X_g = X_g1;
                    Y_g = Y_g1;
                }
                else{
                    X_g = X_g2;
                    Y_g = Y_g2;
                }
            }
        }

        // cout<<(X_g, Y_g)
        tie(x_cv, y_cv) = convert_relative_coordinates(X_g, Y_g);
        return make_tuple(x_cv, y_cv, kc_nf, kc_sf, is_over);
    }

    double ptgt(double denlta_time, double time_s, double v_s, double v_f){
        double v_re = 0.0;
        double denlta_time_now = ros::Time::now().toSec() - time_s;
        double a = (v_f-v_s)/denlta_time;
        if (denlta_time_now <= denlta_time){
            v_re = v_s + a*denlta_time_now;
        }
        else{
            v_re = v_f;
        }

        return v_re;
    }

    float control_navigation(double X_point_goal, double Y_point_goal, float vel_x, double theta, double dis){
        double vel_th = 0.0;
        double l = (X_point_goal*X_point_goal) + (Y_point_goal*Y_point_goal);
        if (Y_point_goal == 0){
            cout<<(Y_point_goal)<<endl;
            Y_point_goal = 0.0001;
        }

        float r = l/(2*fabs(Y_point_goal));
        float vel = vel_x/r;

        if (Y_point_goal > 0){
            vel_th = vel;
        }
        else{
            vel_th = -vel;
        }

        return vel_th;
    }
    
    float control_naviTarget(double X_point_goal, double Y_point_goal){
        float vel_th = 0.0;
        float vel = 0.0;
        
        if (round_3decimal(fabs(Y_point_goal)) == 0.0){
            vel = 0.0;
        }
        else{
            if (round_3decimal(fabs(X_point_goal)) == 0.0){
                X_point_goal = 0.0001;
            }
            double angle = atan2(fabs(Y_point_goal), X_point_goal);
            float vel = 0.45*angle;
        }
            
        if (Y_point_goal > 0){
            vel_th = vel;
        }
        else{
            vel_th = -vel;
        }
        
        return vel_th;
    }

    double find_angle_between(double a, double b, double angle_rb){
        double angle_bt = 0.0;
        double angle_fn = 0.0;
        if (b == 0){
            if (a < 0){
                angle_bt = M_PI/2.0;
            }
            else if (a > 0){
                angle_bt = -M_PI/2.0;
            }
        }
        else if (a == 0){
            if (-b < 0){
                angle_bt = 0.0;
            }
            else if (-b > 0){
                angle_bt = M_PI;
            }
        }
        else{
            angle_bt = acos(b/sqrt(b*b + a*a));
            if (-a/b > 0){
                if (fabs(angle_bt) > M_PI/2){
                    angle_bt = -angle_bt;
                }
                else{
                    angle_bt = angle_bt;
                }
            }
            else{
                if (fabs(angle_bt) > M_PI/2){
                    angle_bt = angle_bt;
                }
                else{
                    angle_bt = -angle_bt;
                }
            }
        }

        angle_fn = angle_bt - angle_rb;
        // cout<<(angle_bt, angle_fn)
        if (fabs(angle_fn) >= M_PI){
            double angle_fnt = (2*M_PI - fabs(angle_fn));
            if (angle_fn > 0){
                angle_fn = -angle_fnt;
            }
            else{
                angle_fn = angle_fnt;
            }
        }

        // cout<<(angle_fn)
        return angle_fn;
    }

    float turn_ar(double theta, double tol_theta, float vel_rot){
        float vel_th = 0.0;
        if (fabs(theta) > tol_theta){ // +- 10 do
            if (theta > 0){ //quay trai
                cout<< "Im here! ---------------------------AGV thuc hien quay trai -----------------" << endl;
                if (fabs(theta) <= angle_giam_toc){
                    // cout<<('hhhhhhhhhhh')
                    float vel_th = (fabs(theta)/angle_giam_toc)*vel_rot;
                }
                else{
                    vel_th = vel_rot;
                }

                if (vel_th < 0.1)
                    vel_th = 0.1;

                // vel_th = fabs(theta) + 0.1
                // if vel_th > vel_rot : vel_th = vel_rot
                // return vel_th;
            }

            else if (theta < 0){ //quay phai , vel_z < 0
                // cout<< "a"
                cout<< "Im here! ---------------------------AGV thuc hien quay phai -----------------" << endl;
                if (fabs(theta) <= angle_giam_toc){
                    // cout<<('hhhhhhhhhhhh')
                    vel_th = (fabs(theta)/angle_giam_toc)*(-vel_rot);
                }
                else{
                    vel_th = -vel_rot;
                }

                if (vel_th > -0.1)
                    vel_th = -0.1;

                // vel_th = -fabs(theta) - 0.1
                // if vel_th < -vel_rot : vel_th = -vel_rot
                // return vel_th;
                // buoc = 1
            }
        }

        else{
            vel_th = -10;
            // return -10;
        }
        return vel_th;
    }

    int8_t check_safetyNAV(double timeRecieve, float timeCheck){
        double t = ros::Time::now().toSec() - timeRecieve;
        if (t >= timeCheck){
            return -1;
        }
        else{
            return dataZoneNav.data;
        }
    }

    int8_t check_safetyTIM(double timeRecieve, float timeCheck){
        double t = ros::Time::now().toSec() - timeRecieve;
        if (t >= timeCheck){
            timeZone3TIM = 0;
            timeZone2TIM = 0;
            return -1;
        }

        else{
            // return zone_lidar.zone_sick_ahead
            if (zone_lidar.zone_sick_ahead == 1){
                timeZone3TIM = 0;
                timeZone2TIM = 0;
                oldzone = 1;
                return 1;
            }
            else if (zone_lidar.zone_sick_ahead == 0){
                timeZone3TIM = 0;
                timeZone2TIM = 0;
                oldzone = 0;
                return 0;
            }

            if (is_check_zone == true){
                is_check_zone = false;

                if (zone_lidar.zone_sick_ahead == 3){
                    timeZone2TIM = 0;
                    timeZone3TIM = timeZone3TIM + 1;
                }

                else if (zone_lidar.zone_sick_ahead == 2){
                    timeZone3TIM = 0;
                    timeZone2TIM = timeZone2TIM + 1;
                }

                if (timeZone3TIM > 3){
                    oldzone = 3;
                    return 3;
                }
                else if (timeZone2TIM > 3){
                    oldzone = 2;
                    return 2;
                }
                else{
                    return oldzone;
                }
            }
            else{
                return oldzone;
            }
        }
    }

    void selectfield(int field){
        if (oldselectfield != field){
            std_msgs::Int8 dfield;
            dfield.data = field;
            for(int i = 0; i < 2; i++){
                pub_requestFields.publish(dfield);
            }

            oldselectfield = field;
        }
    }

    tuple<double, double, double> funcalptduongthang(double X_s, double Y_s, double X_f,  double Y_f){
        double _a = Y_s - Y_f;
        double _b = X_f - X_s;
        double _c = -X_s*_a -Y_s*_b;
        return make_tuple (_a, _b, _c);
    }

    int getIndex(vector<int32_t> v, int32_t K)
    {
        auto it = find(v.begin(), v.end(), K);
    
        // If element was found
        if (it != v.end()) 
        {
            // calculating the index
            // of K
            int index = it - v.begin();
            // cout << index << endl;
            return index;
        }
        else {
            // If the element is not
            // present in the vector
            // cout << "-1" << endl;
            return -1;
        }
    }

    vector<int32_t> slicing(vector<int32_t>& arr, int32_t X, int32_t Y)
    {
    
        // Starting and Ending iterators
        auto start = arr.begin() + X;
        auto end = arr.begin() + Y + 1;
    
        // To store the sliced vector
        vector<int32_t> result(Y - X + 1);
    
        // Copy vector using copy function()
        copy(start, end, result.begin());
    
        // Return the final sliced vector
        return result;
    }

    tuple<int32_t, bool> findGoalValidVS3(vector<double> list_x, vector<double> list_y, vector<int32_t> list_id, double x_robot, double y_robot){
        int32_t id = 0;
        vector<int32_t> listIDValid;
        vector<int32_t> idLineFollow;

        // tim list hop le
        for(int i = 0; i < list_id.size(); i++){
            if (list_id[i] != 0){
                listIDValid.push_back(list_id[i]);
            }
            else{
                break;
            }
        }
        
        id = listIDValid[0];
        //
        float minTotal = numeric_limits<float>::infinity();
        uint8_t numId = listIDValid.size();
        if (numId > 1){
            if (numId > 2){
                float coefficientDistance = 1.5;
                float coefficientAngle = 2.;
                float coefficientHC = 0.5;
                for(int i = 0; i < (numId-1); i++){
                    // cout<<"Point: " + to_string(i)<< endl;
                    double distanceRobotToPoint1 = fnCalcDistPoints(list_x[i], x_robot, list_y[i], y_robot);
                    double distanceRobotToPoint2 = fnCalcDistPoints(list_x[i+1], x_robot, list_y[i+1], y_robot);
                    double distancePoint1ToPoint2 = fnCalcDistPoints(list_x[i], list_x[i+1], list_y[i], list_y[i+1]);
                    double angleReferencePoint = calAngleThreePoint(x_robot, y_robot, \
                                                            list_x[i], list_y[i], \
                                                            list_x[i+1], list_y[i+1]);
                    
                    // cout<<"Truoc: " + to_string(angleReferencePoint) << endl;
                    
                    // if angleReferencePoint > 90.*PI/180.:
                    //     cout<<("aaaa")
                    angleReferencePoint = M_PI  - angleReferencePoint;
                    
                    // cout<<"Sau: " + to_string(angleReferencePoint) << endl;
                    
                    double a,b,c;
                    tie(a,b,c) = funcalptduongthang(list_x[i], list_y[i], list_x[i+1], list_y[i+1]);
                    double disH =  fabs(a*x_robot + b*y_robot + c)/sqrt(a*a + b*b);

                    double total = coefficientDistance*(distanceRobotToPoint1 + distanceRobotToPoint2 - distancePoint1ToPoint2) + coefficientAngle*angleReferencePoint + coefficientHC*disH;
                    // cout<<"Total: " + to_string(total) << endl;
                    if (total <= minTotal){
                        minTotal = total;
                        idLineFollow = slicing(listIDValid, i, i+2);
                    }
                }
            }

            else{
                idLineFollow = listIDValid;
            }

            // cout << idLineFollow << endl;
            
            id = idLineFollow[1];
            double angleReferencePoint = calAngleThreePoint(x_robot, y_robot, \
                                                        list_x[getIndex(list_id, idLineFollow[0])], list_y[getIndex(list_id, idLineFollow[0])], \
                                                        list_x[getIndex(list_id, idLineFollow[1])], list_y[getIndex(list_id, idLineFollow[1])]);
            
            if (angleReferencePoint >= 90.*M_PI/180.){
                id = idLineFollow[0];
            }
        }

        bool id_bool = (getIndex(list_id, id) == (listIDValid.size() - 1))? true : false;
        return make_tuple(id, id_bool);
    }

    void run(){
        if (process == 0){
            ROS_INFO(".......... Start STI Navigation .........");
            // zonetim = check_safetyTIM(timeRecieveTIM, timeWaitTIM)
            // cout<<(zonetim)
            process = 1;
            reset_all();
        }

        // kiem tra du lieu dau vao
        else if (process == 1){
            uint8_t c_k = 0;
            if (is_request_move == true)
                c_k = c_k + 1;
            else{
                cout<<"Wait command from STI_Control" << c_k << endl;
            }

            if (is_pose_robot == true)
                c_k = c_k + 1;
            else{
                cout<<"Wait data from STI_Getpose" << c_k << endl;
            }

            if (is_check_zone == true)
                c_k = c_k + 1;
            else{
                cout<<"Wait data from STI_Zone_lidar" << c_k << endl;
            }

            if (is_ZoneNav == true)
                c_k = c_k + 1;
            else{
                cout<<"Wait data from STI_Zone_Nav" << c_k << endl;
            }

            if (c_k == 4){
                ROS_INFO("Completed wakeup ('_')");
                process = 2;
            }
        }
            // process = 2
        // -- RUN main
        else if (process == 2){  // kiem tra udp client co cho phep di chuyen.
            if (req_move.enable == 0){ // reset all
                cout << "I'm here!----------------reset all ----------------" << endl;
                process = -3;
                mission = 0;
            }

            else if (req_move.enable == 1 || req_move.enable == 3){ // run
                if (req_move.enable == 1){
                    mission = 1;
                    selectfield(0);
                }
                else{
                    // cout << "I'm here!--------------enable = 3 / process = 2---------------------" << endl;
                    mission = 3;
                    selectfield(1);
                }
                // cout << "I'm here!-----------------------------------" << endl;    
                process = 3;
            }
                
            else if (req_move.enable == 2){ //yeu cau lui
                process = 50;
                mission = 2;
            }
        }

        else if (process == 3){
            // cout << "request move message " << req_move.target_x << " " << req_move.target_y << endl;
            // cout << "target msg " << target_x << " " << target_y << endl;
            if(req_move.target_x < 500.0 && req_move.target_y < 500.0){
                
                if (( round_3decimal(req_move.target_x) != round_3decimal(target_x) ) || ( round_3decimal(req_move.target_y) != round_3decimal(target_y) )){ // neu co thay doi diem dich
                    stop();
                    uint8_t dk = update_all();
                    if (dk == 1){
                        // ROS_INFO("Change target from: X = %s |Y = %s to: X = %s |Y = %s ", target_x, target_y ,req_move.target_x, req_move.target_y)
                        // cout << "Change target from: X = "<< target_x << " |Y= " << target_y << " to: X = " << req_move.target_x << " |Y = " << req_move.target_y << endl;
                        process = 4;
                    }
                    else{
                        ROS_INFO("Something Wrong!, wait the update Target");
                        process = 2;
                    }
                }

                else{
                    is_target_change = false;
                    process = 4;
                }
            }
                
            else{
                if (stt_agv == 0){
                    error = 1;
                    ROS_INFO("Target = 0.0, wait the update Target");
                    process = 2;
                }

                else{
                    process = 4;
                }
            }
        }

        else if (process == 4){
            if (completed_all != 1){ // agv da di den dich cuoi.
                error = 0;
                if (completed_simple != 1){   // chua hoan thanh di chuyen diem thuong
                    process = 5;      // simple point
                }

                else{
                    process = 8;
                    cout<< "hoan thanh di chuyen diem thuong, dap ung goc cuoi 0" << endl;
                }
            }

            else{
                error = 0;
                stt_agv = 0;
                cout<< "Wait new target" + to_string(0) << endl;
                process = 2;
            }
        }

        // tim ra current goal, next goal
        else if (process == 5){
            // update list x,y
            if (req_move.list_x.size() == 5 && req_move.list_y.size() == 5 && req_move.list_id.size() == 5 && req_move.list_speed.size() == 5){
                try{
                    if (((list_x != req_move.list_x) || (list_y != req_move.list_y) || (list_id != req_move.list_id)) && req_move.list_id[0] != 0.0){
                        list_x = req_move.list_x;
                        list_y = req_move.list_y;
                        // cout<<(list_x)
                        list_id = req_move.list_id;
                        list_vel = req_move.list_speed;
                        end_of_list = false;

                        path_plan.poses.empty();
                        path_plan.poses.push_back(point_path(poseRbMa.position.x, poseRbMa.position.y));

                        for (int i = 0; i < list_x.size(); i++){
                            if (list_id[i] != 0.0){
                                geometry_msgs::PoseStamped point;
                                point.header.frame_id = "frame_map_nav350";
                                point.header.stamp = ros::Time::now();
                                point.pose.position.x = list_x[i];
                                point.pose.position.y = list_y[i];
                                point.pose.position.z = -1.0;
                                point.pose.orientation.w = 1.0;
                                // cout<<(point)
                                path_plan.poses.push_back(point);
                            }
                            else{
                                break;
                            }
                        }

                        pub_path_global.publish(path_plan);
                    }
                }
                catch(...){
                    cout<<"Update list Unknow!!!"<<endl;
                }
            }
                    
            else{
                ROS_INFO("SOME LIST WRONG!!!");
            }


            if (stt_agv == 0 ){ // AGV tinh goal tiep theo sau khi dung
                // cout<<("vao day nay")

                if (end_of_list == true){
                    // cout<<("aaaaaaaaaaaaaaaaaaaaaa")
                    process = 2;
                }

                else{
                    if (is_target_change == true){
                        is_target_change = false;
                        ROS_INFO("kiem tra bat dau");
                        if (id_fl != 0.0){   
                            // --- Them vao ngay 12/4/2023 fix loi quay dau
                            int32_t idFollowValid;
                            bool isIDEndList;
                            tie(idFollowValid, isIDEndList) = findGoalValidVS3(list_x, list_y, list_id, poseRbMa.position.x, poseRbMa.position.y);
                            cout<< "ID follow is " << idFollowValid << " | is ID END: " << isIDEndList << endl;
                            int8_t indexOfId = getIndex(list_id, idFollowValid);
                            cur_goal_x = req_move.list_x[indexOfId];
                            cur_goal_y = req_move.list_y[indexOfId];
                            id_fl = req_move.list_id[indexOfId];
                            vel_fl = req_move.list_speed[indexOfId];

                            double dist_debug = fnCalcDistPoints(poseRbMa.position.x, cur_goal_x, poseRbMa.position.y, cur_goal_y);
                            // cout << "Khoang cach dist_debug is " << dist_debug << "  " << ss_luivsnextGoal << endl;
                            cout << "robot x is " << poseRbMa.position.x << " robot y is " << poseRbMa.position.y << endl;
                            cout << "current goal x is " << cur_goal_x << " curr goal is " << cur_goal_y << endl;
                            if (fnCalcDistPoints(poseRbMa.position.x, cur_goal_x, poseRbMa.position.y, cur_goal_y) > ss_luivsnextGoal){
                                is_need_turn_step1 = 1;
                                is_need_pttt = 1;
                                point_goal_start_x = poseRbMa.position.x;
                                point_goal_start_y = poseRbMa.position.y;
                                process = 6;

                                // cout << "Im here! -------------------process = 5 number 1---------------" << endl;
                            }

                            else{
                                ROS_INFO("chon goal tiep theo lam cur goal");
                                
                                if (round_3decimal(cur_goal_x) == round_3decimal(target_x) && round_3decimal(cur_goal_y) == round_3decimal(target_y)){
                                    if (need_check_goal == true){
                                        is_need_turn_step1 = 1;
                                        is_need_pttt = 1;
                                        point_goal_start_x = poseRbMa.position.x;
                                        point_goal_start_y = poseRbMa.position.y;
                                        process = 6;
                                        // cout << "Im here! -------------------process = 5 number 2---------------" << endl;
                                    }
                                    
                                    else{
                                        //----- them xoay dap ung goc cuoi
                                        is_over_goal = false;
                                        is_need_pttt = 0;
                                        is_pre_pttt = 0;
                                        ROS_INFO("da den goal cuoi roi roi!!!!!!");
                                        stop();
                                        completed_simple = 1;
                                        process = 8;
                                    }
                                }
                                        
                                else{	
                                    if (!isIDEndList){ 
                                    // if list_id[1] != 0.0){
                                        is_need_turn_step1 = 1;
                                        // cout << "Im here! -------------------process = 5 number 3---------------" << endl;
                                        is_need_pttt = 1;
                                        point_goal_start_x = poseRbMa.position.x;
                                        point_goal_start_y = poseRbMa.position.y;
                                        // point_goal_start_x = list_x[0]
                                        // point_goal_start_y = list_y[0]
                                        cur_goal_x = list_x[indexOfId+1];
                                        cur_goal_y = list_y[indexOfId+1];
                                        id_fl = list_id[indexOfId+1];
                                        vel_fl = list_vel[indexOfId+1];
                                        process = 6;
                                    }
                                    
                                    else{
                                        end_of_list = true;
                                        process = 2;
                                    }
                                }
                            }
                        }

                        else{
                            end_of_list = true;
                            process = 2;
                        }

                    }
                        
                                                
                    else{
                        int f = find_xy(cur_goal_x, cur_goal_y, list_x, list_y);
                        // cout<<(f,cur_goal_x,cur_goal_y,list_x,list_y)   
                        if (f == -1){
                            // cout<<('aaa')
                            if (list_id[0] != 0.0){
                                is_need_turn_step1 = 1;
                                is_need_pttt = 1;
                                point_goal_start_x = poseRbMa.position.x;
                                point_goal_start_y = poseRbMa.position.y;

                                cur_goal_x = list_x[0];
                                cur_goal_y = list_y[0];
                                id_fl = list_id[0];
                                vel_fl = list_vel[0];
                                // cout << "Im here! -------------------process = 5 number 4---------------" << endl;
                                
                                process = 6;
                            }

                            else{
                                end_of_list = true;
                                process = 2;
                            }
                        }

                        else{
                            int f_next = f + 1; // diem tiep theo
                            if (f_next > 4){
                                f_next = 4;
                            }

                            if (list_id[f_next] != 0.0){
                                is_need_turn_step1 = 1;
                                is_need_pttt = 1;
                                point_goal_start_x = poseRbMa.position.x;
                                point_goal_start_y = poseRbMa.position.y;

                                cur_goal_x = list_x[f_next];
                                cur_goal_y = list_y[f_next];
                                id_fl = list_id[f_next];
                                vel_fl = list_vel[f_next];
                                // cout << "Im here! -------------------process = 5 number 5---------------" << endl;
                                process = 6;
                            }

                            else{
                                end_of_list = true;
                                process = 2;
                            }
                        }
                    }
                }
            }

            else if (stt_agv == 1){
                
                double kc = fnCalcDistPoints(poseRbMa.position.x,cur_goal_x,poseRbMa.position.y,cur_goal_y);
                // cout<<('aaa')  
                // if kc > distance_goArc){
                if (kc > distance_goArc){
                    check_goArc = 1;
                }   
                    
                if ((kc < khoang_offset || (kc < distance_goArc && check_goArc == 1)) && cur_goal_is != 2 ){
                    if ((round_3decimal(cur_goal_x) == round_3decimal(target_x)) && (round_3decimal(cur_goal_y) == round_3decimal(target_y))){
                        cur_goal_is = 3;
                    }

                    else{
                        // cout<<('aaa')
                        int f = find_xy(cur_goal_x, cur_goal_y, list_x, list_y);
                        if (f == -1){
                            if (list_id[0] != 0.0){
                                int temp = find_point_special(cur_goal_x,cur_goal_y,list_x[0],list_y[0],check_goArc);
                                if (temp == 2){
                                    cur_goal_is = 2;
                                }
                                    
                                else if (temp == 1){
                                    cur_goal_is = 1;
                                    point_goal_start_x = cur_goal_x;
                                    point_goal_start_y = cur_goal_y;
                                    cur_goal_x = list_x[0];
                                    cur_goal_y = list_y[0];
                                    id_fl = list_id[0];
                                    vel_fl = list_vel[0];
                                }
                                    
                                else if (temp == 3){
                                    ROS_INFO("CHECK NEXT POINT GO ARC!!!");
                                    if (list_id[1] != 0.0){
                                        double dist = fnCalcDistPoints(cur_goal_x, list_x[1], cur_goal_y, list_y[1]);
                                        double goc = calAngleThreePoint(cur_goal_x, cur_goal_y, list_x[0], list_y[0], list_x[1], list_y[1]);
                                        if (dist > 3.0 && goc > 165.0*M_PI/180.0){
                                            cur_goal_is = 1;
                                            point_goal_start_x = cur_goal_x;
                                            point_goal_start_y = cur_goal_y;
                                            cur_goal_x = list_x[1];
                                            cur_goal_y = list_y[1];
                                            id_fl = list_id[1];
                                            vel_fl = list_vel[1];
                                        }
                                    }
                                    else{
                                        ROS_INFO("OUT CHECK GO ARC!!!, MODE 3");
                                    }
                                }
                                else{
                                    ROS_INFO("OUT CHECK GO ARC!!!");
                                }
                            }
                            else{
                                cur_goal_is = 2;
                                end_of_list = true;
                            }
                        }

                        else{
                            if (f >= 4){
                                cur_goal_is = 2;
                                end_of_list = true;
                            }

                            else{
                                int f_next = f + 1; // diem tiep theo
                                // if f_next > 4){
                                //     f_next = 4
                                if (list_id[f_next] != 0.0){   
                                    int temp = find_point_special(cur_goal_x,cur_goal_y,list_x[f_next],list_y[f_next],check_goArc);
                                    if (temp == 2){
                                        cur_goal_is = 2;
                                    }
                                    else if (temp == 1){
                                        point_goal_start_x = cur_goal_x;
                                        point_goal_start_y = cur_goal_y;

                                        cur_goal_x = list_x[f_next];
                                        cur_goal_y = list_y[f_next];
                                        id_fl = list_id[f_next];
                                        vel_fl = list_vel[f_next];
                                    }
                                        
                                    else if (temp == 3){
                                        ROS_INFO("CHECK NEXT POINT GO ARC!!!");
                                        if (f_next <= 3 && list_id[f_next + 1] != 0.0){
                                            double dist = fnCalcDistPoints(cur_goal_x, list_x[f_next + 1], cur_goal_y, list_y[f_next + 1]);
                                            double goc = calAngleThreePoint(cur_goal_x, cur_goal_y, list_x[f_next], list_y[f_next], list_x[f_next + 1], list_y[f_next + 1]);
                                            if (dist > 3.0 && goc > 170.0*M_PI/180.0){
                                                cur_goal_is = 1;
                                                point_goal_start_x = cur_goal_x;
                                                point_goal_start_y = cur_goal_y;
                                                cur_goal_x = list_x[f_next + 1];
                                                cur_goal_y = list_y[f_next + 1];
                                                id_fl = list_id[f_next + 1];
                                                vel_fl = list_vel[f_next + 1];
                                            }
                                        }
                                        else{
                                            ROS_INFO("OUT CHECK GO ARC!!!, MODE 3");
                                        }
                                    }

                                    else{
                                        ROS_INFO("OUT CHECK GO ARC!!!");
                                    }
                                }

                                else{
                                    // cout<<("phat cuoi vao day")
                                    cur_goal_is = 2;
                                    end_of_list = true;
                                }
                            }
                        }
                    }
                                    
                    check_goArc = 0;
                }
                
                process = 6;
            }

            else if (stt_agv == 3 ){
                process = 6;
            }
        }

        else if (process == 6){   // quay trước khi di chuyển
            if (id_fl != 0.0){
                error = 0;
                if (is_need_turn_step1 == 1){
                    stt_agv = 3;
                    // quay agv toi goal
                    double theta_poin = atan2(cur_goal_y - poseRbMa.position.y, \
                                        cur_goal_x - poseRbMa.position.x );

                    double a = poseRbMa.position.y - cur_goal_y;
                    double b = cur_goal_x - poseRbMa.position.x;

                    double theta = find_angle_between(a, b, theta_rb_ht);
                    // cout << "theta Angle is " << theta << " " << tolerance_rot_step1 << "  " << vel_rot_step1 << endl;

                    if (zone_lidar.zone_sick_ahead == 1 || zone_lidar.zone_sick_behind != 0){
                        war_agv = 1;
                        ROS_INFO("co vat can o vung tron truoc sau");
                        stop();
                    }
                    else{
                        war_agv = 0;
                        double gt = turn_ar(theta, tolerance_rot_step1, vel_rot_step1);
                        if (gt == -10){
                            stop();
                            ros::Duration(0.3).sleep();
                            stt_agv = 1;
                            is_need_turn_step1 = 0;
                            time_start_navi = ros::Time::now().toSec();
                            // cout<< "Im here !----------------gt = - 10------------------" << endl;
                        }

                        else{
                            geometry_msgs::Twist twist;
                            twist.angular.z = gt;
                            // cout<< "Im here !----------------AGV đang thuc hien quay------------------" << endl;
                            pub_cmdVel(twist, rate_cmdvel);
                        }
                    }

                    process = 2;
                }
                        
                else{
                    process = 7;
                }
            }

            else{
                stop();
                cout<<"Stop send Goal because Now Goal Unknown" << cur_goal_x << endl; 
                error = 2;
                process = 2;
            }
        }
        //----- dieu huong AGV
        else if (process == 7){

            path_plan.poses.push_back(point_path(poseRbMa.position.x, poseRbMa.position.y));
            path_plan.poses.push_back(point_path(cur_goal_x,cur_goal_y));
            pub_path_local.publish(path_plan);
            path_plan.poses.empty();

            double arg, b_rg;
            tie(X_n, Y_n, a_qd, b_qd, c_qd, dis_hc, arg, b_rg) = find_hc(point_goal_start_x,\
                                                                                            point_goal_start_y,\
                                                                                            cur_goal_x,\
                                                                                            cur_goal_y);

            theta = find_angle_between(a_qd, b_qd, theta_rb_ht);
            // cout<<(theta)
            // cout<<("Theta= %s, agnle_find_vel= %s" %(fabs(theta) ,angle_find_vel))
            is_target = 0;
            if ((round_3decimal(cur_goal_x) != round_3decimal(target_x)) && (round_3decimal(cur_goal_y) != round_3decimal(target_y))){
                is_target = 0;
            }
            else{
                is_target = 1;
            }

            tie(x_td_goal, y_td_goal, kc_con_lai, kc_qd, is_over_goal) = find_point_goal(point_goal_start_x,\
                                                                                                            point_goal_start_y,\
                                                                                                            cur_goal_x,\
                                                                                                            cur_goal_y,\
                                                                                                            a_qd,b_qd,c_qd,\
                                                                                                            X_n,Y_n,\
                                                                                                            is_target);
            // angle_Mode = atan2(y_td_goal,x_td_goal)
            // if fabs(angle_Mode) > M_PI/2.0){

            distance_goal = fnCalcDistPoints(poseRbMa.position.x,\
                                                                cur_goal_x,\
                                                                poseRbMa.position.y,\
                                                                cur_goal_y);

            // cout<<("Mode Target= %s, dis_hc= %s , x_now= %s, y_now= %s, distance_goal= %s, kc_conlai= %s" %(is_target ,dis_hc, poseRbMa.position.x, poseRbMa.position.y, distance_goal, kc_con_lai))
            cout << "Mode Target= " << is_target << " dis_hc " << dis_hc << " x_now= " << poseRbMa.position.x << " y_now= " << poseRbMa.position.y << " distance_goal= " << distance_goal << " kc_conlai= " << kc_con_lai << endl;
            // theta = theta_poin - theta_rb_ht
            
            // vel_x_control = 0.45
            // new traffic
            if (vel_fl == 0){ // roi vao th khong xac dinh
                vel_x_control = 0.3;
            }
                
            else{
                vel_x_control = round_3decimal((vel_fl/120.0)*vel_x_max);
            }
                
            if (vel_x_control > vel_x_max){
                vel_x_control = vel_x_max;
            }

            float v_x = 0.0;
            if (fabs(theta) > angle_find_vel){
                v_x = min_vel_x_gh;
            }

            else if (round_3decimal(fabs(theta)) == 0.0){
                v_x = vel_x_control;
            }

            else{
                v_x = min_vel_x_gh + ((angle_find_vel - fabs(theta))/angle_find_vel)*(vel_x_control - min_vel_x_gh);
            }

            // cout<<(v_x)
            float v_x_send = 0.0;
            if (is_need_pttt == 1 && distance_goal > dis_gt){
                is_pre_pttt = 1;
                vel_x_now = ptgt(4.0,time_start_navi,0.0, v_x);
                v_x_send = vel_x_now;
                if (v_x_send >= v_x){
                    is_need_pttt = 0;
                    is_pre_pttt = 0;
                    v_x_send = v_x;
                    // cout<<("1")
                }
            }

            else{
                is_need_pttt = 0;
                if (is_pre_pttt == 1 && vel_x_now >= 0.3){
                    v_x_send = vel_x_now*(distance_goal/dis_gt);
                    // cout<<("2")
                }
                else{
                    is_pre_pttt = 0;
                    v_x_send = v_x*(distance_goal/dis_gt);
                    // cout<<("3")
                }
                    
                if (v_x_send > v_x){
                    v_x_send = v_x;
                }
            }
                        
            // cout<<("v_x= %s, vel_x= %s" %(v_x,v_x_send))


            if (cur_goal_is == 2 && (distance_goal <= tol_simple || kc_con_lai <= tol_simple || is_over_goal == true) ){
                cur_goal_is = 0;
                is_over_goal = false;
                is_need_pttt = 0;
                is_pre_pttt = 0;
                ROS_INFO("da den goal trung gian roi!!!!!!");
                stop();

                stt_agv = 0;
                process = 2;
            }

            else if (cur_goal_is == 3 && (distance_goal <= tol_target || kc_con_lai <= tol_target || is_over_goal == true) ){
                //----- them xoay dap ung goc cuoi
                is_target = 0;
                is_over_goal = false;
                is_need_pttt = 0;
                is_pre_pttt = 0;
                ROS_INFO("da den goal cuoi roi roi!!!!!!");
                stop();
                completed_simple = 1;
                process = 8;
                usleep(600000);
            }

            else{
                stt_agv = 1;

                int8_t zonetim = check_safetyTIM(timeRecieveTIM, timeWaitTIM);
                int8_t zonenav = check_safetyNAV(timeRecieveNAV, timeWaitNAV);
                // cout<<(zonetim)
                if (zonetim == -1 || zonenav == -1){
                    stop();
                    // ROS_INFO('khong nhan duoc du lieu laser')
                    error = 3;
                    process = 2;
                }

                else{
                    error = 0;
                    // if zonetim == 1 || zonenav == 1){
                    if (zone_lidar.zone_sick_ahead == 1 || zone_lidar.zone_sick_ahead == 2 || zonenav == 1){
                        stop();
                        is_need_pttt = 1;
                        is_pre_pttt = 0;
                        time_start_navi = ros::Time::now().toSec();
                        war_agv = 1;
                    }

                    else{
                        float vel_x = 0.0;
                        // if zonetim == 3){
                        if (zone_lidar.zone_sick_ahead == 3){
                            war_agv = 2;
                            vel_x = v_x_send*0.45;
                        }

                        else{
                            war_agv = 0;
                            vel_x = v_x_send;
                        }

                        if (vel_x >= vel_x_max){
                            vel_x = vel_x_max;
                        }

                        if (vel_x <= min_vel_x){
                            vel_x = min_vel_x;
                        }
                            
                        // cout<<("v_x_send= %s, vel_x= %s" %(v_x_send,vel_x))
                            
                        float v_th_send = 0.0;
                        if (is_target == 0){ 
                            v_th_send = control_navigation(x_td_goal, y_td_goal,vel_x,theta, dis_hc);
                        }
                        else{
                            v_th_send = control_naviTarget(x_td_goal, y_td_goal);
                        }
                        // cout<<('v_dai = %f, v_goc = %f' %(vel_x,v_th_send))
                        geometry_msgs::Twist twist;
                        twist.linear.x = vel_x;
                        twist.angular.z = v_th_send;
                        pub_cmdVel(twist,rate_cmdvel);
                    }
                    process = 2;
                }
            }
        }


        else if (process == 8){
                
            if (needRotatyFinish == 1){
                double theta = target_z - theta_rb_ht;
                // cout<<(angle_bt, angle_fn)
                if (fabs(theta) >= M_PI){
                    double theta_t = (2*M_PI - fabs(theta));
                    if (theta > 0){
                        theta = -theta_t;
                    }
                    else{
                        theta = theta_t;
                    }
                }

                if (zone_lidar.zone_sick_ahead == 1 || zone_lidar.zone_sick_behind != 0){
                    war_agv = 1;
                    // ROS_INFO('co vat can o vung tron')
                    stop();
                }

                else{
                        
                    war_agv = 0;
                    double gt = turn_ar(theta,tolerance_theta,vel_rot_step_f);
                    if (gt == -10){
                        stop();
                        ros::Duration(0.3).sleep();
                        stt_agv = 0;
                        completed_all = 1;
                        path_plan.poses.empty();
                        pub_path_global.publish(path_plan);
                        process = 2;
                    }
                    else{
                        geometry_msgs::Twist twist;
                        twist.angular.z = gt;
                        pub_cmdVel(twist,rate_cmdvel);
                        stt_agv = 3;
                    }
                }
            }
            else{
                    
                war_agv = 0;
                stop();
                stt_agv = 0;
                completed_all = 1;
                path_plan.poses.empty();
                pub_path_global.publish(path_plan);
                process = 2;
            }

            process = 2;
        }
        
        else if (process == -3){   // RESET: khong cho phep di chuyen -reset all.
            if (completed_reset == 0){
                stop();
                reset_all();
                completed_reset = 1;
                process = 2;
                // cout<< "Im here ! ------------------------reset at mode -3------------------------" << endl;
            }

            else{
                stt_agv = 0;
                cout<< "Wait new misson" << endl;
                process = 2;
            }
        }
                
        else if (process == 50){
            if (completed_backward == 0){
                completed_reset = 0;
                if (stt_agv == 0){ // agv chua di chuyen
                    if (req_move.target_x < 500 && req_move.target_y < 500){
                        error = 0;
                        selectfield(0);

                        kc_backward = fnCalcDistPoints(poseRbMa.position.x,\
                                                                req_move.target_x,\
                                                                poseRbMa.position.y,\
                                                                req_move.target_y );

                        if (fabs(kc_backward) > gioihan_lui ){
                            kc_backward = gioihan_lui;
                        }

                        // odom_x_ht = odom_rb.pose.pose.position.x
                        // odom_y_ht = odom_rb.pose.pose.position.y

                        XRobotStart = poseRbMa.position.x;
                        YRobotStart = poseRbMa.position.y;

                        process = 51;
                    }

                    else{
                        stop();
                        // ROS_INFO('target khong hop le')
                        error = 1; // loi target ko hop le
                        process = 2;
                    }
                }

                else if (stt_agv == 2){ // agv dang lui
                    process = 51;
                }

                else if (stt_agv == -1){
                    process = 2;
                }
            }

            else{
                stt_agv = 0;
                // cout<<("info", "Wait new target",0)
                process = 2;
            }
        }

        else if (process == 51){
            geometry_msgs::Twist twist;
            error = 0;
            // s = fnCalcDistPoints(odom_rb.pose.pose.position.x,odom_x_ht,odom_rb.pose.pose.position.y,odom_y_ht)
            double s_nav = fnCalcDistPoints(poseRbMa.position.x,XRobotStart,poseRbMa.position.y,YRobotStart);
            if (s_nav < fabs(kc_backward)){
                stt_agv = 2;
                int8_t zonetim = check_safetyTIM(timeRecieveTIM, timeWaitTIM);
                int8_t zonenav = check_safetyNAV(timeRecieveNAV, timeWaitNAV);
                // cout<<(zonetim)

                // -- Edit 14/04/2022
                // if zonetim == -1 || zonenav == -1){
                //     stop()
                //     error = 3

                // else){
                // if zone_lidar.zone_sick_ahead == 1 || zone_lidar.zone_sick_ahead == 2 || zonenav == 1){
                if (zone_lidar.zone_sick_ahead == 1 || zonenav == 1){
                    cout<<("co vat can")<< endl;
                    war_agv = 1;
                    stop();
                }

                else{
                    war_agv = 0;
                    float vel_lui = (fabs((fabs(kc_backward)- s_nav))/dis_gt_khilui)*0.16;
                    if (vel_lui > 0.16){
                        vel_lui = 0.16;
                    } 
                    if (vel_lui < 0.1){
                        vel_lui = 0.1;
                    }
                    twist.linear.x = vel_lui;
                    pub_cmdVel(twist,rate_cmdvel);
                }
            }

            else{
                stop();
                stt_agv = 0;
                completed_backward = 1;
            }

            process = 2;
        }

        // cout << "Current mission is " << mission << endl;
        if (mission == 0){
            pub_status(mission,0,0,0,0,0);
        }

        else if (mission == 1 || mission == 3){
            pub_status(mission, stt_agv, error, war_agv, completed_all, id_fl);
        }

        else if (mission == 2){
            pub_status(mission,stt_agv,error,war_agv,completed_backward,id_fl);
        }          
    }
};

int main(int argc, char **argv)
{
    cout << "Program start!";

    ros::init(argc, argv, "goalControl_cpp");
    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle("~");
    ros::Rate loop_rate(50);

    goalControl self = goalControl(&nh, &private_node_handle);

    while(ros::ok()){
        self.run();
        ros::spinOnce();     // allow receiving callbacks function
        loop_rate.sleep();
    }

    return 0;
}
