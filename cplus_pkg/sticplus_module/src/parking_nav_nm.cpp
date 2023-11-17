// Author : Phùng Quý Dương 
// Date: 21-09-2023

/*
    - Node Parking_nav_cpp 
    - function:
        + Điều khiển AGV lùi vào lấy kệ
*/

#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include "nav_msgs/Odometry.h"

#include "sti_msgs/HC_info.h"

#include "message_pkg/Parking_request.h"
#include "message_pkg/Parking_respond.h"

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

using namespace std;

class ParkingNAV{
    public:
    string rate_str;
    int rate;
    // ros::Rate loop_rate(int x);
    ros::Subscriber sub_ParkingRequest;
    message_pkg::Parking_request req_parking;
    bool is_request_parking;

    ros::Subscriber sub_robotPose;
    bool is_pose_robot;
    geometry_msgs::Pose poseRbMa;
    geometry_msgs::PoseStamped poseStampedAGV;
    float theta_robotNow;

    ros::Subscriber sub_GetRobotOdom;
    bool is_odom_rb;
    nav_msgs::Odometry odom_rb;

    ros::Subscriber sub_HCinfo;
    sti_msgs::HC_info zone_lidar;
    bool is_check_zone;

    ros::Publisher pub_cmd_vel;
    double time_tr;
    uint8_t rate_pubVel;

    ros::Publisher pub_ParkingRespond;
    // message_pkg::Parking_respond data_PubRespond;
    double timePubRespond;
    uint8_t rate_pubRespond;
    uint8_t auto_reset;

    //tf
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener;

    double odom_x_ht;
    double odom_y_ht;
    double odom_g;

    int8_t step_moveForward;
    double x_odom_start;
    double y_odom_start; 

    int8_t step_Rotary;
    double angle_odom_start;

    float min_vel;
    float min_velFinish;
    float min_rol;
    float min_rolFinish;

    float max_vel;
    float max_rol;

    bool pubTransform;
    int16_t process;
    
    double ss_x;
    double ss_y;
    double ss_a;
        
    int8_t warn_agv;
    // geometry_msgs::Pose poseThenTransform;
    int time_waitTransfrom;
    // double time_startTransform;
    bool is_transform;

    bool shutdown_flag;
    geometry_msgs::Pose posetf;
    double yawtf;

    void request_callback(const message_pkg::Parking_request data){
        req_parking = data;
        is_request_parking = true;
    }

    void getPose(const geometry_msgs::PoseStamped data){
        poseStampedAGV = data;
        poseRbMa = data.pose;
        // quata = ( poseRbMa.orientation.x,\
        //         poseRbMa.orientation.y,\
        //         poseRbMa.orientation.z,\
        //         poseRbMa.orientation.w )
        // euler = euler_from_quaternion(quata)
        // theta_rb_ht = euler[2]
        is_pose_robot = true;
    }

    void cbGetRobotOdom(nav_msgs::Odometry msg){

        tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        odom_x_ht = msg.pose.pose.position.x;
        odom_y_ht = msg.pose.pose.position.y;

        odom_g = yaw;
        is_odom_rb = true;
    }

    void pub_cmdVel(geometry_msgs::Twist twist, uint8_t rate){
        if ((ros::Time::now().toSec() - time_tr) > float(1/rate)){    // < 20Hz
            time_tr = ros::Time::now().toSec();
            pub_cmd_vel.publish(twist);
        }
    }

    void zone_callback(const sti_msgs::HC_info data){
        zone_lidar = data;
        is_check_zone = true;
    }

    void pub_Stop(){
        geometry_msgs::Twist twist;
        for(int i = 0; i < 3; i++){
            pub_cmd_vel.publish(twist);
        }
    }

    float constrain(float val, float min_val, float max_val){
        if (val < min_val) return min_val;
        if (val > max_val) return max_val;
        return val;
    }

    float calculate_distance(double x1, double y1, double x2, double y2){
        double x,y;
        x = x2 - x1;
        y = y2 - y1;
        return sqrt(x*x + y*y);
    }

    string Meaning(int16_t stt, int8_t war){
        string mess_step, mess_warn;
        string mess;
        
        switch(stt){
            case 0:
                mess_step = "Step: Wait All data";
                break;
            case 1:
                mess_step = "Step: Select Mode";
                break;            
            case 21:
                mess_step = "Tinh quang duong di chuyen";
                break;
            case -21:
                mess_step = "Di chuyen vao duong thang target";
                break;   
            case -210:
                mess_step = "Quay goc truoc khi tinh chinh khoang cach neu goc di chuyen lon";
                break;
            case 31:
                mess_step = "Tinh goc can quay";
                break;            
            case -31:
                mess_step = "Quay vao duong thang target";
                break;
            case 41:
                mess_step = "Parking vao ke";
                break;
            case 51:
                mess_step = "Doi reset";
                break;
            default:
                break;
        }

        switch (war){
            case 0:
                mess_warn = "AGV di chuyen binh thuong :)";
                break;
            case 1:
                mess_warn = "AGV gap vat can :(";
                break;
            case 2:
                mess_warn = "AGV gap loi chuong trinh";
                break;
            default:
                break;
        }

        mess = mess_step + " | " + mess_warn;
        return mess;
    }

    void pub_Status(int8_t stt, int8_t modeRun, geometry_msgs::Pose poseTf, geometry_msgs::Pose pose, double offset, double ss_x, double ss_y, double ss_a, string meaning, int8_t warn){
        message_pkg::Parking_respond mess;
        mess.status = stt;
        mess.warning = warn;
        mess.modeRun = modeRun;
        mess.poseBefore = poseTf;
        mess.poseTarget = pose;
        mess.offset = offset;
        mess.ss_x = ss_x;
        mess.ss_y = ss_y;
        mess.ss_a = ss_a;
        mess.message = meaning;

        if ((ros::Time::now().toSec() - timePubRespond) > float(1/rate_pubRespond)){   // < 20hz 
            timePubRespond = ros::Time::now().toSec();
            pub_ParkingRespond.publish(mess);
        }
    }

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

    void sendTransform(string frame_world, string frame_id_pointTarget, geometry_msgs::Pose point_target){
        // time_startTransform = ros::Time::now().toSec();
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(point_target.position.x, point_target.position.y, -1.0));

        // tf::Quaternion q(point_target.orientation.x, point_target.orientation.y, \
        //                     point_target.orientation.z, point_target.orientation.w);
        // tf::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);
        double yaw = quaternion_to_euler(point_target.orientation);

        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_world, frame_id_pointTarget));
    }

    geometry_msgs::Pose transformPoseNAV(string frame_id_pointTarget, string frame_agvNAV){
        geometry_msgs::Pose poseThenTransform;
        tf::StampedTransform transform;
        try{
            tf_listener.waitForTransform(frame_id_pointTarget, frame_agvNAV, ros::Time(0), ros::Duration(10.0));
            tf_listener.lookupTransform(frame_id_pointTarget, frame_agvNAV, ros::Time(0), transform);
            poseThenTransform.position.x = transform.getOrigin().x();
            poseThenTransform.position.y = transform.getOrigin().y();
            poseThenTransform.position.z = transform.getOrigin().z();

            tf::Quaternion q = transform.getRotation();
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            poseThenTransform.orientation = euler_to_quaternion(yaw);
            ss_x = transform.getOrigin().x();                         
            ss_y = transform.getOrigin().y();
            ss_a = 180 - fabs(yaw)*(180/M_PI);
            yawtf = yaw;
        }
        catch (...){

        }

        return poseThenTransform;
    }

    int8_t move_forward(double S, int8_t direct, float velocity_min, float permission_tolerance){
        geometry_msgs::Twist twist;
        if (step_moveForward == 0){
            x_odom_start = odom_x_ht;
            y_odom_start = odom_y_ht;
            step_moveForward = 1;
        }

        else if (step_moveForward == 1){
            if (req_parking.modeRun == 1 || req_parking.modeRun == 2 || req_parking.modeRun == 3){
                float S_moved = calculate_distance(odom_x_ht, odom_y_ht, x_odom_start, y_odom_start);
                if (fabs(S_moved - S) >= permission_tolerance && (S_moved - S) <= permission_tolerance){
                    twist.angular.z = 0;
                    // vel = velocity_max*(fabs(S_moved - S)/S)
                    float vel = velocity_min;
                    // if vel <= velocity_min:
                    //     vel = velocity_min
                    if (direct == 1){ // tien
                        twist.linear.x = vel;
                    }
                    else if (direct == 2){ // lui
                        twist.linear.x = vel*(-1);
                    }
                    ROS_INFO("S= %f , S_moved= %f, direct= %d", S, S_moved, direct);
                    pub_cmdVel(twist, rate_pubVel);
                    return 0;
                }

                else{
                    pub_Stop();
                    step_moveForward = 0;
                    return 1;
                }
            } 
            else{
                if (req_parking.modeRun == 4){
                    // ROS_INFO("Recieve data Stop!");
                    geometry_msgs::Twist twist;
                    pub_cmdVel(twist, rate_pubVel);
                }
                else if (req_parking.modeRun == 0){
                    // ROS_INFO("Recieve data Reset!")
                    resetAll();
                    pub_Stop();
                }
                return 0;
            }
        }

        else{
            return 0;
        }
        return 0;
    }

    int8_t rotary_around(double Angle, int8_t direct, float velocity_min, float velocity_max, double Angle_StartDecel, float permission_tolerance){
        geometry_msgs::Twist twist;
        double Angle_turned;
        float vel;
        if (step_Rotary == 0){
            angle_odom_start = odom_g;
            step_Rotary = 1;
        }
        else if (step_Rotary == 1){
            if (req_parking.modeRun == 1 || req_parking.modeRun == 2 || req_parking.modeRun == 3){
                Angle_turned = odom_g - angle_odom_start;
                if (fabs(Angle_turned) > M_PI){
                    Angle_turned = 2*M_PI - fabs(Angle_turned);
                }
                else{
                    Angle_turned = fabs(Angle_turned);
                }
                if (fabs(Angle_turned - Angle) >= permission_tolerance && Angle_turned - Angle <= permission_tolerance){
                    twist.linear.x = 0;
                    vel = 0.0;
                    if (Angle >= M_PI/9){ // M_PI/6
                        if (fabs(Angle_turned - Angle) <= Angle_StartDecel){
                            vel = velocity_max*(fabs(Angle_turned - Angle)/Angle);
                            if (vel <= velocity_min)
                                vel = velocity_min;
                        }
                        else{
                            vel = velocity_max;
                        }
                    }

                    else{
                        vel = velocity_min;
                    }
                            
                    if (direct == 1){ // quay trai
                        twist.angular.z = vel;
                    }
                    else if (direct == 2){ // quay phai
                        twist.angular.z = vel*(-1);
                    }

                    ROS_INFO("Angle= %f , Angle_turned= %f, direct= %d", Angle, Angle_turned, direct);
                    pub_cmdVel(twist, rate_pubVel);
                    return 0;
                } 

                else{
                    pub_Stop();
                    step_Rotary = 0;
                    return 1;
                }
            }
            else{
                if (req_parking.modeRun == 4){
                    // ROS_INFO("Recieve data Stop!");
                    geometry_msgs::Twist twist;
                    pub_cmdVel(twist, rate_pubVel);
                }
                else if (req_parking.modeRun == 0){
                    // ROS_INFO("Recieve data Reset!");
                    resetAll();
                    pub_Stop();
                }
                return 0;
            }
        }

        else{
            return 0;
        }

        return 0;
    }


    int8_t follow_target(float velocity_min, float velocity_max, float vel_rotMax, float distance_decel, float distance_ahead, float permission_tolerance){
        float vel_x = 0.0;
        float vel_rot = 0.0;
        geometry_msgs::Twist twist;
        uint8_t selectAngle = 0;
        uint8_t directMode1 = 0; // dung voi truong hop selectAngle = 1 | 1 quay trai, 2 quay phai
        geometry_msgs::Pose poseThenTransform;
        double roll, pitch, yaw;
        double angle;
        double angleAGV;
        float distance;
        double angle_follow;
        float kg;
        float kpg;
        float kpd;

        poseThenTransform = transformPoseNAV("frame_target", "frame_robot");
        posetf = poseThenTransform;

        if (poseThenTransform.position.x == 0.0 && poseThenTransform.position.y == 0.0 && poseThenTransform.orientation.z == 0.0 && poseThenTransform.orientation.w == 0.0){
            ROS_INFO("Value poseThenTransform is 0! Stop process, Wait for restart!");
            warn_agv = 2;
            return 2;                     // lỗi ko thể get tọa độ chuyển đổi
        }
        else{

            angleAGV = yawtf;
            distance = sqrt(poseThenTransform.position.x * poseThenTransform.position.x + poseThenTransform.position.y*poseThenTransform.position.y);
            angle = 0.0;
            if (req_parking.modeRun == 1 ||req_parking.modeRun == 2 || req_parking.modeRun == 3){
                if (distance >= permission_tolerance && poseThenTransform.position.x < permission_tolerance){ // dieu kien den target
                    if ((zone_lidar.zone_sick_behind == 1 && req_parking.modeRun == 3) || (zone_lidar.zone_sick_behind != 0 && req_parking.modeRun == 1)){
                        warn_agv = 1;
                        ROS_INFO("co vat can");
                        pub_Stop();
                    }
                    else{
                        warn_agv = 0;
                        // tính vận tốc dài 
                        if (distance <= distance_decel){   //tinh van toc dai theo khoang cach, lúc đầu đi vào thì AGV đi vào với vận tốc tối đa, đến vị trí giảm tốc thì vận tốc AGV tỉ lệ theo khoảng cách lùi vào 
                            vel_x = velocity_max*(distance/distance_decel);
                            if (vel_x >= velocity_max)
                                vel_x = velocity_max;
                            if (vel_x <= velocity_min)
                                vel_x = velocity_min;
                        }
                        else{
                            vel_x = velocity_max;
                        }

                        twist.linear.x = vel_x*(-1);     // âm tức là đi lùi 

                        //// -- Khi AGV gần tới điểm đích thì góc mong muốn của AGV với điểm đích, mong muốn là PI, nên ta sẽ dùng PID để tìm ra tỉ lệ vận tốc góc và sai số góc ( ở đây chỉ sử dụng hệ số P)
                        //// -- thuật toán ở đây là AGV lùi thẳng vào 
                        if (fabs(poseThenTransform.position.x) <= distance_decel){  // tinh van toc goc
                            angle = M_PI - fabs(angleAGV);
                            selectAngle = 1;

                            kg = 0.25;   // 0.7
                            vel_rot = kg*angle;
                            if (vel_rot >= vel_rotMax)
                                vel_rot = vel_rotMax;
                        }

                        //// -- Khi AGV bắt đầu di chuyển, mong muốn vị trí AGV sẽ bám theo đường thẳng kẻ từ frame target nên ta cần tính vận tốc góc của AGV theo cả góc lệch và sai số 
                        else{
                            angle_follow = atan(fabs(poseThenTransform.position.y)/distance_ahead);
                            if (poseThenTransform.position.y > 0 && angleAGV > 0){
                                if (angleAGV > angle_follow){
                                    angle = M_PI - angle_follow - fabs(angleAGV);
                                    directMode1 = 1;
                                }
                                else{
                                    angle = angle_follow + fabs(angleAGV) - M_PI;
                                    directMode1 = 2;
                                }
                            }
                            else if (poseThenTransform.position.y > 0 && angleAGV < 0){
                                angle = M_PI + angle_follow - fabs(angleAGV);
                                directMode1 = 2;
                            }
                            else if (poseThenTransform.position.y < 0 && angleAGV < 0){
                                if (fabs(angleAGV) > angle_follow){
                                    angle = M_PI - angle_follow - fabs(angleAGV);
                                    directMode1 = 2;
                                }
                                else{
                                    angle = angle_follow + fabs(angleAGV) - M_PI;
                                    directMode1 = 1;
                                }
                            }
                            else if (poseThenTransform.position.y < 0 && angleAGV > 0){
                                angle = M_PI + angle_follow - fabs(angleAGV);
                                directMode1 = 1;
                            }
                            selectAngle = 2;

                            kpg = 0.55;  // 2. #1.5 #1.55
                            kpd = 0.45;
                            float vel_rot = kpg*angle + kpd*fabs(poseThenTransform.position.y);
                            if (vel_rot >= vel_rotMax){
                                vel_rot = vel_rotMax;
                            }
                        }

                        if (selectAngle == 1){  // hướng của AGV lùi 
                            if (angleAGV > 0){
                                twist.angular.z = vel_rot;
                            }
                            else{
                                twist.angular.z = vel_rot*(-1);
                            }
                        }

                        else{
                            if (directMode1 == 1){
                                twist.angular.z = vel_rot;
                            }
                            else if (directMode1 == 2){
                                twist.angular.z = vel_rot*(-1);
                            }
                        }

                        double lech = M_PI - fabs(angleAGV);
                        cout << "MODE= " << selectAngle << " Distance = " << poseThenTransform.position.y << " X = " << poseThenTransform.position.x << " goc lech= " << lech << " goc_follow= " << angle << endl;
                        // ROS_INFO("MODE = %s ,Distan= %s, X= %s , goc_lech= %s, goc_follow= %s" %(selectAngle, poseThenTransform.position.y, poseThenTransform.position.x, lech, angle))
                        pub_cmdVel(twist, rate_pubVel);
                    }
                    return 0;
                }

                else{     // đã đến vùng target nên dừng lại 
                    pub_Stop();
                    return 1;
                }
            }

            else{
                if (req_parking.modeRun == 3){    // 4
                    // ROS_INFO("Recieve data Stop!")
                    geometry_msgs::Twist twist;
                    pub_cmdVel(twist, rate_pubVel);
                }
                else if (req_parking.modeRun == 0){
                    // ROS_INFO("Recieve data Reset!")
                    resetAll();
                    pub_Stop();
                }
                return 0;
            }
        }
    }

    void resetAll(){
        process = 1;
        pubTransform = false;
        is_request_parking = false;
        time_waitTransfrom = time(NULL);
        step_moveForward = 0;
        step_Rotary = 0;
        warn_agv = 0;
    }

    void Program1(){
        while(shutdown_flag == 0){
            // cout << " Chay chuong trinh 1";
            if (pubTransform == true && (req_parking.modeRun == 1 || req_parking.modeRun == 2 || req_parking.modeRun == 3)){
                // cout << " Đang biến đổi" << endl;
                sendTransform("frame_map_nav350", "frame_target", req_parking.poseTarget);
                is_transform = true;
            }
            else{
                // cout<<"Đang ko biến đổi" << endl;
                is_transform = false;
            }
            usleep(10000); 
        }
    }
};


// void fnShutDown(int sig){
//     ROS_INFO("Shutting down. cmd_vel will be 0");
//     ParkingNAV self;
//     geometry_msgs::Twist twist;
//     self.pub_cmd_vel.publish(twist);

//     ros::shutdown(); 
// }

void signal_handler(int signal_num){
    ParkingNAV self;
    self.shutdown_flag = 1;
    cout << "Program stop due to Ctrl C";
    geometry_msgs::Twist twist;
    self.pub_cmd_vel.publish(twist);

    ros::shutdown();
    exit(signal_num);
}

int main(int argc, char **argv)
{
    std::cout << "Program start!";

    ros::init(argc, argv, "Parking_navNM_cpp");
    ros::NodeHandle n;
    ros::Rate loop_rate(40);

    ParkingNAV self;
    // ros::param::get("~rate", self.rate_str);
    // self.rate = stoi(self.rate_str);
    // self.rate = 40;
    // ros::Rate loop_rate(self.rate);
    // self.loop_rate(self.rate);
    
    // subcribe topic
    self.sub_ParkingRequest = n.subscribe("/parking_request", 1000, &ParkingNAV::request_callback, &self);
    self.is_request_parking = false;

    self.sub_robotPose = n.subscribe("/robotPose_nav", 1000, &ParkingNAV::getPose, &self);
    self.is_pose_robot = false;
    self.theta_robotNow = 0.0;

    self.sub_GetRobotOdom = n.subscribe("/odometry", 1000, &ParkingNAV::cbGetRobotOdom, &self);
    self.is_odom_rb = false;

    self.sub_HCinfo = n.subscribe("/HC_info", 1000, &ParkingNAV::zone_callback, &self);   
    self.is_check_zone = false;

    //pusblish topic
    self.pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    self.time_tr = ros::Time::now().toSec();
    self.rate_pubVel = 15;

    self.pub_ParkingRespond = n.advertise<message_pkg::Parking_respond>("/parking_respond", 1000);
    self.timePubRespond = ros::Time::now().toSec();
    self.rate_pubRespond = 30;

    self.auto_reset = 0;  

    self.odom_x_ht = 0.;
    self.odom_y_ht = 0.;
    self.odom_g = 0.;

    self.step_moveForward = 0;
    self.x_odom_start = 0.0;
    self.y_odom_start = 0.0;

    self.step_Rotary = 0;
    self.angle_odom_start = 0.0;

    self.min_vel = 0.03;
    self.min_velFinish = 0.04;
    self.min_rol = 0.1;
    self.min_rolFinish = 0.15;

    self.max_vel = 0.07;
    self.max_rol = 0.3;

    self.pubTransform = false;
    self.process = 0;

    self.ss_x = 0.0;
    self.ss_y = 0.0;
    self.ss_a = 0.0;
    
    self.warn_agv = 0;

    self.time_waitTransfrom = time(NULL);
    // self.time_startTransform = ros::Time::now().toSec();
    self.is_transform = false;

    self.shutdown_flag = 0;

    self.yawtf = 0.;

    //off all progress
    signal(SIGABRT, signal_handler);
    double SGo_forward = 0.0;
    double AGO_rotary = 0.0;
    int8_t direct_forward = 0; // 1 tien, 2 lui
    int8_t direct_rotary = 0; // 1 quay trai, 2 quay phai
    geometry_msgs::Pose poseThenTransform;
    try{
        thread th1(&ParkingNAV::Program1, &self);

        while (ros::ok())
        {

            if (self.process == 0){
                // ROS_INFO("wait receive all need data......");
                // if (self.is_pose_robot == true){
                //     c_k = c_k + 1;
                // }
                // if (self.is_odom_rb == true) {
                //     c_k = c_k + 1;
                // }
                // if (self.is_check_zone == true){
                //     c_k = c_k + 1;
                // }
                // if (c_k == 3){
                //     self.process = 1;
                //     // ROS_INFO("Done receive all need data!");
                // }
                if  (self.is_pose_robot == true && self.is_odom_rb == true && self.is_check_zone == true){
                    self.process = 1;
                }
            }
            else if (self.process == 1){ // cho tin hieu Parking
                if (self.is_request_parking == true){
                    if (self.req_parking.modeRun == 1 || self.req_parking.modeRun == 2 || self.req_parking.modeRun == 3){ //un
                        ROS_INFO("Received data request Parking, Start Process!");
                        self.time_waitTransfrom = time(NULL);
                        self.pubTransform = true;
                        self.process = 41;
                    }
                }
            }
            else if (self.process == 21){   // tinh quang duong di chuyen
                
                if (float(time(NULL) - self.time_waitTransfrom) <= 1.5){
                    poseThenTransform = self.transformPoseNAV("frame_target", "frame_robot");
                }
                else{
                    if (poseThenTransform.position.x == 0.0 && poseThenTransform.position.y == 0.0 && poseThenTransform.orientation.z == 0.0 && poseThenTransform.orientation.w == 0.0){
                        // ROS_INFO("Value poseThenTransform is 0! Stop process, Wait for restart!");
                        self.process = 52;
                        self.pub_Stop();
                    }

                    else{
                        if (fabs(poseThenTransform.position.y) <= 0.02){
                            self.time_waitTransfrom = time(NULL);
                            self.process = 31; // chuyen sang buoc tinh goc quay de lui vao ke
                        }
                        else{
                            double angle = self.yawtf;

                            if (fabs(angle) < M_PI/4 || fabs(angle) > (3*M_PI)/4){   // truong hop AGV lech goc lon
                                AGO_rotary = fabs((M_PI/2) - fabs(angle));
                                if ((angle > 0 && angle < M_PI/2) || (angle < 0 && fabs(angle) > M_PI/2)){
                                    direct_rotary = 1;
                                }
                                else{
                                    direct_rotary = 2;
                                }

                                self.step_moveForward = 0;
                                self.process = -210;
                            }

                            else{
                                if (fabs(angle) > M_PI/2){
                                    SGo_forward = fabs(poseThenTransform.position.y)/sin(M_PI - fabs(angle));
                                }
                                else{
                                    SGo_forward = fabs(poseThenTransform.position.y)/sin(fabs(angle));
                                }

                                if ((poseThenTransform.position.y > 0 && angle > 0) || (poseThenTransform.position.y < 0 && angle < 0)){
                                    direct_forward = 2;
                                }
                                else{
                                    direct_forward = 1;
                                }

                                self.step_moveForward = 0;
                                self.process = -21;
                            }
                        }
                    }
                }
            }

            else if (self.process == -21){ // di chuyen vao duong thang target
                if (self.move_forward(SGo_forward, direct_forward, self.min_vel, 0) == 1){
                    self.time_waitTransfrom = time(NULL);
                    self.process = 21; // kiem tra lai xem da chinh xac chua     
                }
            }

            else if (self.process == -210){ // quay goc truoc khi tinh chinh khoang cach neu goc di chuyen lon
                if (self.rotary_around(AGO_rotary, direct_rotary, self.min_rol, self.max_rol, M_PI/5.0, 0.015) == 1){
                    self.time_waitTransfrom = time(NULL);
                    self.process = 21; // kiem tra lai xem da chinh xac chua  
                }
            }

            else if (self.process == 31){  //tinh goc can quay
                if (time(NULL) - self.time_waitTransfrom <= 1.5){
                    poseThenTransform = self.transformPoseNAV("frame_target", "frame_robot");
                }

                else{
                    // ROS_INFO(poseThenTransform);                   //****
                    if (poseThenTransform.position.x == 0.0 && poseThenTransform.position.y == 0.0 && poseThenTransform.orientation.z == 0.0 && poseThenTransform.orientation.w == 0.0 ){
                        ROS_INFO("Value poseThenTransform is 0! Stop process, Wait for restart!");
                        self.process = 52;
                        self.pub_Stop();
                    }
                    else{
                        double angle = self.yawtf;

                        ROS_INFO("%f", M_PI - fabs(angle));
                        if ((M_PI - fabs(angle)) <= 0.04){
                            self.process = 41; // chuyen sang buoc parking
                        }

                        else{
                            AGO_rotary = M_PI - fabs(angle);
                            if (angle > 0){
                                direct_rotary = 1;
                            }
                            else{
                                direct_rotary = 2;
                            }

                            self.step_Rotary = 0;
                            self.process = -31;
                        }
                    }
                }
            }

            else if (self.process == -31){ // quay vao duong thang target
                if (self.rotary_around(AGO_rotary, direct_rotary, self.min_rol, self.max_rol, M_PI/6.0, 0.015) == 1){
                    self.time_waitTransfrom = time(NULL);
                    self.process = 31; // kiem tra lai xem da chinh xac chua
                }
            }

            else if (self.process == 41){ // parking vao ke
                // ROS_INFO("Start Parking!")
                // if follow_target(min_vel, max_vel, min_vel, 0.3, 0) == 1){
                // if follow_target(min_vel, max_vel, min_rol, 0.6, 0.4, 0) == 1){  // 0.5 | 0.35
                int8_t stt = self.follow_target(self.min_velFinish, self.max_vel, self.min_rolFinish, 0.6, 0.4, 0);
                if (stt == 1){  // 0.5 | 0.35
                    // ROS_INFO("Done Parking!")
                    usleep(500000);
                    self.process = 51;
                }
                else if (stt == 2){
                    // ROS_INFO("Value poseThenTransform is 0! Stop self.process, Wait for restart!")
                    self.process = 52;
                    self.pub_Stop();
                }
            }

            else if (self.process == 51){ // wait for reset transform
                // ROS_INFO("Done!")
                if (self.req_parking.modeRun == 0){ //Reset
                    // self.req_parking = Parking_request()
                    // ROS_INFO("Recieve data Reset!")
                    self.resetAll();
                }
            }

            else if (self.process == 52){ // loi pose transform error
                if (self.req_parking.modeRun == 0){ //Reset
                    // self.req_parking = Parking_request()
                    // ROS_INFO("Recieve data Reset!")
                    self.resetAll();
                }
            }
                    
            string mess_pub = self.Meaning(self.process, self.warn_agv);

            self.pub_Status(self.process, self.req_parking.modeRun, self.posetf, self.req_parking.poseTarget, self.req_parking.offset, self.ss_x, self.ss_y, self.ss_a, mess_pub, self.warn_agv);
            ros::spinOnce();     // allow receiving callbacks function
            loop_rate.sleep();
        }
        th1.join();
    }
    catch(...){
        self.shutdown_flag = 1;
    }

    return 0;
}
