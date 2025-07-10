// Author : Phùng Quý Dương 
// Date: 21-09-2023

/*
    - Node filter_topic
    - function:
        + Pub dữ liệu lấy từ động cơ tại 1 thời điểm khác 
*/

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/Int16.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include "message_pkg/Driver_respond.h"
#include "message_pkg/Driver_query.h"

#include <bits/stdc++.h>
#include <thread>
#include <signal.h>
#include <csignal>
using namespace std;

struct RPM
{
    /* data */
    int motor1 = 0;
    int motor2 = 0;
};

class kinematic{
    public:
    float wheel_circumference;
    float transmission_ratio;
    float distanceBetwentWheels;
    float frequency_control;
    double linear_max;
    double angular_max;
    int max_rpm;

    bool shutdown_flag = 1;

    string topicControl_vel;
    string topicGet_vel;

    string topicControl_driverLeft;
    string topicControl_driverRight;
    string topicRespond_driverLeft;
    string topicRespond_driverRight;

    string frame_id;

    double lastTime_pub;
    double time_pub;

    // -- 
    double lastTime_speed1;
    double nowTime_speed1;
    
    double lastTime_speed2;
    double nowTime_speed2;
    // -- find main driver.
    bool is_finded = 0;
    uint8_t mainDriver = 0;
    // -- frequence pub raw vel.
    float fre_rawVel = 25.;
    float cycle_rawVel = 1/fre_rawVel;
    float timeout = cycle_rawVel*4;
    bool isNew_driver1 = 0;
    bool isNew_driver2 = 0;
    // -- 
    bool is_exit = 1;
    //-- 
    double nowTime_cmdVel;
    double timeout_cmdVel = 0.4; 
    bool is_timeout = 0;
    // -- 
    double max_deltaTime = 0.0;
    // -- 
    double max_rotation = 0.6;

    ros::Subscriber sub_ControlVel; 
    geometry_msgs::Twist cmd_vel;

    ros::Subscriber sub_taskDriver;
    std_msgs::Int16 task_driver;

    ros::Subscriber sub_driver1Respond; 
    message_pkg::Driver_respond driver1_respond;

    ros::Subscriber sub_driver2Respond; 
    message_pkg::Driver_respond driver2_respond;

    // -----------------
    ros::Publisher pub_rawVel;
    geometry_msgs::TwistWithCovarianceStamped raw_vel;

    ros::Publisher pub_driverLeft;
    message_pkg::Driver_query driver1_query;

    ros::Publisher pub_driverRight;
    message_pkg::Driver_query driver2_query;
    RPM driverRPM_query;

    // -----------------
    Program(ros::NodeHandle *nh, ros::NodeHandle *npr){
        ros::param::get("wheel_circumference", self.wheel_circumference);
        ros::param::get("transmission_ratio", self.transmission_ratio);
        ros::param::get("distanceBetwentWheels", self.distanceBetwentWheels);
        ros::param::get("frequency_control", self.frequency_control);

        ros::param::get("linear_max", self.linear_max);
        ros::param::get("angular_max", self.angular_max);
        ros::param::get("max_rpm", self.max_rpm);

        ros::param::get("topicControl_vel", self.topicControl_vel);
        ros::param::get("topicGet_vel", self.topicGet_vel);

        ros::param::get("topicControl_driverLeft", self.topicControl_driverLeft);
        ros::param::get("topicControl_driverRight", self.topicControl_driverRight);
        ros::param::get("topicRespond_driverLeft", self.topicRespond_driverLeft);
        ros::param::get("topicRespond_driverRight", self.topicRespond_driverRight);

        ros::param::get("frame_id", self.frame_id);

        sub_ControlVel = nh->subscribe(self.topicControl_vel, 50, &kinematic::cmdVel_callback, this);
        sub_taskDriver = nh->subscribe("/task_driver", 50, &kinematic::taskDriver_callback, this);
        sub_driver1Respond = nh->subscribe(self.topicRespond_driverLeft, 50, &kinematic::driver1Respond_callback, this);
        sub_driver2Respond = nh->subscribe(self.topicRespond_driverRight, 50, &kinematic::driver2Respond_callback, this);

        // -----------------
        pub_rawVel = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>(self.topicGet_vel, 50);
        pub_driverLeft = nh->advertise<message_pkg::Driver_query>(self.topicControl_driverLeft, 50);
        pub_driverRight = nh->advertise<message_pkg::Driver_query>(self.topicControl_driverRight, 50);
        lastTime_pub = ros::Time::now().toSec();
        time_pub = 1/frequency_control;
        lastTime_speed1 = ros::Time::now().toSec();
        nowTime_speed1 = ros::Time::now().toSec();
        
        lastTime_speed2 = ros::Time::now().toSec();
        nowTime_speed2 = ros::Time::now().toSec();
        nowTime_cmdVel = ros::Time::now().toSec();
    }

	void taskDriver_callback(const std_msgs::Int16 data){
		task_driver = data;
    }

	void cmdVel_callback(const geometry_msgs::Twist data){
		cmd_vel = data;
		nowTime_cmdVel = ros::Time::now().toSec();
    }

	void driver1Respond_callback(const message_pkg::Driver_respond data){
		driver1_respond = data;
		nowTime_speed1 = ros::Time::now().toSec();
		isNew_driver1 = 1;
    }

	void driver2Respond_callback(const message_pkg::Driver_respond data){
		driver2_respond = data;
		nowTime_speed2 = ros::Time::now().toSec();
		isNew_driver2 = 1;
    }

	double constrain(double val_in, double val_compare1, double val_compare2){
		double val = 0.0;
		val = max(val_in, val_compare1);
		val = min(val_in, val_compare2);
		return val;
    }

	RPM calculateRPM(double linear_x, double angular_z){
		double linear_x_right = linear_x;
		double angular_x_right = angular_z;

		double linear_vel_x_mins = 0.0;
		double angular_vel_z_mins = 0.0;
		double tangential_vel = 0.0;
		double x_rpm = 0.0;
		double tan_rpm = 0.0;
		// -- limit
		if (abs(linear_x_right) > linear_max){
			if (linear_x_right >= 0){
				linear_x_right = linear_max;
            }
			else{
				linear_x_right = -linear_max;
            }
        }

		if (abs(angular_x_right) > angular_max){
			if (angular_x_right >= 0){
				angular_x_right = angular_max;
            }
			else{
				angular_x_right = -angular_max;
            }
        }

		// convert m/s to m/min
		linear_vel_x_mins = linear_x_right * 60;
		// convert rad/s to rad/min
		angular_vel_z_mins = angular_x_right * 60;
		tangential_vel = angular_vel_z_mins * (distanceBetwentWheels / 2);

		x_rpm = linear_vel_x_mins / wheel_circumference;
		tan_rpm = tangential_vel / wheel_circumference;

		RPM rpm_query;

		// calculate for the target motor RPM and direction
		// front-left motor
		rpm_query.motor1 = (x_rpm - tan_rpm)*transmission_ratio;
		rpm_query.motor1 = constrain(rpm_query.motor1, -max_rpm, max_rpm);
		// front-right motor
		rpm_query.motor2 = (x_rpm + tan_rpm)*transmission_ratio;
		rpm_query.motor2 = constrain(rpm_query.motor2, -max_rpm, max_rpm);
		return rpm_query;
    }

	geometry_msgs::TwistWithCovarianceStamped calculate_rawVel(double rp1, double rp2){ // rp1,rp2: RPM | out: Velocities(m/s;rad/s)
		geometry_msgs::TwistWithCovarianceStamped vel;
		double average_rps = ((rp2 + rp1)/2.)/60./transmission_ratio;
		double angular_rps = ((rp2 - rp1)/2.)/60./transmission_ratio;

		vel.twist.twist.linear.x = average_rps*wheel_circumference;
		vel.twist.twist.angular.z = (angular_rps*wheel_circumference)/(distanceBetwentWheels/2.);

		vel.twist.covariance[0] = 0.001;
		vel.twist.covariance[7] = 0.001;
		vel.twist.covariance[35] = 0.001;

		vel.header.stamp = ros::Time::now();
		vel.header.frame_id =  frame_id;

		return vel;
    }

	void getVel_v1(){
		if (isNew_driver1 == 1){
			int16_t vel_1 = driver1_respond.speed;
			int16_t vel_2 = driver2_respond.speed;
			raw_vel = calculate_rawVel(vel_1, vel_2);
			pub_rawVel.publish(raw_vel);
			isNew_driver1 = 0;
        }
    }

	int8_t getVel(){
		double t1 = ros::Time::now().toSec() - nowTime_speed1;
		double t2 = ros::Time::now().toSec() - nowTime_speed2;
		int16_t vel_1 = 0;
		int16_t vel_2 = 0;

		if (t1 < timeout && t2 < timeout){ // -- check time out
			if (is_finded){ // 
				// dong bo du lieu
				if (mainDriver == 1){ // -- driver left
					if (isNew_driver1 == 1){ // neu co du lieu moi.
						vel_1 = driver1_respond.speed;
						vel_2 = driver2_respond.speed;
						isNew_driver1 = 0;
                    }
                }

				else{ // -- driver right
					if (isNew_driver2 == 1){ // neu co du lieu moi.
						vel_1 = driver1_respond.speed;
						vel_2 = driver2_respond.speed;
						isNew_driver2 = 0;
                    }
                }

				raw_vel = calculate_rawVel(vel_1, vel_2);
				pub_rawVel.publish(raw_vel);
				return 1;
            }

			else{
				double delta_dif = abs(nowTime_speed2 - nowTime_speed1);
				if (nowTime_speed2 >= nowTime_speed1){
					if (delta_dif >= cycle_rawVel/2.){
						is_finded = 1;
						mainDriver = 1;
                    }
					else{
						is_finded = 1;
						mainDriver = 2;
                    }
                }
				else{
					if (delta_dif >= cycle_rawVel/2.){
						is_finded = 1;
						mainDriver = 2;
                    }
					else{
						is_finded = 1;
						mainDriver = 1;
                    }
                }
				return 0;
            }
        }
		else{
			is_finded = 0;
			return -1;
        }
    }

	void run_getVel(){
        while(shutdown_flag == 0){
		    getVel_v1();
            usleep(1000);
        }
    }

    void run(){
        if (task_driver.data == 0){ // -- Nothing
            driver1_query.task = 0;
            driver2_query.task = 0;
        }

        else if (task_driver.data == 1){ // -- Reset + Read status
            driver1_query.task = 1;
            driver2_query.task = 1;
        }
        else if (task_driver.data == 2){ // -- Read status
            driver1_query.task = 2;
            driver2_query.task = 2;
        }

        // -- PUB
        double t_pub_1 = ros::Time::now().toSec() - lastTime_pub;
        double t_pub = t_pub_1 - t_pub_1/60.0;

        if (t_pub > time_pub){
            lastTime_pub = ros::Time::now().toSec();
            driverRPM_query = calculateRPM(cmd_vel.linear.x, cmd_vel.angular.z);

            driver1_query.modeStop = 1;
            driver1_query.rotationSpeed = int(driverRPM_query.motor1);

            driver2_query.modeStop = 1;
            driver2_query.rotationSpeed = int(driverRPM_query.motor2);
            
            if (is_timeout == 1){
                message_pkg::Driver_query driver_query;
                pub_driverLeft.publish(driver_query);
                pub_driverRight.publish(driver_query);
            }
            else{ // -- ok
                pub_driverLeft.publish(driver1_query);
                pub_driverRight.publish(driver2_query);
            }
        }
        ros::spinOnce();     // allow receiving callbacks function
        loop_rate.sleep();        
    }
};

void signal_handler(int signal_num){
    kinematic self;
    self.shutdown_flag = 1;
    cout << "Program stop due to Ctrl C" << endl;
    ros::shutdown();
    exit(signal_num);
}

int main(int argc, char **argv)
{
    std::cout << "Program start!";

    ros::init(argc, argv, "kinematicNode_cpp");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    kinematic self;

    // parameters
    signal(SIGABRT, signal_handler);

    try{
        thread th1(&kinematic::run_getVel, &self);

        while(ros::ok()){
            self.run();
        }

        th1.join();
    }
    catch(...){
        self.shutdown_flag = 1;
    }

    return 0;

}
