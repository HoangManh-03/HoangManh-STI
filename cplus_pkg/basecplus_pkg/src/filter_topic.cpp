// Author : Phùng Quý Dương 
// Date: 21-09-2023

/*
    - Node filter_topic
    - function:
        + 
*/

#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Twist.h"

#include "message_pkg/Driver_respond.h"

#include <bits/stdc++.h>
using namespace std;

class filterTopic{
    public:
    float pub_frequence;
    string topicSub_driver1;
    string topicSub_driver2;
    string topicPub_driver1;
    string topicPub_driver2;

    // -- SUB
    ros::Subscriber sub_topic_driver1;
    message_pkg::Driver_respond driver1_respond;

    ros::Subscriber sub_topic_driver2;
    message_pkg::Driver_respond driver2_respond;

    // -- PUB
    ros::Publisher pub_filter1;
    message_pkg::Driver_respond driver1_filter;

    ros::Publisher pub_filter2;
    message_pkg::Driver_respond driver2_filter;

    // --
    float time_pub; // s
    double preTime_pub1;
    double preTime_pub2;
    // -- 
    double nowTime_speed1;
    double nowTime_speed2;
    bool is_timeout1;
    bool is_timeout2;
    float timeout;

	void driver1Respond_callback(const message_pkg::Driver_respond data){
		driver1_respond = data;
        // cout << "Tao co nhan duo du lieu tu driver 1" << endl;
		nowTime_speed1 = ros::Time::now().toSec();
    }

	void driver2Respond_callback(const message_pkg::Driver_respond data){
        // cout << "Tao co nhan duo du lieu tu driver 2" << endl;
		driver2_respond = data;
		nowTime_speed2 = ros::Time::now().toSec();
    }
};

int main(int argc, char **argv)
{
    cout << "Program start!";

    ros::init(argc, argv, "filterTopic_cpp");
    ros::NodeHandle n;

    filterTopic self;

    // -- parameter
    ros::param::get("frequence", self.pub_frequence);
    ros::Rate loop_rate(self.pub_frequence);

    ros::param::get("topicSub_driver1", self.topicSub_driver1);
    ros::param::get("topicSub_driver2", self.topicSub_driver2);

    ros::param::get("topicPub_driver1", self.topicPub_driver1);
    ros::param::get("topicPub_driver2", self.topicPub_driver2);

    // // -- SUB
    self.sub_topic_driver1 = n.subscribe(self.topicSub_driver1, 20, &filterTopic::driver1Respond_callback, &self);
    self.sub_topic_driver2 = n.subscribe(self.topicSub_driver2, 20, &filterTopic::driver2Respond_callback, &self);

    // -- PUB
    self.pub_filter1 = n.advertise<message_pkg::Driver_respond>(self.topicPub_driver1, 50);
    self.pub_filter2 = n.advertise<message_pkg::Driver_respond>(self.topicPub_driver2, 50);      

    // --
    self.time_pub = 1/self.pub_frequence; // s
    self.preTime_pub1 = ros::Time::now().toSec();
    self.preTime_pub2 = ros::Time::now().toSec();
    // -- 
    self.nowTime_speed1 = ros::Time::now().toSec();
    self.nowTime_speed2 = ros::Time::now().toSec();
    self.is_timeout1 = 0;
    self.is_timeout2 = 0;
    self.timeout = self.time_pub*3.;

    while(ros::ok()){

        double t1_out = ros::Time::now().toSec() - self.nowTime_speed1;
        if (t1_out > self.timeout){
            self.is_timeout1 = 1;
        }
        else{
            self.is_timeout1 = 0;
        }
        // --
        double t2_out = ros::Time::now().toSec() - self.nowTime_speed2;
        if (t2_out > self.timeout){
            self.is_timeout2 = 1;
        }
        else{
            self.is_timeout2 = 0;
        }
        // --
        if (self.is_timeout1 == 0){
            // cout<< "Im here!--------------------- pub ra driver1 ------------------" << endl;
            self.pub_filter1.publish(self.driver1_respond);
        }
        // --
        if (self.is_timeout2 == 0){
            // cout<< "Im here!--------------------- pub ra driver2 ------------------" << endl;
            self.pub_filter2.publish(self.driver2_respond);
        }        
        ros::spinOnce();     // allow receiving callbacks function
        loop_rate.sleep();
    }

    return 0;
}