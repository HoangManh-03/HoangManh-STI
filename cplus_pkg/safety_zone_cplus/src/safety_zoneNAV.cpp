// Author : Phùng Quý Dương 
// Date: 15-9-2023

/*
    Node: safety_Zone -- C++
    Mission: lấy dữ liệu an toàn của cảm biến NAV

*/

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/Int8.h"
#include "sensor_msgs/LaserScan.h"

#include <bits/stdc++.h>

using namespace std;

class SafetyZone{
    public:
    double dis_max;
    double dis_min;

    // topic pub-sub
    ros::Subscriber sub_scan;
    ros::Publisher  zone_lidarNAV;
    std_msgs::Int8 dPubZone;

    // variable
    sensor_msgs::LaserScan data_scan;
    bool is_scanNav;

    void call_sub(const sensor_msgs::LaserScan data){
        data_scan = data;
        is_scanNav = true;
    }
};

int main(int argc, char **argv){
    cout << "Program start!";

    ros::init(argc, argv, "AcquireScan");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);

    SafetyZone self;
    ros::param::get("~dis_max", self.dis_max);
    ros::param::get("~dis_min", self.dis_min);

    // topic pub-sub
    self.sub_scan = n.subscribe("/scan", 100, &SafetyZone::call_sub, &self);
    self.zone_lidarNAV = n.advertise<std_msgs::Int8>("/safety_NAV", 10);

    // variable
    self.is_scanNav = false;
    int numberPointinCircle = 0;
    while(ros::ok()){
        if (self.is_scanNav == true){
            self.is_scanNav = false;
            numberPointinCircle = 0;
            try{
                for(int i = 0; i < self.data_scan.ranges.size(); i++){ 
                    if (self.data_scan.ranges[i] >= self.dis_min && self.data_scan.ranges[i] <= self.dis_max){
                        numberPointinCircle = numberPointinCircle + 1;
                    }
                }
            }
            catch(...){
                numberPointinCircle = 500;
                cout<<"Something wrong!!!";
            }

            // print(numberPointinCircle)
            if (numberPointinCircle >= 15){
                self.dPubZone.data = 1;
            }

            else{
                self.dPubZone.data = 0;
            }

            self.zone_lidarNAV.publish(self.dPubZone);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}