// Author : Phùng Quý Dương 
// Date: 15-9-2023

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Int16.h"
#include <bits/stdc++.h>

using namespace std;

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "first_work");   // specify node names
    ros::NodeHandle n;      //create a handle to this process's node
    ros::Publisher pub_run = n.advertise<std_msgs::Int16>("/first_work/run", 1000); 
    ros::Rate loop_rate(10);   // 10Hz

    std_msgs::Int16 run;
    uint8_t count = 0;
    string command = "yes | rosclean purge";
    int result = system(command.c_str());
    if (result == -1){
        ROS_ERROR("Failed to execute command");
    }

    while (ros::ok())
    {
        pub_run.publish(run);
        count++;
        if(count > 100){
            break;
        }     
        
        ros::spinOnce();     // allow receiving callbacks function
        loop_rate.sleep();   //ros pause in 10Hz
    }

    return 0;
}