
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/String.h>

nav_msgs::Odometry odom_lidar, odom_temp ;
geometry_msgs::PoseWithCovarianceStamped pose_temp;
ros::Publisher odom_pub ;

bool enable_pub = false ;

void getOdom_hector(const nav_msgs::Odometry& odom)
{   
    odom_lidar = odom ;  
    enable_pub = true;
    // ROS_WARN("1");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "conver_lidar");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/scanmatch_odom", 10, getOdom_hector);
    ros::Publisher  pub1 = n.advertise<nav_msgs::Odometry>("odom_lidar", 10);
    ros::Publisher  pub2 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_lidar", 10);
    ros::Publisher  pub3 = n.advertise<std_msgs::String>("syscommand", 10);
    tf::TransformBroadcaster        odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans      ;
    ros::Rate r(15);
    double tg_ht, tg_tr ;
    std_msgs::String cmt ;
    cmt.data   = "reset";
    while(n.ok()){
        ros::spinOnce(); 
        
        tg_ht =ros::Time::now().toSec() ;
        // printf("time = %f \n", tg_ht);
        if(tg_ht - tg_tr > 30){ // 1ph xoa map 1 lan == 60s
            tg_tr = tg_ht ;
            ROS_WARN("xoa map hector");
            pub3.publish(cmt);
        }
        
        // ROS_WARN("2");
        if (enable_pub == true ){
            // ROS_WARN("3");

            // ROS_INFO("hihi");
            //odom pub
            // odom_temp.header.stamp   = ros::Time::now();
            // odom_temp.header.frame_id = "odom";
            // odom_temp.child_frame_id  = "base_footprint";
            // odom_temp.pose.pose.position    = odom_lidar.pose.pose.position ;
            // odom_temp.pose.pose.orientation = odom_lidar.pose.pose.orientation ;
            // odom_temp.pose.covariance = odom_lidar.pose.covariance;
            // pub1.publish(odom_temp);

            // pose pub
            pose_temp.header.stamp   = ros::Time::now();
            pose_temp.header.frame_id = "odom";
            pose_temp.pose.pose.position    = odom_lidar.pose.pose.position ;
            pose_temp.pose.pose.orientation = odom_lidar.pose.pose.orientation ;
            pose_temp.pose.covariance       = odom_lidar.pose.covariance;
            pub2.publish(pose_temp);


            //broadcaster
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id =  "base_foot";
            // odom_trans.child_frame_id =  "base_footprint";
            odom_trans.transform.translation.x = odom_lidar.pose.pose.position.x ;                                          
            odom_trans.transform.translation.y = odom_lidar.pose.pose.position.y ;
            odom_trans.transform.translation.z = odom_lidar.pose.pose.position.z ;
            odom_trans.transform.rotation.x    = odom_lidar.pose.pose.orientation.x ;
            odom_trans.transform.rotation.y    = odom_lidar.pose.pose.orientation.y ;
            odom_trans.transform.rotation.z    = odom_lidar.pose.pose.orientation.z ; 
            odom_trans.transform.rotation.w    = odom_lidar.pose.pose.orientation.w ; 
            odom_broadcaster.sendTransform(odom_trans);

        }
        r.sleep();
        
    }
}

