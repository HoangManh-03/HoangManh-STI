/*
 * sicknav350_node.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: Punith Mallesha
 *
 * Based on the sicklms.cpp and sickld.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 * 
 * Released under BSD license.
 */

#include <iostream>
#include <sicktoolbox/SickNAV350.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <deque>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// - add
#include <message_pkg/Nav350_data.h>
#include <message_pkg/Reflector_array.h>
#include <message_pkg/Reflector_data.h>

#define DEG2RAD M_PI/180.0

using namespace std;
using namespace SickToolbox;

#define DEFAULT_SICK_IP_ADDRESS "192.168.100.10"  ///< Default Sick LD INet 4 address
#define DEFAULT_SICK_TCP_PORT   (2111)  ///< Default TCP port Le Duc Anh chinh sua 2/2/2024

void publish_scan(ros::Publisher *pub, double *range_values,
                  uint32_t n_range_values, unsigned int *intensity_values,
                  uint32_t n_intensity_values, ros::Time start,
                  double scan_time, bool inverted, float angle_min,
                  float angle_max, std::string frame_id,
                  unsigned int sector_start_timestamp)
{
	static int scan_count = 0;
	sensor_msgs::LaserScan scan_msg;
	scan_msg.header.frame_id = frame_id;
	scan_count++;
	if (inverted) { 
		scan_msg.angle_min = angle_max*DEG2RAD;
		scan_msg.angle_max = angle_min*DEG2RAD;
	} else {
		scan_msg.angle_min = angle_min*DEG2RAD;
		scan_msg.angle_max = angle_max*DEG2RAD;
	}

	scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(n_range_values-1);
	scan_msg.scan_time = 0.125;//scan_time 125ms;
	scan_msg.time_increment = scan_msg.scan_time / n_range_values;
	scan_msg.range_min = 0.1;
	scan_msg.range_max = 250.;
	scan_msg.ranges.resize(n_range_values);
	scan_msg.header.stamp = start;

	for (size_t i = 0; i < n_range_values; i++) {
		scan_msg.ranges[i] = (float)range_values[i]/1000;
	}

		scan_msg.intensities.resize(n_intensity_values);

	for (size_t i = 0; i < n_intensity_values; i++) {
		scan_msg.intensities[i] = 0;//(float)intensity_values[i];
	}

	pub->publish(scan_msg);
}

void PublishLaserTransform(tf::TransformBroadcaster laser_broadcaster, std::string header_frame_id, std::string child_frame_id)
{
	laser_broadcaster.sendTransform(
	tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.2374)),
	ros::Time::now(),header_frame_id, child_frame_id)); // distance from the focal point of the scanner to its base (199.4mm) + offset from the mount (38mm)

} //you can also define a customized urdf model using the nav350 meshes given


//necessary for sensor fusion using robot_localization package
void PublishLaserOdometry(double x, double y, double th, ros::Publisher *pub, std::string frame_id, std::string laser_child_frame_id)
{
	ros::Time current_time;
	current_time = ros::Time::now();
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = frame_id;
	odom.child_frame_id = laser_child_frame_id;
	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0;
	odom.pose.pose.orientation = odom_quat;

	pub->publish(odom);
}

// position as tf 
void PublishPositionTransform(double x, double y, double th, tf::TransformBroadcaster odom_broadcaster, std::string header_frame_id, std::string child_frame_id)
{
	ros::Time current_time;
	current_time=ros::Time::now();
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = header_frame_id;// "map"
	odom_trans.child_frame_id = child_frame_id;// "reflector or base or odom frame"

	odom_trans.transform.translation.x = x;//global x coordinate 
	odom_trans.transform.translation.y = y; //global y coordinate 
	odom_trans.transform.translation.z = 0; 
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
}

std::string GetError_str(int number_error)
{
	std::string mess;
	
	switch (number_error)
	{
		case 0:
		{
			mess = "no error";
			break;
		}
		case 1:
		{
			mess = "wrong operating mode";
			break;
		}
		case 2:
		{
			mess = "asynchrony Method terminated";
			break;
		}
		case 3:
		{
			mess = "invalid data";
			break;
		}
		case 4:
		{
			mess = "no position available";
			break;
		}
		case 5:
		{
			mess = "timeout";
			break;
		}
		case 6:
		{
			mess = "method already active";
			break;
		}
		case 7:
		{
			mess = "general error";
			break;
		}
		default:
		{
			mess = "Not matching";
		}
	}
	return mess;
}

// std::string GetStr_InfoState(int number)
// {
// 	std::string mess;
	
// 	switch (number)
// 	{
// 		case 0:
// 		{
// 			mess = "No external speed available";
// 			break;
// 		}
// 		case 1:
// 		{
// 			mess = "No internal speed available";
// 			break;
// 		}
// 		case 2:
// 		{
// 			mess = "Internal prediction used";
// 			break;
// 		}
// 		case 3:
// 		{
// 			mess = "Simple prediction used";
// 			break;
// 		}
// 		case 4:
// 		{
// 			mess = "No prediction used";
// 			break;
// 		}
// 		case 5:
// 		{
// 			mess = "No speed compensation done";
// 			break;
// 		}
// 		case 6:
// 		{
// 			mess = "Not enough landmarks in layer";
// 			break;
// 		}
// 		case 7:
// 		{
// 			mess = "No landmarks available";
// 			break;
// 		}
// 		case 7:
// 		{
// 			mess = "Less than 3 landmarks used";
// 			break;
// 		}
// 		case 7:
// 		{
// 			mess = "Positioning failed";
// 			break;
// 		}
// 		case 7:
// 		{
// 			mess = "Positioning reinitialized with internal speed";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Positioning reinitialized";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Positioning reinitialized";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Position is extrapolated";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Filter operating radii was skipped";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Filter sector muting was skipped";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Filter NClosest was skipped";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Speed computation done";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Prediction done";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Correlation done";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Reinitialized";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Reinitialized";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Landmark selection done";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Positioning done";
// 			break;
// 		}
// 		case :
// 		{
// 			mess = "Positioning finished";
// 			break;
// 		}

// 		default:
// 		{
// 			mess = "Not Matching!";
// 		}
// 	}
// 	return mess;
// }

// odometry call back from the robot 
double vx, vy, vth;
void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vx = msg->twist.twist.linear.x;
	vy = msg->twist.twist.linear.y;
	vth = msg->twist.twist.angular.z;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "sicknav350");

	int port;
	std::string ipaddress;
	std::string frame_id, fixed_frame_id;
	std::string odometry;
	std::string scan;
	bool inverted;
	bool publish_tf_, publish_odom_, publish_scan_;
	int sick_motor_speed = 8; //10; // Hz
	double sick_step_angle = 1.5; //0.5;//0.25; 
	double active_sector_start_angle = 0;
	double active_sector_stop_angle = 360;//269.75;
	std::string laser_frame_id,laser_child_frame_id,odom_frame_id;
	// --
	message_pkg::Nav350_data nav350_data;
	message_pkg::Reflector_array reflector_array;
	// --
  	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");

	nh_ns.param<bool>("publish_tf", publish_tf_, true);
	nh_ns.param<bool>("publish_odom_", publish_odom_, true);
	nh_ns.param<bool>("publish_scan", publish_scan_, true);

	nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
	nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
	
	nh_ns.param("inverted", inverted, false);

  	nh_ns.param<std::string>("scan", scan, "scan");
	nh_ns.param<std::string>("frame_id", frame_id, "frame_nav350"); //laser frame for scan data
	nh_ns.param<std::string>("fixed_frame_id", fixed_frame_id, "frame_nav350"); // nav350 mount position frame on the robot | frame_map_nav350
	nh_ns.param<std::string>("laser_frame_id", laser_frame_id, "frame_map_nav350"); // global cooridnate frame measurement for navigation and position based on reflectors
	nh_ns.param<std::string>("laser_child_frame_id", laser_child_frame_id, "frame_nav350"); // a fixed frame eg: odom or base or reflector | "reflector"

	nh_ns.param("resolution", sick_step_angle, 1.0);
	nh_ns.param("start_angle", active_sector_start_angle, 0.);
	nh_ns.param("stop_angle", active_sector_stop_angle, 360.);
	nh_ns.param("scan_rate", sick_motor_speed, 5);

	ros::Subscriber sub = nh.subscribe("odometry/filtered", 10, OdometryCallback); // data from sensor fusion or wheel odometry of jackal robot

	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 100);

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("nav350laser/odom", 10);
	
	ros::Publisher pub_nav350_data = nh.advertise<message_pkg::Nav350_data>("nav350_data", 10);
	
	ros::Publisher reflector_pub = nh.advertise<message_pkg::Reflector_array>("nav350_reflectors", 10);
	/* Define buffers for return values */
	double range_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
	unsigned int intensity_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};

	/* Define buffers to hold sector specific data */
	unsigned int num_measurements = {0};
	unsigned int sector_start_timestamp = {0};
	unsigned int sector_stop_timestamp = {0};
	double sector_step_angle = {0};
	double sector_start_angle = {0};
	double sector_stop_angle = {0};

	/* Instantiate the object */
	SickNav350 sick_nav350(ipaddress.c_str(),port);

	//  ros::Duration(50).sleep(); //timedelay for jackal robot startup jobs
	double last_time_stamp = 0;
	try {
		ROS_INFO("Version edit by STI VietNam - Bee ");
		/* Initialize the device */
		sick_nav350.Initialize();

		// try{
		// 	sick_nav350.SetReflectorSize(60);
		// } catch (...) {
		// 	ROS_ERROR("SetReflectorSize: error");
		// 	// return -1;
		// }

		// try{
		// 	// 1 = flat
		// 	// 2 = cylindric (Default)
		// 	sick_nav350.SetReflectorType(2);
		// } catch (...) {
		// 	ROS_ERROR("SetReflectorType: error");
		// 	// return -1;
		// }

		// try{
		// 	sick_nav350.SetReflectorThreshold(40);
		// } catch (...) {
		// 	ROS_ERROR("SetReflectorThreshold: error");
		// 	// return -1;
		// }

		try {
			sick_nav350.SetLandmarkDataFormat(1, 1, 1); // (Format, ShowOptParam, LandmarkFilter)
			/*
			Format
				0 = Cartesian (Default)
				1 = polar
			ShowOptParam: 
				0 = suppress (Default)
				1 = enable
			LandmarkFilter:
				0 = used
				1 = detected (Default)
				2 = expected landmark
			*/
		} catch (...) {
			ROS_ERROR("SetLandmarkDataFormat: error");
			return -1;
		}


		// try{
		// 	sick_nav350.SetReflectorSize(60);
		// } catch (...) {
		// 	ROS_ERROR("SetReflectorSize: error");
		// 	// return -1;
		// }

		// try{
		// 	sick_nav350.SetReflectorThreshold(40);
		// } catch (...) {
		// 	ROS_ERROR("SetReflectorThreshold: error");
		// 	// return -1;
		// }

		try {
			sick_nav350.SetDataGet(1, 2); // (wait, Mask)
			/*
			wait
				0 = instantly last pose result
				1 = wait for next pose result
			Mask: 
				0 = pose + reflectors
				1 = pose + scan
				2 = pose + reflectors + scan
			*/
		} catch (...) {
			ROS_ERROR("SetDataGet: error");
			return -1;
		}

		try {
			sick_nav350.SetShowOption(1, 1); // (OutputMode, ShowOptParam)
			/*
			OutputMode
				0 = normal
				1 = extrapolated (Default)
			ShowOptParam: 
				0 = suppressed (Default)
				1 = enabled
			*/
		} catch (...) {
			ROS_ERROR("SetShowOption: error");
			return -1;
		}

		try {
			sick_nav350.SetOperatingMode(4); // navigation Mode
			/*
				0 = power down
				1 = standby (Default)
				2 = mapping
				3 = landmark detection
				4 = navigation
			*/
		} catch (...) {
			ROS_ERROR("SetOperatingMode: error");
			return -1;
		}
		

		ros::Time last_start_scan_time;
		unsigned int last_sector_stop_timestamp = 0;
		ros::Rate loop_rate(8);
		tf::TransformBroadcaster odom_broadcaster;
		tf::TransformBroadcaster laser_broadcaster;

		while (ros::ok()) {
			/* Get the scan and landmark measurements */
			sick_nav350.GetDataNavigation(1, 2); //n(1, 1) 
			sick_nav350.GetSickMeasurements(range_values,
										&num_measurements,
										&sector_step_angle,
										&sector_start_angle,
										&sector_stop_angle,
										&sector_start_timestamp,
										&sector_stop_timestamp
										);
			// sick_nav350.GetDataLandMark(1 , 0);

			double x1 = (double)sick_nav350.PoseData_.x;
			double y1 = (double)sick_nav350.PoseData_.y;
			double phi1 = sick_nav350.PoseData_.phi;
			
			double x2, y2;
			double phi2 = phi1 - 180000 - 1250 - 300;

			phi2 = phi2/1000*3.141592653/180;
			x2 = x1/1000;
			y2 = y1/1000;

			// -----------------------
			nav350_data.header.stamp = ros::Time::now();
			nav350_data.version_positioning = sick_nav350.PoseData_.versionPositioning;
			nav350_data.error_code = GetError_str(sick_nav350.PoseData_.numberError);
			nav350_data.wait = sick_nav350.PoseData_.wait;
			nav350_data.mask = sick_nav350.PoseData_.mask;
			nav350_data.poseDataEnb = sick_nav350.PoseData_.poseDataEnb;
			nav350_data.x = x2;
			nav350_data.y = y2;
			nav350_data.phi = phi2;
			nav350_data.optPoseData = sick_nav350.PoseData_.optionalPoseData;
			
			nav350_data.output_mode = sick_nav350.PoseData_.outputMode;
			nav350_data.time_stamp = sick_nav350.PoseData_.timeStamp;
			nav350_data.mean_dev = sick_nav350.PoseData_.meanDeviation;
			nav350_data.nav_mode = sick_nav350.PoseData_.positionMode;
			nav350_data.info_state = sick_nav350.PoseData_.infoState;
			nav350_data.number_reflectors = sick_nav350.PoseData_.numUsedReflectors;
			
			pub_nav350_data.publish(nav350_data);
			// -----------------------

			if(publish_tf_) // && nav350_data.number_reflectors >= 3)
			{
				PublishPositionTransform(x2, y2, phi2, odom_broadcaster, laser_frame_id, laser_child_frame_id); //publish position data as transform in map frame 
				// PublishLaserTransform(laser_broadcaster, fixed_frame_id, frame_id); // publish laser transform with respect to base frame (scan data)
			}else{
				
			}

			if(publish_odom_)
			{
				PublishLaserOdometry(x2, y2, phi2, &odom_pub, laser_frame_id, laser_child_frame_id); // publish odometry data from nav350 for sensor fusion
			}

			if (sector_start_timestamp < last_time_stamp)
			{
				loop_rate.sleep();
				ros::spinOnce();
				continue;
			}

			last_time_stamp = sector_start_timestamp;
			ros::Time end_scan_time = ros::Time::now();

			double scan_duration = 0.125;

			ros::Time start_scan_time = end_scan_time - ros::Duration(scan_duration);
			sector_start_angle -= 180;
			sector_stop_angle -= 180;

			if(publish_scan_)
			{
				publish_scan(&scan_pub, range_values, num_measurements, intensity_values,
							num_measurements, start_scan_time, scan_duration, inverted,
							(float)sector_start_angle, (float)sector_stop_angle, frame_id,sector_start_timestamp);
			}

			last_start_scan_time = start_scan_time;
			last_sector_stop_timestamp = sector_stop_timestamp;

			// sick_nav350.SetSpeed(vx, vy, vth, sector_start_timestamp, 0);

			/* -- reflector_pub -- */
			reflector_array.filter = sick_nav350.ReflectorData_.filter;
			reflector_array.num_reflector = sick_nav350.ReflectorData_.num_reflector;
			reflector_array.reflectors.clear();

			for (int i = 0; i < reflector_array.num_reflector; i++)
			{
				message_pkg::Reflector_data reflector_data;
				// reflector_array.append(reflector_data);

				reflector_data.Cart = sick_nav350.ReflectorData_.cart[i];
				reflector_data.Cart_X = sick_nav350.ReflectorData_.x[i];
				reflector_data.Cart_Y = sick_nav350.ReflectorData_.y[i];
				reflector_data.Polar = sick_nav350.ReflectorData_.polar[i];
				reflector_data.Polar_Dist = sick_nav350.ReflectorData_.dist[i];
				reflector_data.Polar_Phi = sick_nav350.ReflectorData_.phi[i];

				reflector_data.optLandmarkData = sick_nav350.ReflectorData_.optional[i];
				reflector_data.LocalID = sick_nav350.ReflectorData_.LocalID[i];
				reflector_data.GlobalID = sick_nav350.ReflectorData_.GlobalID[i];
				reflector_data.Landmark_type = sick_nav350.ReflectorData_.type[i];
				reflector_data.Reflector_type = sick_nav350.ReflectorData_.subtype[i];
				reflector_data.Quality = sick_nav350.ReflectorData_.quality[i];
				reflector_data.Timestamp = sick_nav350.ReflectorData_.timestamp[i];
				reflector_data.Size = sick_nav350.ReflectorData_.size[i];
				reflector_data.HitCount = sick_nav350.ReflectorData_.hitCount[i];
				reflector_data.Mean_Echo = sick_nav350.ReflectorData_.meanEchoAmplitude[i];
				reflector_data.Index_Begin = sick_nav350.ReflectorData_.indexStart[i];
				reflector_data.Index_End = sick_nav350.ReflectorData_.indexEnd[i];

				reflector_array.reflectors.push_back(reflector_data);
			}

			reflector_array.header.stamp = ros::Time::now();
			reflector_array.header.frame_id = laser_frame_id;
			reflector_pub.publish(reflector_array);
			loop_rate.sleep();
			ros::spinOnce();
		}

		/* Uninitialize the device */
		sick_nav350.Uninitialize();

	}catch(...) {
		ROS_ERROR("Error!");
		return -1;
	}
	return 0;
}
