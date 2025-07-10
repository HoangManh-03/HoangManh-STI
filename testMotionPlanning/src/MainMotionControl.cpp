#include "testMotionPlanning/goalControl.h"
#include "testMotionPlanning/pidControl.h"
#include <signal.h>
#include <cstdlib>

#define PI 3.1416

using namespace GoalControl; 

bool shutdownRequested = false;

void signalHandler(int signum) {
    ROS_INFO("Signal %d received, but not calling ros::shutdown()", signum);
    shutdownRequested = true;
}

double roundToDecimal(double value, int decimalPlaces) {
    double factor = std::pow(10.0, decimalPlaces);
    return std::round(value * factor) / factor;
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "run_test");
  ros::NodeHandle nh("~");

  int status = 0;
  double kp = 0.1;
  double ki = 0.0;
  double kd = 0.00001;

  goalControl goalcontrol(nh);
  PID pid(kp, ki, kd); 
  move_task move(nh);
  ros::Rate loop_rate(10);
  signal(SIGINT, signalHandler); 

  double theta; // 
  double targetx = 7.489;
  double targety = 0.459;

  double targetz = 3.1415;
  double tol_target = 0.05;
  double startx, starty;
  double X_projection, Y_projection, distance_projection;
  double v_th_send, v_x_send, vel_x_now; // 
  double v_test = 0.1;
  double angle_find_vel = 25.0*PI/180.0;
  double v_x;
  double min_vel = 0.2;
  double dis_gt = 1.15;
  double vel_x_control = 1.0;

  while(ros::ok() && !shutdownRequested){

        double theta = goalcontrol.Cal_angle(goalcontrol.poseRbMa.position.x, goalcontrol.poseRbMa.position.y, targetx, targety, goalcontrol.theta_rb_ht);
        
        if (status == 0){
          for(int i = 0; i < 10; i++){
            ROS_INFO("Hello world");
          }
          status = 1;
        }
        
        else if (status == 1){
          // ROS_INFO("pose rb: x = %f", goalcontrol.poseRbMa.position.x);
          ROS_INFO("theta: %f", theta);
          geometry_msgs::Twist twist;
          double gt = goalcontrol.turn_ar(theta, 0.05, 1.5);
          if (gt == -10){
            goalcontrol.stop2();
            ROS_INFO("da quay dung goc: ");
            startx = goalcontrol.poseRbMa.position.x;
            starty = goalcontrol.poseRbMa.position.y;
            status = 2;
            ros::Duration(0.5).sleep();

          }
          else{
            ROS_INFO("gt bang: %f", gt);
            twist.angular.z = gt;
            goalcontrol.pub_cmdVel(twist, 20);
            // move.handle_request();
          }
        }

        else if (status == 2){

          double distance_goal = goalcontrol.distance_two_point(goalcontrol.poseRbMa.position.x, goalcontrol.poseRbMa.position.y, targetx, targety);

          double distance_start = goalcontrol.distance_two_point(startx, starty, targetx, targety);
          std::tie(X_projection, Y_projection, distance_projection) = goalcontrol.find_hc(startx, starty, targetx, targety);

          double distance_2 = goalcontrol.distance_two_point(goalcontrol.poseRbMa.position.x, goalcontrol.poseRbMa.position.y, X_projection, Y_projection);

          ROS_INFO("distance_goal %f", distance_goal);
          geometry_msgs::Twist twist;

          if (std::fabs(theta) > angle_find_vel){
            v_x = min_vel;
          }
        
          else if (roundToDecimal(std::fabs(theta), 3) == 0.0){
            v_x = vel_x_control;
          }
          else{
            v_x = min_vel + ((angle_find_vel - std::fabs(theta)) / angle_find_vel) * (vel_x_control - min_vel);

          }

          if (distance_goal > dis_gt){
            vel_x_now = goalcontrol.accellometer(4.0,ros::Time::now().toSec(),0.0, v_x);
            v_x_send = vel_x_now;
            if (v_x_send >= v_x){
                v_x_send = v_x;
            }
          }
          else{
            if (vel_x_now >= 0.3){
              v_x_send = vel_x_now * (distance_goal / dis_gt);
              // ROS_INFO("Iiiiiiiiiiiiiiiiiiiiii");
            }
            else{
              v_x_send = v_x * (distance_goal / dis_gt);
            }
            if (v_x_send > v_x){
              v_x_send = v_x;
            }
          }


          if (distance_goal <= tol_target){
            ROS_INFO("Da den goal cuoi roi roi!!!!!!");
            ros::Duration(0.6).sleep();
            status = 4;
          }

          else{
            v_th_send = pid.calculate(distance_projection);
            ROS_INFO("v_th_send: %f", v_x_send);
            ROS_INFO("distance_projection: %f", distance_projection);
            twist.linear.x = v_x;
            twist.angular.z = v_th_send;
            goalcontrol.pub_cmdVel(twist, 20);  
          }
        }
        else if (status == 3){
          // dung lai diem dung
          goalcontrol.stop2();
        }

        else if (status == 4){
          // xoay dap ung goc
          theta = goalcontrol.theta_rb_ht - targetz;
          ROS_INFO("theta: %f", theta);
          geometry_msgs::Twist twist;
          double gt = goalcontrol.turn_ar(theta, 0.05, 1.5);
          if (gt == -10){
            goalcontrol.stop2();
            ROS_INFO("da quay dung goc: ");
            startx = goalcontrol.poseRbMa.position.x;
            starty = goalcontrol.poseRbMa.position.y;
            status = 5;
            ros::Duration(0.5).sleep();

          }
          else{
            ROS_INFO("gt bang: %f", gt);
            twist.angular.z = gt;
            goalcontrol.pub_cmdVel(twist, 20);
            // move.handle_request();
          }

        }

        else if (status == 5){
          goalcontrol.stop2();
          // lui vao lay ke
        }

        else if (status == 6){
          // dung lai
        }
        
      ros::spinOnce();     // allow receiving callbacks function
      loop_rate.sleep();
  }
  return 0;
}
