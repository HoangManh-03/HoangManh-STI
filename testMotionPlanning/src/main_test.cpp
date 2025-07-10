#include "testMotionPlanning/goalControl.h"
#include "testMotionPlanning/pidControl.h"
#include <signal.h>
#include <cstdlib>
#include "testMotionPlanning/bezier_curves.h"

#define PI 3.1416

using namespace GoalControl; 

// bool shutdownRequested = false;

// void signalHandler(int signum) {
//     ROS_INFO("Signal %d received, but not calling ros::shutdown()", signum);
//     shutdownRequested = true;
// }

// double roundToDecimal(double value, int decimalPlaces) {
//     double factor = std::pow(10.0, decimalPlaces);
//     return std::round(value * factor) / factor;
// }

int main( int argc, char** argv ){
  ros::init(argc, argv, "run_test");
  ros::NodeHandle nh("~");

//   goalControl goalcontrol(nh);
// //   PID pid(kp, ki, kd); 
//   move_task move(nh);
  ros::Rate loop_rate(10);
//   signal(SIGINT, signalHandler); 
  Point P0(0,0);
  Point P1(1,1);
  Point P2(2,2);
  Point P3(3,3);


    std::vector<Point> bezierCurve = generateBezierCurves(P0, P1, P2, P3, 100);
  while(ros::ok()){
    for (const auto& point : bezierCurve) {
        std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
    }
  }
  return 0;
}
