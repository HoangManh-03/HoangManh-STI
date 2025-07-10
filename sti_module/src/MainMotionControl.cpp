#include<sti_module/goalCPP.h>
#include<sti_module/PID.h>
using namespace GoalControl;

int main( int argc, char** argv ){
  ros::init(argc, argv, "goal_control_cpp");
  ros::NodeHandle n("~");
  ros::Rate rate;
//   rate = 20;
  
    while (ros::ok()){
        ROS_INFO("Hello world");
        rate.sleep();
    }
}
