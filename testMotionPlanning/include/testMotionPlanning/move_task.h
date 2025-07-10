#include<iostream>
#include<ros/ros.h>
#include<sti_msgs/NN_cmdRequest.h>
#include<sti_msgs/Move_request.h>

namespace GoalControl{
class move_task{
    public:

        move_task(ros::NodeHandle& nh);  // Constructor declaration
        void getRequestMove(const sti_msgs::Move_request& data);
        void handle_request();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber req_move;
    sti_msgs::Move_request request_data;
    bool is_request_move;
    double cur_goal_x, cur_goal_y, id_fl, vel_fl, target_x, target_y,target_z;

    };
}
