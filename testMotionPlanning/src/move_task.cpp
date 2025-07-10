#include"testMotionPlanning/move_task.h"
#include<ros/ros.h>
#include <cmath>

namespace GoalControl{

    move_task::move_task(ros::NodeHandle& nh) : nh_(nh){
        req_move = nh_.subscribe("/request_move", 20, &move_task::getRequestMove, this);
    }
    void move_task::getRequestMove(const sti_msgs::Move_request& data){
        // subscribe du lieu tu request move
        request_data = data;
        is_request_move = true;
    }
    void move_task::handle_request(){
        // lay du lieu tu request_move sti_control sau do lay du lieu tu day de xu ly
        if(request_data.list_x.size() == 5 && request_data.list_y.size() == 5 && request_data.list_id.size() == 5 && request_data.list_id[0] != 0.0){
            cur_goal_x = request_data.list_x[0];
            cur_goal_y = request_data.list_y[0];
            id_fl = request_data.list_id[0];
            vel_fl = request_data.list_speed[0];
            



            target_x = std::round(request_data.target_x * 1000.0)/1000.0;
            target_y = std::round(request_data.target_y * 1000.0)/1000.0;
            target_z = std::round(request_data.target_z * 1000.0)/1000.0;


            
        }

    }

    
}
