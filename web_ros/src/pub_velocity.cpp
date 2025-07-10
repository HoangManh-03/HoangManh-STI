#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

// Tỉ lệ điều chỉnh cho vận tốc
constexpr float LINEAR_SCALE = 1.0;
constexpr float ANGULAR_SCALE = 1.0;

class PubCmd {
public:
    PubCmd() : rate_pubVel(30) {
        ros::NodeHandle nh;
        
        // Khởi tạo Publisher và Subscriber
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        sub = nh.subscribe("/web_info", 10, &PubCmd::joyCallback, this);
        
        joy_data.resize(5, 0); // Khởi tạo kích thước cho mảng joy_data
        time_tr = ros::Time::now();
    }

    void joyCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() >= 5) {
            joy_data = msg->data;
            // ROS_INFO("joy_data: %s", joy_data[2]);
            ROS_INFO("Dang nhan data tu topic");
        }
    }

    void pubCmdVel(const geometry_msgs::Twist& twist, int rate) {
        if ((ros::Time::now() - time_tr).toSec() > 1.0 / rate) {
            time_tr = ros::Time::now();
            pub.publish(twist);
        }
    }

    void run() {
        ros::Rate rate(rate_pubVel);

        while (ros::ok()) {
            // Kiểm tra nếu joy_data có đủ phần tử trước khi truy cập


            if (joy_data.size() >= 5) {
                float x = joy_data[2];
                float y = joy_data[3];
                float w = joy_data[4];

                // Tính toán vận tốc tuyến tính và góc
                float a = -y / w;
                float b = -x / w;

                geometry_msgs::Twist vel;
                vel.linear.x = LINEAR_SCALE * a;
                vel.angular.z = ANGULAR_SCALE * b;

                pubCmdVel(vel, rate_pubVel);
            } else {
                ROS_WARN("joy_data không đủ phần tử: %zu", joy_data.size());
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Publisher pub;
    ros::Subscriber sub;
    std::vector<float> joy_data;
    ros::Time time_tr;
    int rate_pubVel;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_joystick");
    
    PubCmd pub_cmd;
    pub_cmd.run();
    
    return 0;
}
