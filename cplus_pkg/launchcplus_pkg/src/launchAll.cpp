// Author: Phùng Quý Dương
// DATE  : 11/09/2023

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include "sti_msgs/NN_infoRespond.h"
#include "sti_msgs/NN_infoRequest.h"
#include "sti_msgs/POWER_info.h"
#include "sti_msgs/HC_info.h"
#include "sti_msgs/Lift_status.h"
#include "sti_msgs/Status_goal_control.h"

#include "message_pkg/Status_port.h"
#include "message_pkg/Status_launch.h"
#include "message_pkg/Status_reconnect.h"
#include "message_pkg/Driver_respond.h"
#include "message_pkg/Loadcell_respond.h"
#include "message_pkg/Driver_query.h"
#include "message_pkg/Parking_respond.h"

#include <bits/stdc++.h>
#include <unistd.h>
#include <Python.h>
using namespace std;

class Launch{
    public:
    string filelaunch;
    string command;
    int process = 0;
    time_t time_pre = time(NULL);
    // char buf[100];

    // Launch(string file_launch){
    //     filelaunch = file_launch;
    //     // command = "roslaunch " + filelaunch;
    //     process = 0;
    //     time_pre = time(NULL);
    //     // std::cout << time_pre;
        
    // }

    void getFileLaunch(string file_launch)
    {
        filelaunch = file_launch;
    }

    void start(){
        if (process == 0){
            Py_Initialize();
            PyRun_SimpleString("import roslaunch\n");
            PyRun_SimpleString("uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)\n");
            PyRun_SimpleString("roslaunch.configure_logging(uuid)\n");
            string cmd_launch = "launch = roslaunch.parent.ROSLaunchParent(uuid, [\"" +  filelaunch + "\"])\n";
            PyRun_SimpleString(cmd_launch.c_str());
            PyRun_SimpleString("launch.start()\n");
            // Py_Finalize();           

            process = 1;
        }
    }

    int start_and_wait(int time_wait){
        if (process == 0){
            Py_Initialize();
            PyRun_SimpleString("import roslaunch\n");
            PyRun_SimpleString("uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)\n");
            PyRun_SimpleString("roslaunch.configure_logging(uuid)\n");
            string cmd_launch = "launch = roslaunch.parent.ROSLaunchParent(uuid, [\"" +  filelaunch + "\"])\n";
            PyRun_SimpleString(cmd_launch.c_str());
            PyRun_SimpleString("launch.start()\n");
            // Py_Finalize();   
            
            process = 1;
            time_pre = time(NULL);
            return 0;
        }

        else if (process == 1){
            double t = (time(NULL) - time_pre)%60;
            if (t > time_wait){
                process = 2;

            }
            return 0;
        }
        else{
            return 1;
        }
    }
};


class Program
{            
public:
    string path_firstWork;
    string path_checkPort;
    string path_reconnectBase;
    string path_reconnectDriver;
    string path_main;
    string path_driverReconnect;
    string path_nav350;
    string path_hc;
    string path_oc;

    string path_imu;
    string path_loadcell;
    string path_imuFilter;
    string path_kinematic;
    string path_robotPoseNav;
    string path_ekf;
    string path_safetyNav350;
    string path_goalControl;
    string path_parkingControl;
    string path_client;
    string path_control;
    string path_posePublisher;
    string path_dataApp;

    string notification = "";
    uint8_t count_node = 0;
    uint8_t step = 0;
    float timeWait = 0.4;

    // publish topic 
    ros::Publisher pub_statusLaunch;
    message_pkg::Status_launch statusLaunch;

    //subscribe topic
    ros::Subscriber sub_firstWork;
    Launch launch_firstWork;
    uint8_t is_firstWork = 0;

    ros::Subscriber sub_checkPort;
    Launch launch_checkPort;
    uint8_t is_checkPort = 0;

    ros::Subscriber sub_reconnectBase;
    Launch launch_reconnectBase;
    uint8_t is_reconnectBase = 0;

    ros::Subscriber sub_reconnectDriver;
    Launch launch_reconnectDriver;
    uint8_t is_reconnectDriver = 0;

    ros::Subscriber sub_main;
    Launch launch_main;
    uint8_t is_main = 0;

    ros::Subscriber sub_driver1;
    ros::Subscriber sub_driver2;
    Launch launch_driverReconnect;
    uint8_t is_driverLeft = 0;
    uint8_t is_driverRight = 0;

    ros::Subscriber sub_nav350;
    Launch launch_nav350;
    uint8_t is_nav350 = 0;

    ros::Subscriber sub_hc;
    Launch launch_hc;
    uint8_t is_hc = 0;

    ros::Subscriber sub_oc;
    Launch launch_oc;
    uint8_t is_oc = 0;

    ros::Subscriber sub_imu;
    Launch launch_imu;
    uint8_t is_imu = 0;

    ros::Subscriber sub_loadcell;
    Launch launch_loadcell;
    uint8_t is_loadcell = 0;

    ros::Subscriber sub_imuFilter;
    Launch launch_imuFilter;
    uint8_t is_imuFilter = 0;

    ros::Subscriber sub_kinematic;
    Launch launch_kinematic;
    uint8_t is_kinematic = 0;

    ros::Subscriber sub_robotPoseNav;
    Launch launch_robotPoseNav;
    uint8_t is_robotPoseNav = 0;

    ros::Subscriber sub_ekf;
    Launch launch_ekf;
    uint8_t is_ekf = 0;

    ros::Subscriber sub_safetyNav350;
    Launch launch_safetyNav350;
    uint8_t is_safetyNav350 = 0;

    ros::Subscriber sub_goalControl;
    Launch launch_goalControl;
    uint8_t is_goalControl = 0;

    ros::Subscriber sub_parkingControl;
    Launch launch_parkingControl;
    uint8_t is_parkingControl = 0;

    ros::Subscriber sub_client;
    Launch launch_client;
    uint8_t is_client = 0;

    ros::Subscriber sub_control;
    Launch launch_control;
    uint8_t is_control = 0;

    ros::Subscriber sub_dataApp;
    Launch launch_dataApp;

    ros::Subscriber sub_posePublisher;
    Launch launch_posePublisher;
    uint8_t is_posePublisher = 0;

    Program(ros::NodeHandle *nh, ros::NodeHandle *npr){
        ros::param::get("path_firstWork", path_firstWork);
        launch_firstWork.getFileLaunch(path_firstWork);
        sub_firstWork = nh->subscribe("/first_work/run", 10, &Program::callBack_firstWork, this);

        ros::param::get("path_checkPort", path_checkPort);
        launch_checkPort.getFileLaunch(path_checkPort);
        sub_checkPort = nh->subscribe("/status_port", 10, &Program::callBack_checkPort, this);

        ros::param::get("path_reconnectBase", path_reconnectBase);
        launch_reconnectBase.getFileLaunch(path_reconnectBase);
        sub_reconnectBase = nh->subscribe("/status_reconnect", 10, &Program::callBack_reconnectBase, this);

        ros::param::get("path_reconnectDriver", path_reconnectDriver);
        launch_reconnectDriver.getFileLaunch(path_reconnectDriver);
        sub_reconnectDriver = nh->subscribe("/status_reconnectDriver", 10, &Program::callBack_reconnectDriver, this);

        ros::param::get("path_main", path_main);
        launch_main.getFileLaunch(path_main);
        sub_main = nh->subscribe("/POWER_info", 10, &Program::callBack_main, this);

        ros::param::get("path_driverReconnect", path_driverReconnect);
        launch_driverReconnect.getFileLaunch(path_driverReconnect);
        sub_driver1 = nh->subscribe("/driver1_respond", 10, &Program::callBack_driverLeft, this);
        sub_driver2 = nh->subscribe("/driver2_respond", 10, &Program::callBack_driverRight, this);

        ros::param::get("path_nav350", path_nav350);
        launch_nav350.getFileLaunch(path_nav350);
        sub_nav350 = nh->subscribe("/scan", 10, &Program::callBack_nav350, this);

        ros::param::get("path_hc", path_hc);
        launch_hc.getFileLaunch(path_hc);
        sub_hc = nh->subscribe("/HC_info", 10, &Program::callBack_hc, this);
    
        ros::param::get("path_oc", path_oc);
        launch_oc.getFileLaunch(path_oc);
        sub_oc = nh->subscribe("/OC_info", 10, &Program::callBack_oc, this);

        ros::param::get("path_imu", path_imu);
        launch_imu.getFileLaunch(path_imu);
        sub_imu = nh->subscribe("/imu/data", 10, &Program::callBack_imu, this);

        ros::param::get("path_loadcell", path_loadcell);
        launch_loadcell.getFileLaunch(path_loadcell);
        sub_loadcell = nh->subscribe("/loadcell_respond", 10, &Program::callBack_loadcell, this);

        ros::param::get("path_imuFilter", path_imuFilter);
        launch_imuFilter.getFileLaunch(path_imuFilter);
        sub_imuFilter = nh->subscribe("/imu_filter", 10, &Program::callBack_imuFilter, this);

        ros::param::get("path_kinematic", path_kinematic);
        launch_kinematic.getFileLaunch(path_kinematic);
        sub_kinematic = nh->subscribe("/driver1_query", 10, &Program::callBack_kinematic, this);

        ros::param::get("path_robotPoseNav", path_robotPoseNav);
        launch_robotPoseNav.getFileLaunch(path_robotPoseNav);
        sub_robotPoseNav = nh->subscribe("/robotPose_nav", 10, &Program::callBack_robotPoseNav, this);

        ros::param::get("path_ekf", path_ekf);
        launch_ekf.getFileLaunch(path_ekf);
        sub_ekf = nh->subscribe("/odometry", 10, &Program::callBack_ekf, this);

        ros::param::get("path_safetyNav350", path_safetyNav350);
        launch_safetyNav350.getFileLaunch(path_safetyNav350);
        sub_safetyNav350 = nh->subscribe("/safety_NAV", 10, &Program::callBack_safetyNav350, this);

        ros::param::get("path_goalControl", path_goalControl);
        launch_goalControl.getFileLaunch(path_goalControl);
        sub_goalControl = nh->subscribe("/status_goal_control", 10, &Program::callBack_goalControl, this);

        ros::param::get("path_parkingControl", path_parkingControl);
        launch_parkingControl.getFileLaunch(path_parkingControl);
        sub_parkingControl = nh->subscribe("/parking_respond", 10, &Program::callBack_parkingControl, this);

        ros::param::get("path_client", path_client);
        launch_client.getFileLaunch(path_client);
        sub_client = nh->subscribe("/NN_infoRequest", 10, &Program::callBack_client, this);

        ros::param::get("path_control", path_control);
        launch_control.getFileLaunch(path_control);
        sub_control = nh->subscribe("/NN_infoRespond", 10, &Program::callBack_control, this);

        ros::param::get("path_dataApp", path_dataApp);
        launch_dataApp.getFileLaunch(path_dataApp);

        ros::param::get("path_posePublisher", path_posePublisher);
        launch_posePublisher.getFileLaunch(path_posePublisher);
        sub_posePublisher = nh->subscribe("/robot_pose", 10, &Program::callBack_posePublisher, this);

        pub_statusLaunch = nh->advertise<message_pkg::Status_launch>("/status_launch", 10);

    }

    ~Program(){};

    void callBack_firstWork(const std_msgs::Int16& msg){
        is_firstWork = 1;
        // ROS_INFO("firstWork value = %d", is_firstWork);
    }

    void callBack_checkPort(const message_pkg::Status_port& msg){
        is_checkPort = 1;
        // ROS_INFO("checkPort value = %d", is_checkPort);
    }

    void callBack_reconnectBase(const message_pkg::Status_reconnect& msg){
        is_reconnectBase = 1;
        // ROS_INFO("reconnectBase value = %d", is_reconnectBase);
    }

    void callBack_reconnectDriver(const message_pkg::Status_reconnect& msg){
        is_reconnectDriver = 1;
        // ROS_INFO("reconnectDriver value = %d", is_reconnectDriver);
    }

    void callBack_main(const sti_msgs::POWER_info& msg){
        is_main = 1;
    }

    void callBack_driverLeft(const message_pkg::Driver_respond& msg){
        is_driverLeft = 1;
    }

    void callBack_driverRight(const message_pkg::Driver_respond& msg){
        is_driverRight = 1;
    }

    void callBack_nav350(const sensor_msgs::LaserScan& msg){
        is_nav350 = 1;
    }

    void callBack_hc(const sti_msgs::HC_info& msg){
        is_hc = 1;
    }

    void callBack_oc(const sti_msgs::Lift_status& msg){
        is_oc = 1;
    }

    void callBack_imu(const sensor_msgs::Imu& msg){
        is_imu = 1;
    }

    void callBack_loadcell(const message_pkg::Loadcell_respond& msg){
        is_loadcell = 1;
    }

    void callBack_imuFilter(const sensor_msgs::Imu& msg){
        is_imuFilter = 1;
    }

    void callBack_kinematic(const message_pkg::Driver_query& msg){
        is_kinematic = 1;
    }

    void callBack_robotPoseNav(const geometry_msgs::PoseStamped& msg){
        is_robotPoseNav = 1;
    }

    void callBack_ekf(const nav_msgs::Odometry& msg){
        is_ekf = 1;
    }

    void callBack_safetyNav350(const std_msgs::Int8& msg){
        is_safetyNav350 = 1;
    }

    void callBack_goalControl(const sti_msgs::Status_goal_control& msg){
        is_goalControl = 1;
    }

    void callBack_parkingControl(const message_pkg::Parking_respond& msg){
        is_parkingControl = 1;
    }

    void callBack_client(const sti_msgs::NN_infoRequest& msg){
        is_client = 1;
    }

    void callBack_control(const sti_msgs::NN_infoRespond& msg){
        is_control = 1;
    }

    void callBack_posePublisher(const geometry_msgs::Pose& msg){
        is_posePublisher = 1;
    }

    void run()
    {
        if (step == 0){
            notification = "launch_firstWork";
            launch_firstWork.start();
            if (is_firstWork == 1){
                step += 1;
                usleep(timeWait*1000000);
            }
        }
        // -- checkPort
        else if (step == 1){
            notification = "launch_checkPort";
            launch_checkPort.start();
            if (is_checkPort == 1){
                step += 1;
                usleep(timeWait*1000000);
            }
        }

        // -- reconnectBase
        else if (step == 2){
            notification = "launch_reconnectBase";
            launch_reconnectBase.start();
            if (is_reconnectBase == 1){
                step += 1;
                usleep(timeWait*1000000);
            }
        }

        // -- reconnectDriver
        else if (step == 3){
            notification = "launch_reconnectDriver";
            step += 1;
            launch_reconnectDriver.start();
            if (is_reconnectDriver == 1){
                step += 1;
                usleep(timeWait*1000000);
            }
        }

        // -- data APP
        else if (step == 4){
            notification = "launch_dataApp";
            step += 1;
        }

        // -- main
        else if (step == 5){
            notification = "launch_main";
            launch_main.start();
            if (is_main == 1){
                step += 1;
                usleep(timeWait*1000000);
            }
        }

        // -- hc
        else if (step == 6){
            notification = "launch_hc";
            step = 7;
        }
        // -- oc
        // -- imu
        else if (step == 7){
            notification = "launch_imu";
            step = 8;
        }

        // -- driverAll
        else if (step == 8){
            notification = "launch_driverReconnect";
            int sts_driver = launch_driverReconnect.start_and_wait(3.);
            if(sts_driver == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }
        
        // -- nav350
        else if (step == 9){
            notification = "launch_nav350";
            launch_nav350.start();
            if(is_nav350 == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }

        // -- data APP
        else if (step == 10){
            notification = "launch_dataApp";
            try{
                int sts_app = launch_dataApp.start_and_wait(2.);
                if(sts_app == 1){
                    step += 1;
                    usleep(timeWait*1000000); 
                }
            }
            catch(...){
                step += 1;
            }
        }

        // -- kinematic
        else if (step == 11){
            notification = "launch_kinematic";
            launch_kinematic.start();
            if(is_kinematic == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }

        // -- get pose robot from nav.
        else if (step == 12){
            notification = "launch_robotPoseNav";
            launch_robotPoseNav.start();
            if(is_robotPoseNav == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }

        // -- ekf
        else if (step == 13){
            notification = "launch_ekf";
            int sts_ekf = launch_ekf.start_and_wait(3.);
            if(is_ekf == 1 || sts_ekf == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }

        // -- safety zone Nav350
        else if (step == 14){
            notification = "launch_safetyNav350";
            launch_safetyNav350.start();
            if(is_safetyNav350 == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }

        // -- goal control
        else if (step == 15){
            notification = "launch_goalControl";
            launch_goalControl.start();
            if(is_goalControl == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }

        // -- parking control
        else if (step == 16){
            notification = "launch_parkingControl";
            launch_parkingControl.start();
            if(is_parkingControl == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }

        // -- client
        else if (step == 17){
            notification = "launch_StiClient";
            int sts_client = launch_client.start_and_wait(3.);
            if(is_client == 1 || sts_client == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }

        // -- control
        else if (step == 18){
            notification = "launch_StiControl";
            launch_control.start();
            if(is_control == 1){
                step += 1;
                usleep(timeWait*1000000);                    
            }
        }

        // -- posePublisher
        // -- completed
        else if (step == 19){
            notification = "Completed!";
        }

        // -- -- PUBLISH STATUS
        statusLaunch.persent = int((step/19)*100.);
        statusLaunch.position = step;
        statusLaunch.notification = notification;
        pub_statusLaunch.publish(statusLaunch);   
    }

};

int main (int argc, char **argv)
{
    cout << "Program start" << endl;
    ros::init(argc, argv, "Launch_nodepp");
    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle("~");
    ros::Rate rate(10); // ROS Rate at 5Hz

    Program self = Program(&nh, &private_node_handle);

    while (ros::ok()) {
        self.run();
        rate.sleep();
        ros::spinOnce(); 
    }
    return 0;
}

