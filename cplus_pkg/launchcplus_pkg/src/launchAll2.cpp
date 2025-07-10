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
    int process;
    time_t time_pre;
    // char buf[100];

    Launch(string file_launch){
        filelaunch = file_launch;
        // command = "roslaunch " + filelaunch;
        process = 0;
        time_pre = time(NULL);
        // std::cout << time_pre;
        
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


class Program{
            
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

    string notification;
    uint8_t count_node;
    uint8_t step;
    float timeWait;

    uint8_t is_firstWork;
    uint8_t is_checkPort;
    uint8_t is_reconnectBase;
    uint8_t is_reconnectDriver;
    uint8_t is_main;
    uint8_t is_driverLeft;
    uint8_t is_driverRight;
    uint8_t is_nav350;
    uint8_t is_hc;
    uint8_t is_oc;
    uint8_t is_imu;
    uint8_t is_loadcell;
    uint8_t is_imuFilter;
    uint8_t is_kinematic;
    uint8_t is_robotPoseNav;
    uint8_t is_ekf;
    uint8_t is_safetyNav350;
    uint8_t is_goalControl;
    uint8_t is_parkingControl;
    uint8_t is_client;
    uint8_t is_control;
    uint8_t is_posePublisher;

    Program(){
        count_node = 0;
        step = 0;
        timeWait = 0.4;   // s

    }

    void callBack_firstWork(const std_msgs::Int16::ConstPtr& msg){
        is_firstWork = 1;
        ROS_INFO("firstWork value = %d", is_firstWork);
    }

    void callBack_checkPort(const message_pkg::Status_port::ConstPtr& msg){
        is_checkPort = 1;
        ROS_INFO("checkPort value = %d", is_checkPort);
    }

    void callBack_reconnectBase(const message_pkg::Status_reconnect::ConstPtr& msg){
        is_reconnectBase = 1;
    }

    void callBack_reconnectDriver(const message_pkg::Status_reconnect::ConstPtr& msg){
        is_reconnectBase = 1;
    }

    void callBack_main(const sti_msgs::POWER_info::ConstPtr& msg){
        is_main = 1;
    }

    void callBack_driverLeft(const message_pkg::Driver_respond::ConstPtr& msg){
        is_driverLeft = 1;
    }

    void callBack_driverRight(const message_pkg::Driver_respond::ConstPtr& msg){
        is_driverRight = 1;
    }

    void callBack_nav350(const sensor_msgs::LaserScan::ConstPtr& msg){
        is_nav350 = 1;
    }

    void callBack_hc(const sti_msgs::HC_info::ConstPtr& msg){
        is_hc = 1;
    }

    void callBack_oc(const sti_msgs::Lift_status::ConstPtr& msg){
        is_oc = 1;
    }

    void callBack_imu(const sensor_msgs::Imu::ConstPtr& msg){
        is_imu = 1;
    }

    void callBack_loadcell(const message_pkg::Loadcell_respond::ConstPtr& msg){
        is_loadcell = 1;
    }

    void callBack_imuFilter(const sensor_msgs::Imu::ConstPtr& msg){
        is_imuFilter = 1;
    }

    void callBack_kinematic(const message_pkg::Driver_query::ConstPtr& msg){
        is_kinematic = 1;
    }

    void callBack_robotPoseNav(const geometry_msgs::PoseStamped::ConstPtr& msg){
        is_robotPoseNav = 1;
    }

    void callBack_ekf(const nav_msgs::Odometry::ConstPtr& msg){
        is_ekf = 1;
    }

    void callBack_safetyNav350(const std_msgs::Int8::ConstPtr& msg){
        is_safetyNav350 = 1;
    }

    void callBack_goalControl(const sti_msgs::Status_goal_control::ConstPtr& msg){
        is_goalControl = 1;
    }

    void callBack_parkingControl(const message_pkg::Parking_respond::ConstPtr& msg){
        is_parkingControl = 1;
    }

    void callBack_client(const sti_msgs::NN_infoRequest::ConstPtr& msg){
        is_client = 1;
    }

    void callBack_control(const sti_msgs::NN_infoRespond::ConstPtr& msg){
        is_control = 1;
    }

    void callBack_posePublisher(const geometry_msgs::Pose::ConstPtr& msg){
        is_posePublisher = 1;
    }

};

int main(int argc, char **argv){
    std::cout << "Program start!";

    Program program;
    message_pkg::Status_launch statusLaunch;

    ROS_INFO("ROS Initial!");
    ros::init(argc, argv, "Program_launch");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher pub_statusLaunch = n.advertise<message_pkg::Status_launch>("/status_launch", 1000);

    // -- module - firstWork.
    ros::param::get("/path_firstWork", program.path_firstWork);
    Launch launch_firstWork(program.path_firstWork);
    ros::Subscriber sub_firstWork = n.subscribe("/first_work/run", 1000, &Program::callBack_firstWork, &program);
    program.is_firstWork = 0;
    program.count_node += 1;

    // -- module - checkPort.
    ros::param::get("/path_checkPort", program.path_checkPort);
    std::cout << program.path_checkPort;

    Launch launch_checkPort(program.path_checkPort);
    ros::Subscriber sub_checkPort = n.subscribe("/status_port", 1000, &Program::callBack_checkPort, &program);
    program.is_checkPort = 0;
    program.count_node += 1;

    // -- module - reconnectBase.
    ros::param::get("/path_reconnectBase", program.path_reconnectBase);
    Launch launch_reconnectBase(program.path_reconnectBase);
    // launch_reconnectBase.start();
    ros::Subscriber sub_reconnectBase = n.subscribe("/status_reconnectBase", 1000, &Program::callBack_reconnectBase, &program);
    program.is_reconnectBase = 1;
    program.count_node += 1;

    // -- module - reconnectDriver.
    ros::param::get("/path_reconnectDriver", program.path_reconnectDriver);
    Launch launch_reconnectDriver(program.path_reconnectDriver);
    // launch_reconnectDriver.start();
    ros::Subscriber sub_reconnectDriver = n.subscribe("/status_reconnectDriver", 1000, &Program::callBack_reconnectDriver, &program);
    program.is_reconnectDriver = 1;
    program.count_node += 1;

    // -- module - Main.
    ros::param::get("/path_main", program.path_main);
    Launch launch_main(program.path_main);
    ros::Subscriber sub_main = n.subscribe("/POWER_info", 1000, &Program::callBack_main, &program);
    program.is_main = 0;
    program.count_node += 1;

    // -- module - driverAll.
    ros::param::get("/path_driverReconnect", program.path_driverReconnect);
    Launch launch_driverReconnect(program.path_driverReconnect);
    // ros::Subscriber sub_driverLeft = n.subscribe("/driver1_respond", 1000, &Program::callBack_driverLeft, &program);
    // ros::Subscriber sub_driverRight = n.subscribe("/driver2_respond", 1000, &Program::callBack_driverRight, &program);
    program.is_driverLeft = 0;
    program.is_driverRight = 0;
    program.count_node += 1;

    // -- module - nav350.
    ros::param::get("/path_nav350", program.path_nav350);
    Launch launch_nav350(program.path_nav350);
    // ros::Subscriber sub_nav350 = n.subscribe("/scan", 1000, &Program::callBack_nav350, &program);
    program.is_nav350 = 0;
    program.count_node += 1;

    // -- module - HC.
    ros::param::get("/path_hc", program.path_hc);
    Launch launch_hc(program.path_hc);
    // ros::Subscriber sub_hc = n.subscribe("/HC_info", 1000, &Program::callBack_hc, &program);
    program.is_hc = 0;
    program.count_node += 1;

    // -- module - OC.
    ros::param::get("/path_oc", program.path_oc);
    Launch launch_oc(program.path_oc);
    // ros::Subscriber sub_oc = n.subscribe("/lift_status", 1000, &Program::callBack_oc, &program);
    program.is_oc = 0;
    program.count_node += 1;

    // -- module - imu.
    ros::param::get("/path_imu", program.path_imu);
    Launch launch_imu(program.path_imu);
    // ros::Subscriber sub_imu = n.subscribe("/imu/data", 1000, &Program::callBack_imu, &program);
    program.is_imu = 0;
    program.count_node += 1;

    // -- module - loadcell
    ros::param::get("/path_loadcell", program.path_loadcell);
    Launch launch_loadcell(program.path_loadcell);
    // ros::Subscriber sub_loadcell = n.subscribe("/loadcell_respond", 1000, &Program::callBack_loadcell, &program);
    program.is_loadcell = 0;
    program.count_node += 1;

    // -- module - imuFilter.
    ros::param::get("/path_imuFilter", program.path_imuFilter);
    Launch launch_imuFilter(program.path_imuFilter);
    // ros::Subscriber sub_imuFilter = n.subscribe("/imu_filter", 1000, &Program::callBack_imuFilter, &program);
    program.is_imuFilter = 0;
    program.count_node += 1;

    // -- module - kinematic.
    ros::param::get("/path_kinematic", program.path_kinematic);
    Launch launch_kinematic(program.path_kinematic);
    // ros::Subscriber sub_kinematic = n.subscribe("/driver1_query", 1000, &Program::callBack_kinematic, &program);
    program.is_kinematic = 0;
    program.count_node += 1;

    // -- get pose robot from nav.
    ros::param::get("/path_robotPoseNav", program.path_robotPoseNav);
    Launch launch_robotPoseNav(program.path_robotPoseNav);
    // ros::Subscriber sub_robotPoseNav = n.subscribe("/robotPose_nav", 1000, &Program::callBack_robotPoseNav, &program);
    program.is_robotPoseNav = 0;
    program.count_node += 1;

    // -- module - ekf.
    ros::param::get("/path_ekf", program.path_ekf);
    Launch launch_ekf(program.path_ekf);
    // ros::Subscriber sub_ekf = n.subscribe("/odometry", 1000, &Program::callBack_ekf, &program);
    program.is_ekf = 0;
    program.count_node += 1;

    // -- module - safety zone Nav350.
    ros::param::get("/path_safetyNav350", program.path_safetyNav350);
    Launch launch_safetyNav350(program.path_safetyNav350);
    // ros::Subscriber sub_safetyNav350 = n.subscribe("/safety_NAV", 1000, &Program::callBack_safetyNav350, &program);
    program.is_safetyNav350 = 0;
    program.count_node += 1;

    // -- module - goalControl.
    ros::param::get("/path_goalControl", program.path_goalControl);
    Launch launch_goalControl(program.path_goalControl);
    // ros::Subscriber sub_goalControl = n.subscribe("/status_goal_control", 1000, &Program::callBack_goalControl, &program);
    program.is_goalControl = 0;
    program.count_node += 1;

    // -- module - parkingControl.
    ros::param::get("/path_parkingControl", program.path_parkingControl);
    Launch launch_parkingControl(program.path_parkingControl);
    // ros::Subscriber sub_parkingControl = n.subscribe("/parking_respond", 1000, &Program::callBack_parkingControl, &program);
    program.is_parkingControl = 0;
    program.count_node += 1;

    // -- module - stiClient.
    ros::param::get("/path_client", program.path_client);
    Launch launch_client(program.path_client);
    // ros::Subscriber sub_client = n.subscribe("/NN_infoRequest", 1000, &Program::callBack_client, &program);
    program.is_client = 0;
    program.count_node += 1;

    // -- module - stiControl.
    ros::param::get("/path_control", program.path_control);
    Launch launch_control(program.path_control);
    // ros::Subscriber sub_control = n.subscribe("/NN_infoRespond", 1000, &Program::callBack_control, &program);
    program.is_control = 0;
    program.count_node += 1;
    
    // -- module - data App.
    ros::param::get("/path_dataApp", program.path_dataApp);
    Launch launch_dataApp(program.path_dataApp);
    program.count_node += 1;

    // -- module - posePublisher.
    ros::param::get("/path_posePublisher", program.path_posePublisher);
    Launch launch_posePublisher(program.path_posePublisher);
    // ros::Subscriber sub_posePublisher = n.subscribe("/robot_pose", 1000, &Program::callBack_posePublisher, &program);
    program.is_posePublisher = 0;
    program.count_node += 1;

    // -- ko dc xoa
    program.count_node += 1;

    while (ros::ok()){
        // -- firstWork
        if (program.step == 0){
            program.notification = "launch_firstWork";
            launch_firstWork.start();
            if (program.is_firstWork == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);
            }
        }
        // -- checkPort
        else if (program.step == 1){
            program.notification = "launch_checkPort";
            launch_checkPort.start();
            if (program.is_checkPort == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);
            }
        }

        // -- reconnectBase
        else if (program.step == 2){
            program.notification = "launch_reconnectBase";
            launch_reconnectBase.start();
            if (program.is_reconnectBase == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);
            }
        }

        // -- reconnectDriver
        else if (program.step == 3){
            program.notification = "launch_reconnectDriver";
            program.step += 1;
            launch_reconnectDriver.start();
            if (program.is_reconnectDriver == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);
            }
        }

        // -- data APP
        else if (program.step == 4){
            program.notification = "launch_dataApp";
            program.step += 1;
        }

        // -- main
        else if (program.step == 5){
            program.notification = "launch_main";
            launch_main.start();
            if (program.is_main == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);
            }
        }

        // -- hc
        else if (program.step == 6){
            program.notification = "launch_hc";
            program.step = 7;
        }
        // -- oc
        // -- imu
        else if (program.step == 7){
            program.notification = "launch_imu";
            program.step = 8;
        }

        // -- driverAll
        else if (program.step == 8){
            program.notification = "launch_driverReconnect";
            int sts_driver = launch_driverReconnect.start_and_wait(3.);
            if(sts_driver == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }
        
        // -- nav350
        else if (program.step == 9){
            program.notification = "launch_nav350";
            launch_nav350.start();
            if(program.is_nav350 == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }

        // -- data APP
        else if (program.step == 10){
            program.notification = "launch_dataApp";
            try{
                int sts_app = launch_dataApp.start_and_wait(2.);
                if(sts_app == 1){
                    program.step += 1;
                    usleep(program.timeWait*1000000); 
                }
            }
            catch(...){
                program.step += 1;
            }
        }

        // -- kinematic
        else if (program.step == 11){
            program.notification = "launch_kinematic";
            launch_kinematic.start();
            if(program.is_kinematic == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }

        // -- get pose robot from nav.
        else if (program.step == 12){
            program.notification = "launch_robotPoseNav";
            launch_robotPoseNav.start();
            if(program.is_robotPoseNav == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }

        // -- ekf
        else if (program.step == 13){
            program.notification = "launch_ekf";
            int sts_ekf = launch_ekf.start_and_wait(3.);
            if(program.is_ekf == 1 || sts_ekf == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }

        // -- safety zone Nav350
        else if (program.step == 14){
            program.notification = "launch_safetyNav350";
            launch_safetyNav350.start();
            if(program.is_safetyNav350 == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }

        // -- goal control
        else if (program.step == 15){
            program.notification = "launch_goalControl";
            launch_goalControl.start();
            if(program.is_goalControl == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }

        // -- parking control
        else if (program.step == 16){
            program.notification = "launch_parkingControl";
            launch_parkingControl.start();
            if(program.is_parkingControl == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }

        // -- client
        else if (program.step == 17){
            program.notification = "launch_StiClient";
            int sts_client = launch_client.start_and_wait(3.);
            if(program.is_client == 1 || sts_client == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }

        // -- control
        else if (program.step == 18){
            program.notification = "launch_StiControl";
            launch_control.start();
            if(program.is_control == 1){
                program.step += 1;
                usleep(program.timeWait*1000000);                    
            }
        }

        // -- posePublisher
        // -- completed
        else if (program.step == 19){
            program.notification = "Completed!";
        }

        // -- -- PUBLISH STATUS
        statusLaunch.persent = int((program.step/19)*100.);
        statusLaunch.position = program.step;
        statusLaunch.notification = program.notification;
        pub_statusLaunch.publish(statusLaunch);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // }
    // catch(...){

    // }
    std::cout << "Program stop!";
    return 0;
}

