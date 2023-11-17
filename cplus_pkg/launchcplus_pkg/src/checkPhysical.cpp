// Author : Phùng Quý Dương 
// Date: 15-9-2023
/*
    - Name node: CheckPort_cpp
    - Function: 
        + Check status connection of AGV Devices
*/

#include "ros/ros.h"
#include "ros/console.h"
#include "message_pkg/Status_port.h"
#include <bits/stdc++.h>

using namespace std;

class checkPhysical
{
    public:
    string port_nav350; 
    string port_main;      
    string port_oc;    
    string port_hc;     
    string port_imu;     
    string port_loadcell; 
    string port_driverAll;

    ros::Publisher pub_statusPort; 
    message_pkg::Status_port statusPort; 

    double pre_time;

    bool ethernet_check(string address){
        try{
            // print address
            // output = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=true);
            // // print(output)
            // result = str(output).find('time=');
            // // print(result) # yes : >0 
            // if (result != -1){
            //     return 1;
            // }
            // return 0;

            int status = system(("ping -c 2 " + address).c_str());  
            if (-1 != status) 
            { 
                int ping_ret = WEXITSTATUS(status); 

                if(ping_ret==0){
                    cout<<"Ping successful"<<endl; ////Proceed 
                    return 1;
                }
                else{
                    cout<<"Ping not successful"<<endl; ///Sleep and agin check for ping
                    return 0;
                }
            }

        }
        catch(...){
            return 0;
        }
    }

    // bool usbSerial_check_c3(string nameport){
    //     try{
    //         output = subprocess.check_output("ls -{} {} {} {} {}".format('l','/dev/','|','grep', nameport ), shell=true);
    //         vitri = output.find("stibase");

    //         name = output[(vitri):(vitri+len(nameport))];
    //         // print(name)
    //         if (name == nameport){
    //             return 1;
    //         }
    //         return 0;
    //     }
    //     catch(...){
    //         return 0;
    //     }
    // }

    bool usbSerial_check(string nameport){
        try{
            // output = subprocess.check_output("ls {} {} {} {}".format('/dev/','|','grep', nameport ), shell=true);
            int output = system(("ls /dev/ | grep " + nameport).c_str());
            if (output == 0){
                return 1;
            }
            else{
                return 0;
            }
        }
        catch(...){
            return 0;
        }
    }

    // bool usbCamera_check(string nameport){
    //     try{
    //         output = subprocess.check_output("rs-enumerate-devices -{}".format('s'), shell=true);
    //         // print(output);
    //         vitri = str(output).find(nameport[1:]);
    //         // name = output[(vitri):(vitri+len(nameport))];
    //         // print(result);
    //         // print(nameport[1:]);
    //         if vitri > 1 : return 1;
    //     }
    //     catch(...){
    //         // print("no port");
    //         return 0;
    //     }
    // }

};
int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "checkPhysical");   // specify node names
    ros::NodeHandle n;      //create a handle to this process's node
    ros::Rate loop_rate(1);   // 10Hz

    checkPhysical self;

    ros::param::get("port_nav350", self.port_nav350);
    ros::param::get("port_main", self.port_main);
    ros::param::get("port_oc", self.port_oc);
    ros::param::get("port_hc", self.port_hc);
    ros::param::get("port_imu", self.port_imu);
    ros::param::get("port_loadcell", self.port_loadcell);
    ros::param::get("port_driverAll", self.port_driverAll);

    self.pub_statusPort = n.advertise<message_pkg::Status_port>("/status_port", 50); 
    self.pre_time = ros::Time::now().toSec();

    while (ros::ok())
    {
        // -- LMS100
        self.pre_time = ros::Time::now().toSec();
        self.statusPort.lms100 = true;
        double t = ros::Time::now().toSec() - self.pre_time;
        // print ("t: ", t)
        // -- TiM551
        self.statusPort.tim551 = true;
        // -- NAV350
        if (self.ethernet_check(self.port_nav350) == 1){
            self.statusPort.nav350 = true;
        }
        else{
            self.statusPort.nav350 = false;
        }

        // -- D435
        self.statusPort.camera = true;
        
        // -- POWER ( main 82)        
        if (self.usbSerial_check(self.port_main) == 1){
            self.statusPort.main = true;
        }
        else{
            self.statusPort.main = false;
        }

        // -- MC 
        self.statusPort.mc = true;         

        // -- SC          
        self.statusPort.sc = true;

        // -- OC    
        self.statusPort.oc = true;      
        // if self.usbSerial_check(self.port_oc) == 1:
        //     self.statusPort.oc = true
        // else:
        //     self.statusPort.oc = false

        // -- HC          
        if (self.usbSerial_check(self.port_hc) == 1){
            self.statusPort.hc = true;
        }
        else{
            self.statusPort.hc = false;
        }

        // -- HMI          
        self.statusPort.hmi = true;

        // -- magLine          
        self.statusPort.magLine = true;

        // -- IMU
        if (self.usbSerial_check(self.port_imu) == 1){
            self.statusPort.imu = true;
        }
        else{
            self.statusPort.imu = false;
        }
        
        // -- loadcell
        self.statusPort.loadcell = true;
        // if self.usbSerial_check(self.port_loadcell) == 1:
        //     self.statusPort.loadcell = true
        // else:
        //     self.statusPort.loadcell = false

        // -- driverAll
        if (self.usbSerial_check(self.port_driverAll) == 1){
            self.statusPort.driverall = true;
        }
        else{
            self.statusPort.driverall = false;
        }

        self.statusPort.new1 = true;
        self.statusPort.new2 = true;
        
        self.pub_statusPort.publish(self.statusPort);  
        
        ros::spinOnce();     // allow receiving callbacks function
        loop_rate.sleep();   //ros pause in 10Hz
    }

    return 0;
}