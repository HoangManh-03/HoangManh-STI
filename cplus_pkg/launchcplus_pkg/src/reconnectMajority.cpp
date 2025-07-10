// Author : Phùng Quý Dương 
// Date: 7/10/2023
/*
    - Name node: reconnectBase_nodecpp 
    - Function: 
        + Kết nối lại nếu node bị die

    """
    yêu cầu thông tin để kết nối lại 1 node:
    1, Topic để kiểm tra node đó có đang hoạt động không.
    2, Tên node để shutdown node đó.
    3, File launch khởi tạo node.
    ----------------
    Nếu cổng vật lý vẫn còn.
    """
*/

#include "ros/ros.h"
#include "ros/console.h"

#include "message_pkg/Status_port.h"
#include "message_pkg/Parking_respond.h"
#include "sti_msgs/POWER_info.h"
#include "sti_msgs/Nav350_data.h"

#include <bits/stdc++.h>
#include <unistd.h>
#include <Python.h>

using namespace std;

class Reconnect
{
    public:
    double time_checkLost;
    double time_waitLaunch;
    string fileLaunch;
    string nameTopic_sub;

    string nameNode = "";
    bool is_nameReaded = 0;
    // -- launch
    // uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    // roslaunch.configure_logging(uuid)
    // -- variable
    ros::Time lastTime_waitConnect = ros::Time::now();
    ros::Time time_readed = ros::Time::now(); 
    uint8_t enable_check = 0;
    int process = 0; 
    int numberReconnect = 0; 
    // -- 
    ros::Time lastTime_waitShutdown = ros::Time::now();

    Reconnect(string nameTopicsub, double timecheckLost, double timewaitLaunch, string file_launch){
		time_checkLost = timecheckLost;
		time_waitLaunch = timewaitLaunch; // wait after launch;
		fileLaunch = file_launch;
		nameTopic_sub = nameTopicsub;
    }

	string read_nameNode(string topic){
		try{
			output = subprocess.check_output("rostopic info {}".format(topic), shell = True);
			// print("out: ", output)

			pos1 = str(output).find("Publishers:"); // tuyet doi ko sua linh tinh.
			pos2 = str(output).find(" (http://");   // tuyet doi ko sua linh tinh.
			// print("pos2: ", pos2)
			if (pos1 >= 0){
				name = str(output)[pos1 + 16 :pos2];
				p1 = name.find("Subscribers:");
				p2 = name.find("*");
				if (p1 == -1 and p2 == -1){
					return name;
                }
				else{
                    cout << "p1:" << p1 << endl;
                    cout << "p2:" << p2 << endl;
					// printf("Read name error: " + name + "|" + nameTopic_sub)
                    cout << "Read name error: " << name << "|" << nameTopic_sub << endl;
					return "";
                }
            }
			else{
				// print ("Not!");
                cout << "Not!" << endl;
				return "";
            }
        }

		catch(...){
			// print ("Error!")
			return "";
        }
    }   

	tuple<int, int> run_reconnect_vs2(int enb_check, double timereaded){   // have kill node via name_topic pub.
		enable_check = enb_check;
		time_readed = timereaded;
		// -- read name node.
		if (enb_check == 1 && is_nameReaded == 0){
			string nameNode = read_nameNode(nameTopic_sub);
			if (len(nameNode) != 0):
				is_nameReaded = 1;
        }
		// -- 	
		launch = roslaunch.parent.ROSLaunchParent(uuid , [fileLaunch]);          // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,

		if (process == 0){ // - Check lost.
			if (enable_check){
				double t = ros::Time::now().toSec() - time_readed;
				if (t >= time_checkLost){
					cout << "Detected Lost: " << str(nameNode) << endl;
					process = 1;
                }
            }
        }

		else if (process == 1){ // - shutdown node
			launch.shutdown();                                         // <<<<<<<<<<<<<<
			cout << "shutdown node: " << str(nameNode) << endl;

			if (is_nameReaded){
				// nameNode = "/lineMagnetic"
				try{
					os.system("rosnode kill " + nameNode);
					cout << "rosnode kill "  << nameNode << endl;
                }
				catch(...){
					cout <<"ERROR rosnode kill /"  <<  nameNode << endl;
                }
            }

			lastTime_waitShutdown = ros::Time::now().toSec();
			cout<<"Wait after shutdown node: " << nameNode << endl;
			process = 2;
        }

		else if(process == 2){ // - wait after shutdown.
			double t = ros::Time::now().toSec() - lastTime_waitShutdown;
			if (t >= 3){
				process = 3;
            }
        }

		else if(process == 3){ // - Launch
			cout << "Launch node: " << nameNode << endl;
			launch.start();                            // <<<<<<<<<<<<<<<<<<<<<<<<<<<<
			numberReconnect += 1;
			lastTime_waitConnect = ros::Time::now().toSec();
			process = 4;
        }

		else if(process == 4){ // - wait after launch
			double t1 = ros::Time::now().toSec() - lastTime_waitConnect;
			double t2 = ros::Time::now().toSec() - time_readed; // have topic pub
			if (t1 >= time_waitLaunch || t2 <= 1){
				// print ("t1: ", t1)
				// print ("t2: ", t2)
				cout << "Launch node completed: " << nameNode << endl;
				process = 0;
				is_nameReaded = 0;
				nameNode = "";
            }
        }

		return make_tuple(process, numberReconnect);
    }

	tuple<int,int> run_reconnect(int enb_check, double time_readed){ 
		enable_check = enb_check;
		time_readed = time_readed;

		launch = roslaunch.parent.ROSLaunchParent(uuid , [fileLaunch]);
		if (process == 0){ // - Check lost.
			if (enable_check){
				double t = ros::Time::now().toSec() - time_readed;
				if (t >= time_checkLost){
					cout << "Detected Lost: " << str(nameNode) << endl;
					process = 1;
                }
            }
        }

		else if (process == 1){ // - shutdown node
			launch.shutdown();
			cout << "shutdown node: " << str(nameNode) << endl;
			lastTime_waitShutdown = ros::Time::now().toSec();

			cout << "Wait after shutdown node: " <<  nameNode << endl;
			process = 2;
        }

		else if (process == 2){ // - wait after shutdown.
			double t = time.time() - lastTime_waitShutdown;
			if (t >= 3){
				process = 3;
            }
        }

		else if (process == 3){ // - Launch
			cout << "Launch node: " << nameNode << endl;
			launch.start();
			numberReconnect += 1;
			lastTime_waitConnect = ros::Time::now().toSec();
			process = 4;
        }

		else if (process == 4){ // - wait after launch
			double t1 = ros::Time::now().toSec() - lastTime_waitConnect;
			double t2 = ros::Time::now().toSec() - time_readed; // have topic pub
			if (t1 >= time_waitLaunch || t2 <= 1){
				// print ("t1: ", t1)
				// print ("t2: ", t2)
				cout << "Launch node completed: " <<  nameNode;
				process = 0;
            }
        }
				
		return make_tuple(process, numberReconnect);
    }
        
};

class Reconnect_node
{
    public:
    // -- Port
    ros::Subscriber sub_statusPort; 
    message_pkg::Status_port port_status;

    ros::Publisher pub_statusReconnect;
    message_pkg::Status_reconnect statusReconnect;
    double pre_timePub;
    float cycle_timePub;

    // Main
    string path_main;
    string timeLost_main;
    string timeWait_main;
    string topicSub_main;
    bool isRuned_main;
    double timeReaded_main;
    ros::Subscriber sub_main;

    // nav350
    string path_nav350;
    string timeLost_nav350;
    string timeWait_nav350;
    string topicSub_nav350;
    bool isRuned_nav350;
    double timeReaded_nav350;
    ros::Subscriber sub_nav350;

    // Parking
    string path_parking;
    string timeLost_parking;
    string timeWait_parking;
    string topicSub_parking;	
    bool isRuned_parking;
    double timeReaded_parking;
    ros::Subscriber sub_parking;

	void callback_port(const message_pkg::Status_port data){
		port_status = data;
    }

	void callback_main(const sti_msgs::POWER_info data){
		isRuned_main = 1;
	    timeReaded_main = ros::Time::now().toSec();
    }

	void callback_nav350(const  data){
		isRuned_nav350 = 1;
		timeReaded_nav350 = ros::Time::now().toSec();
    }
		
	void callback_parking(self, data){
		isRuned_parking = 1;
		timeReaded_parking = ros::Time::now().toSec();
    }

	// void reconnect_node(){
	// 	// -- main
    //     int process, num;
	// 	tie(process, num) = reconnect_main.run_reconnect_vs2(isRuned_main, timeReaded_main);
	// 	statusReconnect.main.sts = process;
	// 	statusReconnect.main.times = num;
	// 	// -- nav350
	// 	process, num = reconnect_nav350.run_reconnect_vs2(isRuned_nav350, timeReaded_nav350);
	// 	statusReconnect.lidar.sts = process;
	// 	statusReconnect.lidar.times = num;
	// 	// -- parking
	// 	process, num = reconnect_parking.run_reconnect(isRuned_parking, timeReaded_parking);
	// 	statusReconnect.parking.sts = process;
	// 	statusReconnect.parking.times = num;

	// 	// -- -- -- 
	// 	double tim1 = (time.time() - pre_timePub);
    //     double tim = tim1 - tim1/60;
	// 	if (tim > cycle_timePub){
	// 		pre_timePub = ros::Time::now().toSec();
	// 		pub_statusReconnect.publish(statusReconnect);
    //     }
    // }

};

int main(int argc, char **argv)
{
    std::cout << "Program start!";

    ros::init(argc, argv, "reconnectBase_nodecpp");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    Reconnect_node self;

    // -- Port
    self.sub_statusPort = n.subscribe("/status_port", 50, &Reconnect_node::callback_port, &self);

    self.pub_statusReconnect = n.advertise<message_pkg::Status_reconnect>("/status_reconnectDriverAll", 10);
    self.pre_timePub = ros::Time::now().toSec();
    self.cycle_timePub = 0.1; // s

    // main
    ros::param::get("path_main", self.path_main);
    ros::param::get("timeLost_main", self.timeLost_main);
    ros::param::get("timeWait_main", self.timeWait_main);
    self.topicSub_main = "/POWER_info"; 
    self.isRuned_main = 0    
    self.timeReaded_main = ros::Time::now().toSec();
    self.sub_main = n.subscribe(self.topicSub_main, 50, &Reconnect_node::callback_main, &self);
    Reconnect reconnect_main(self.topicSub_main, self.timeLost_main, self.timeWait_main, self.path_main);

    // nav350
    ros::param::get("path_nav350", self.path_nav350);
    ros::param::get("timeLost_nav350", self.timeLost_nav350);
    ros::param::get("timeWait_nav350", self.timeWait_nav350);
    self.topicSub_nav350 = "/nav350_data"; 
    self.isRuned_nav350 = 0    
    self.timeReaded_nav350 = ros::Time::now().toSec();
    self.sub_nav350 = n.subscribe(self.topicSub_nav350, 50, &Reconnect_node::callback_nav350, &self);
    Reconnect reconnect_nav350(self.topicSub_nav350, self.timeLost_nav350, self.timeWait_nav350, self.path_nav350);

    ros::param::get("path_parkingControl", self.path_parking);
    ros::param::get("timeLost_parking", self.timeLost_parking);
    ros::param::get("timeWait_parking", self.timeWait_parking);
    self.topicSub_parking = "/parking_respond"; 
    self.isRuned_parking = 0    
    self.timeReaded_parking = ros::Time::now().toSec();
    self.sub_parking = n.subscribe(self.topicSub_parking, 50, &Reconnect_node::callback_parking, &self);
    Reconnect reconnect_parking(self.topicSub_parking, self.timeLost_parking, self.timeWait_parking, self.path_parking);
    
    while(ros::ok()){
		// -- main
        int process, num;
		tie(process, num) = reconnect_main.run_reconnect_vs2(self.isRuned_main, self.timeReaded_main);
		self.statusReconnect.main.sts = process;
		self.statusReconnect.main.times = num;
		// -- nav350
		process, num = reconnect_nav350.run_reconnect_vs2(self.isRuned_nav350, self.imeReaded_nav350);
		self.statusReconnect.lidar.sts = process;
		self.statusReconnect.lidar.times = num;
		// -- parking
		process, num = reconnect_parking.run_reconnect(self.isRuned_parking, self.timeReaded_parking);
		self.statusReconnect.parking.sts = process;
		self.statusReconnect.parking.times = num;

		// -- -- -- 
		double tim1 = (time.time() - self.pre_timePub);
        double tim = tim1 - tim1/60;
		if (tim > self.cycle_timePub){
			self.pre_timePub = ros::Time::now().toSec();
			self.pub_statusReconnect.publish(self.statusReconnect);
        }

        ros::spinOnce();     // allow receiving callbacks function
        loop_rate.sleep();
    }

    return 0;
}