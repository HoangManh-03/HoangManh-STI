// blv 200W.
// DATE: 01/11/2023
// AUTHOR: Archie Phung

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <sstream>

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <string>
#include <modbuspp.h>
#include <ctime>
#include <math.h>

#include <message_pkg/Driver_query.h>
#include <message_pkg/Driver_respond.h>
#include <bits/stdc++.h>
#include <chrono>
#include <sys/time.h>
using namespace std;
using namespace Modbus;

class statusDriver
{
    public:
        int ID = 0;
		int FWD = 0;
		int REV = 0;
		int STOP_MODE = 0;
		int WNG = 0;
		int ALARM_OUT1 = 0;
		int S_BSY = 0;
		int ALARM_OUT2 = 0;	
		int MOVE = 0;
		int VA = 0;
		int TLC = 0;
		int PRESENT_ALARM = 0;
		int PRESENT_WARNING = 0;
		int SPEED = 0;

        statusDriver(int id = 0){
            this->ID = id;
        }
        statusDriver(){};
        ~statusDriver(){};
};

class controlDriver
{
    public:
        int ID = 0;
        int FWD = 0;
        int REV = 0;
        int STOP_MODE = 0;
        int SPEED = 0;
        int ACCELERATION_TIME = 1;
        int DECELERATION_TIME = 1;
        int TORQUE_LIMITING = 200;
        int REVERT = 0; // Đảo ngược quy định chiều động cơ.
        controlDriver(int id = 0, int rev = 0){
            this->ID = id;
            this->REVERT = rev;
        }
        controlDriver(){};
        ~controlDriver(){};
};

struct REG_Driver
{
    int reg_operationData = 89; // (0059h) - [0 to 255] P66
    int reg_operationType = 91; //
    int reg_operatingVelocity = 95; //
    int reg_accelerationRate = 97;  //
    int reg_decelerationRate = 99;  //
    int reg_torqueLimiting = 101;   // 0 to 10,000 (1=0.1%) - P67
    int reg_operationTrigger = 103; //
    // -- 
    int reg_inputCommandUpper = 124; //
    int reg_inputCommandLower = 125; //

    int reg_outputStatusUpper = 126; //
    int reg_outputStatusLower = 127; //
    // -- 
    int reg_presentAlarm = 129;  // - P304
    int reg_feedbackSpeed = 207; // - P305
    
    int reg_resentCommunicationError = 173; // P304
    int reg_driverTemperature = 249; // P306
    
    int reg_supplyVoltage = 329; // (1=0.1 V) P308
    int reg_resetAlarm = 385; // P302
    int reg_stopOperation = 447; // 
    int reg_resetCommunication = 445; //
    // - 
    int reg_operationDataNo0_type = 6145; // P326
    int reg_operationDataNo0_position = 6147; //
    int reg_operationDataNo0_velocity = 6149; // P326
    int reg_operationDataNo0_accelerationRate = 6151; // P326
    int reg_operationDataNo0_decelerationRate = 6153; // P326
    int reg_operationDataNo0_torqueLimiting = 6155; //
    int reg_operationDataNo0_accelerationTime = 6157; //
    int reg_operationDataNo0_decelerationTime = 6159; //
    // -
    int reg_operationDataNo2_type = 6273; // P326
    int reg_operationDataNo2_position = 6275; //
    int reg_operationDataNo2_velocity = 6277; // P326
    int reg_operationDataNo2_accelerationRate = 6279; // P326
    int reg_operationDataNo2_decelerationRate = 6281; // P326
    int reg_operationDataNo2_torqueLimiting = 6283; //
    int reg_operationDataNo2_accelerationTime = 6285; //
    int reg_operationDataNo2_decelerationTime = 6287; //
    // -- 
    int reg_driverOutputCommand_upper = 126; // 0x7F - pape 225
    int reg_driverOutputCommand_lower = 127; // 0x7F - pape 225
};

struct BitControl_Driver
{
    uint8_t bitInputCommand_FW_JOG = 0;
    uint8_t bitInputCommand_FV_JOG = 1;
    uint8_t bitInputCommand_FW_SPD = 2;
    uint8_t bitInputCommand_FV_SPD = 3;
    uint8_t bitInputCommand_HOME = 4;
    uint8_t bitInputCommand_NOT5 = 5;
    uint8_t bitInputCommand_START = 6;
    uint8_t bitInputCommand_SSTART = 7;
    uint8_t bitInputCommand_M0 = 8;
    uint8_t bitInputCommand_M1 = 9;
    uint8_t bitInputCommand_M2 = 10;
    uint8_t bitInputCommand_M3 = 11;
    uint8_t bitInputCommand_M4 = 12;
    uint8_t bitInputCommand_M5 = 13;
    uint8_t bitInputCommand_M6 = 14;
    uint8_t bitInputCommand_M7 = 15;
    // -- 
    uint8_t bitInputCommand_S_ON = 0;
    uint8_t bitInputCommand_PLOOP_MODE = 1;
    uint8_t bitInputCommand_TRQ_LMT = 2;
    uint8_t bitInputCommand_CLR   = 3;
    uint8_t bitInputCommand_QSTOP = 4;
    uint8_t bitInputCommand_STOP  = 5;
    uint8_t bitInputCommand_FREE  = 6;
    uint8_t bitInputCommand_ALM_RST = 7;
    uint8_t bitInputCommand_D_SEL0  = 8;
    uint8_t bitInputCommand_D_SEL1  = 9;
    uint8_t bitInputCommand_D_SEL2  = 10;
    uint8_t bitInputCommand_D_SEL3  = 11;
    uint8_t bitInputCommand_D_SEL4  = 12;
    uint8_t bitInputCommand_D_SEL5  = 13;
    uint8_t bitInputCommand_D_SEL6  = 14;
    uint8_t bitInputCommand_D_SEL7  = 15;
    // --
    uint8_t bitOutputStatus_INFO     = 0;
    uint8_t bitOutputStatus_INFO_MNT = 1;
    uint8_t bitOutputStatus_DRVTMP   = 2;
    uint8_t bitOutputStatus_MTRTMP   = 3;
    uint8_t bitOutputStatus_INFO_TRQ = 4;
    uint8_t bitOutputStatus_INFO_WATT   = 5;
    uint8_t bitOutputStatus_INFO_VOLTH  = 6;
    uint8_t bitOutputStatus_INFO_VOLTL  = 7;
    uint8_t bitOutputStatus_CONST_OFF24 = 8;
    uint8_t bitOutputStatus_CONST_OFF25 = 9;
    uint8_t bitOutputStatus_CONST_OFF26 = 10;
    uint8_t bitOutputStatus_CONST_OFF27 = 11;
    uint8_t bitOutputStatus_CONST_OFF28 = 12;
    uint8_t bitOutputStatus_CONST_OFF29 = 13;
    uint8_t bitOutputStatus_USR_OUT0    = 14;
    uint8_t bitOutputStatus_USR_OUT1    = 15;
    // --
    uint8_t bitOutputStatus_SON_MON   = 0;
    uint8_t bitOutputStatus_PLOOP_MON = 1;
    uint8_t bitOutputStatus_TRQ_LMTD  = 2;
    uint8_t bitOutputStatus_RDY_DD = 3;
    uint8_t bitOutputStatus_ABSPEN = 4;
    uint8_t bitOutputStatus_STOP_R = 5;
    uint8_t bitOutputStatus_FREE_R = 6;
    uint8_t bitOutputStatus_ALM_A  = 7;
    uint8_t bitOutputStatus_SYS_BSY  = 8;
    uint8_t bitOutputStatus_IN_POS   = 9;
    uint8_t bitOutputStatus_RDY_HOME = 10;
    uint8_t bitOutputStatus_RDY_FWRV = 11;
    uint8_t bitOutputStatus_RDY_SD   = 12;
    uint8_t bitOutputStatus_MOVE = 13;
    uint8_t bitOutputStatus_VA   = 14;
    uint8_t bitOutputStatus_TLC  = 15;
};

class driver
{
public:
    string PORT = "/dev/ttyUSB0";
    string BAUDRATE = "57600";
    int ID_driver1 = 1;
    int ID_driver2 = 2; 
    int maxRPM = 3800;
    int minRPM = maxRPM*(-1);
    int revert_1 = 0;
    int revert_2 = 1;

    ros::Time lastTime_readStatus = ros::Time::now();
    double time_readStatus = 2;

    int accelerationRate = 1000; // [1 to 1,000,000] ms
    int decelerationRate = 200; // [1 to 1,000,000] ms
    int torqueLimiting = 200; // 200    # 0 to 10,000 (1=0.1%)

    int reg_communicationTimeout = 5003; // P222
    int reg_CommunicationErrorDetection = 5005; 


    REG_Driver RegDriver;
    BitControl_Driver BitControlDriver;

    statusDriver statusDriver_1 = statusDriver(ID_driver1);
    controlDriver controlDriver_1 = controlDriver(ID_driver1, revert_1);

    statusDriver statusDriver_2 = statusDriver(ID_driver2);
    controlDriver controlDriver_2 = controlDriver(ID_driver2, revert_2);

    uint8_t enable_run = 0;
    uint8_t is_readSafety = 0;
    ros::Time lastTime_checkSafety = ros::Time::now();
    uint8_t cycle_checkSafety = 1; //s

    // sub object
    ros::Subscriber sub_disable_brake;
    std_msgs::Bool disable_brake;

    ros::Subscriber sub_driver1_query;
    message_pkg::Driver_query driverQuery_1;
    uint8_t is_readQuery_1 = 0;
    ros::Time lastTime_checkQuery_1 = ros::Time::now();
    float cycle_checkQuery = 0.5; // s

    ros::Subscriber sub_driver2_query;
    message_pkg::Driver_query driverQuery_2;
    uint8_t is_readQuery_2 = 0;
    ros::Time lastTime_checkQuery_2 = ros::Time::now();
    float cycle_checkQuery_2 = 0.5; // s

    // pub object
    ros::Publisher pub_driverRespond_1;
    message_pkg::Driver_respond driverRespond_1;

    ros::Publisher pub_driverRespond_2;
    message_pkg::Driver_respond driverRespond_2;

    uint8_t status_error = 2;
    uint8_t status_run = 1;
    uint8_t status_stop = 0;
    // -- 
    int32_t timeWait = 400;
    // int32_t timeWait = 1000000;
    // -- 
    uint8_t frequence_pubStatus = 25.;
    float cycle_pubStatus = 1/frequence_pubStatus;
    ros::Time preTime_pubStatus = ros::Time::now();
    // --
    uint8_t countErr_reqSpeed = 0;

    int value_runing = 0;

    time_t time_exec;
    int mt_step_run = 0;

// public:
    driver(ros::NodeHandle *nh, ros::NodeHandle *npr){
        // info pub
        pub_driverRespond_1 = nh->advertise<message_pkg::Driver_respond>("/driver1_respond", 50);
        pub_driverRespond_2 = nh->advertise<message_pkg::Driver_respond>("/driver2_respond", 50);

        // info sub
        sub_disable_brake = nh->subscribe("/disable_brake", 10, &driver::break_infoCallback, this);
        sub_driver1_query = nh->subscribe("/driver1_query", 50, &driver::driverQuery1_infoCallback, this);
        sub_driver2_query = nh->subscribe("/driver2_query", 50, &driver::driverQuery2_infoCallback, this);

    }
    ~driver(){};

    void break_infoCallback(const std_msgs::Bool& data){
        disable_brake = data;
    }

    void driverQuery1_infoCallback(const message_pkg::Driver_query& data){
		driverQuery_1 = data;
		is_readQuery_1 = 1;
		lastTime_checkQuery_1 = ros::Time::now();
    }

    void driverQuery2_infoCallback(const message_pkg::Driver_query& data){
		driverQuery_2 = data;
		is_readQuery_2 = 1;
		lastTime_checkQuery_2 = ros::Time::now();
    }

    string mean_ALM_A(int err){
        if(err == 0) return ("All right!");
        else if(err == 16) return ("Position deviation (300 rev) | L7");
        else if(err == 33) return ("Main circuit overheat > 85°C | L7");
        else if(err == 34) return ("Overvoltage > 63 V | L5");
        else if(err == 37) return ("Undervoltage < 14 V | L5");
        else if(err == 38) return ("Motor overheat 95°C | L7");
        else if(err == 49) return ("Overspeed | L7");

        else if(err == 32) return ("Overcurrent | L9");
        else if(err == 40) return ("Encoder error | L2");
        else if(err == 41) return ("Internal circuit error: CPU peripheral | L9");
        else if(err == 42) return ("Encoder communication error | L2");
        else if(err == 48) return ("Overload | L7");
        else if(err == 65) return ("EEPROM error | L9");
        else if(err == 66) return ("Initial encoder error L2");
        else if(err == 68) return ("Encoder EEPROM error");
        else if(err == 69) return ("Motor combination error");
        else if(err == 74) return ("Homing incomplete");
        else if(err == 80) return ("Electromagnetic brake overcurrent | L9");
        else if(err == 83) return ("HWTO input circuit error");
        else if(err == 85) return ("The electromagnetic brake connection error");
        else if(err == 96) return ("±LS both sides active");
        else if(err == 97) return ("Reverse ±LS connection");
        else if(err == 98) return ("Homing operation error");
        else if(err == 99) return ("No HOMES");
        else if(err == 100) return ("Z, SLIT signal error");
        else if(err == 102) return ("Hardware overtravel");
        else if(err == 103) return ("Software overtravel");
        else if(err == 104) return ("HWTO input detection");
        else if(err == 106) return ("Homing additional operation error");
        else if(err == 112) return ("Operation data error");
        else if(err == 113) return ("Unit setting error");
        else if(err == 129) return ("Network bus error");
        else if(err == 132) return ("RS-485 communication error");
        else if(err == 133) return ("RS-485 communication timeout");
        else if(err == 140) return ("Out of setting range");
        else if(err == 240) return ("CPU error");
        else if(err == 243) return ("CPU overload");
        return ("UNK");
    }

    int readHoldingRegister(Slave &drive, Master &mb, int add){
        int outValue = -1;
        if (mb.open()) { // open a connection
            Data<int32_t> registers;
            // reads values ....
            if (drive.readRegister(add, registers) > 0) {
            // then print them !
                outValue = registers;
            }
            else {
                cerr << "Unable to read input registers ! "  << mb.lastError() << endl;
            }
            mb.close();
        }
        else {
            cerr << "Unable to open MODBUS connection, because " << mb.lastError() << endl;
        }
        return outValue;
    }

    int writeHoldingRegister(Slave &drive, Master &mb, int add, int value){
        int outValue = -1;
        if (mb.open()){
            int ret;
            Data<int32_t> registers (value);
            // then writing to registers
            ret = drive.writeRegister (add, registers);
            if (ret < 0){
                cerr << "Unable to write input registers ! "  << mb.lastError() << endl;
                // exit (EXIT_FAILURE);
            }
            else{
                outValue = ret;
                // cout << ret << " registers written (16-bit).   time: " << buf << endl;
            }
            mb.close();

        }else{
            cerr << "Unable to open MODBUS connection, because " << mb.lastError() << endl;
            // exit (EXIT_FAILURE);
        }
        return outValue;
    }

    void resetAlarm(Slave &drive, Master &mb){
        writeHoldingRegister(drive, mb, RegDriver.reg_resetAlarm, 1);
        usleep(timeWait);
        writeHoldingRegister(drive, mb, RegDriver.reg_resetAlarm, 0); 
        // usleep(timeWait);
    }

    void reset_drive(Slave &drive, Master &mb, statusDriver& sttDriver){
        cout << ("Run reset") << endl;
		if (sttDriver.WNG == 1 || sttDriver.ALARM_OUT1 == 1 || sttDriver.ALARM_OUT2 == 1 || sttDriver.PRESENT_ALARM != 0){
            resetAlarm(drive, mb);
        }
    }

    void getStatus_drive(Slave &drive, Master &mb, int type, statusDriver& sttDriver){
        ros::Duration t = ros::Time::now() - lastTime_readStatus;
        double t1 = t.toSec();
        int a = t1/60;
        double t2 = t1 - a*60;

        if(t2 > time_readStatus || type == 1){
            lastTime_readStatus = ros::Time::now();
            sttDriver.PRESENT_ALARM = readHoldingRegister(drive, mb, RegDriver.reg_presentAlarm);
        }
    }

    void config_drive(Slave &drive, Master &mb){
        // time out
        cout << "IM here config drive" << endl;
        writeHoldingRegister(drive, mb, reg_communicationTimeout, 500); 
        writeHoldingRegister(drive, mb, reg_CommunicationErrorDetection, 3);
        writeHoldingRegister(drive, mb, RegDriver.reg_operationData, 2);
        writeHoldingRegister(drive, mb, RegDriver.reg_operationDataNo2_type, 16);
        writeHoldingRegister(drive, mb, RegDriver.reg_operationDataNo2_accelerationTime, accelerationRate);
        writeHoldingRegister(drive, mb, RegDriver.reg_operationDataNo2_decelerationTime, decelerationRate);
        writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandUpper, 0);
        writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandLower, 0);
        //-
    }

    void getSpeed_drive(Slave &drive, Master &mb, int rev, statusDriver& sttDriver){
        // usleep(timeWait);
        int rawData = readHoldingRegister(drive, mb, RegDriver.reg_feedbackSpeed);
        cout << "Motor speed is "<< rawData << endl;
        int spd = 0;
        if (rawData > 6000) spd = rawData - pow(2, 16);
        else{
            spd = rawData;
        }

        if (rev == 1) sttDriver.SPEED = -spd;
        else{
            sttDriver.SPEED = spd;
        }

    }

	int check_query(){  // - OK
		if (is_readQuery_1 == 1){
			ros::Duration t = ros::Time::now() - lastTime_checkQuery_1;
			if (t.toSec() >= cycle_checkQuery){
				return 0;
            }
			else{
				return 1;
            }
        }
		else{
			return 0;
        }
    }

    int exec_command(Slave &drive, Master &mb, int add, int value, double t1){

        struct timeval time_now {};
        gettimeofday(&time_now, nullptr);
        time_t msecs_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
        static int32_t count = 0;

        double duration = msecs_time - time_exec;
        if(duration >= t1){
            cout << "Im run here -----------" << count << "     " << msecs_time << endl;
            writeHoldingRegister(drive, mb, add, value);
            count++;
            return 1;
        }

        else{
            writeHoldingRegister(drive, mb, add, value);
        }
        return 0;
    }

    void spin_motor(Slave &drive, Master &mb, int spd_1, int rev){
        int vel_1 = spd_1;
        int rev_1 = rev;

        if(spd_1 == 0){
            if(disable_brake.data == 0){
                writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandLower, 1);

            }
            else{
                writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandLower, 65);
            }
            writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandUpper, 0);
        }

        else{
            if (mt_step_run == 0){
                writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandLower, 1);
                writeHoldingRegister(drive, mb, RegDriver.reg_operationData, 2);
                writeHoldingRegister(drive, mb, RegDriver.reg_operationDataNo2_type, 16);
                mt_step_run = 1;
            }
            else{
                if(spd_1 > 0){
                    if(rev_1 == 0) writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandUpper, 516);
                    else if(rev_1 == 1) writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandUpper, 520);
                }
                else{
                    if(rev_1 == 0) writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandUpper, 520);
                    else if(rev_1 == 1) writeHoldingRegister(drive, mb, RegDriver.reg_inputCommandUpper, 516);
                }
                writeHoldingRegister(drive, mb, RegDriver.reg_operationDataNo2_velocity, abs(spd_1));
            }
        }

    }

    void motor_run(Slave &drive1, Master &mb, int speed, int rev, statusDriver sttDriver)     // OK ==> motor can run follow this code. Example, exec function with speed = 1000
    {
        if(mt_step_run == 0){
            writeHoldingRegister(drive1, mb, RegDriver.reg_inputCommandLower, 1);
            writeHoldingRegister(drive1, mb, RegDriver.reg_operationData, 2);
            writeHoldingRegister(drive1, mb, RegDriver.reg_operationDataNo2_type, 16);

            mt_step_run = 1;
            cout << "mode = 0" << endl;
        }
        else if(mt_step_run == 1)
        {
            writeHoldingRegister(drive1, mb, RegDriver.reg_inputCommandUpper, 516);
            writeHoldingRegister(drive1, mb, RegDriver.reg_operationDataNo2_velocity, speed);
            getSpeed_drive(drive1, mb, rev, sttDriver);
        }

    }

    void run(Slave &drive, Master &mb, int rev, message_pkg::Driver_respond driverRespond, message_pkg::Driver_query driverQuery, statusDriver& sttDriver, ros::Publisher& rospb ){
        // -- check communicate
        if (check_query() == 1){ //  and  check_safety() == 1
            enable_run = 1;
            driverRespond.status = status_run;
            value_runing = 1;
        }
        else{
            enable_run = 0;
            driverRespond.status = status_stop;
            value_runing = 0;
        }

        if (driverQuery.task == 1){
            reset_drive(drive, mb, sttDriver);
        }

        // -- Task Read Status
        getStatus_drive(drive, mb, 0, sttDriver);
        getSpeed_drive(drive, mb, rev, sttDriver);

        enable_run = 1;
        if (enable_run){
            spin_motor(drive, mb, 1000, rev);
        }
        else{
            spin_motor(drive, mb, 0, 1);
        }

        driverRespond.REV = sttDriver.REV;
        driverRespond.FWD = sttDriver.FWD;
        driverRespond.speed = sttDriver.SPEED;
        driverRespond.alarm_all = sttDriver.PRESENT_ALARM;
        // driverRespond_1.alarm_overload = sttDriver.ALARM_OUT1;
        driverRespond.warning = sttDriver.ALARM_OUT1;
        driverRespond.message_error = mean_ALM_A(sttDriver.PRESENT_ALARM);
        driverRespond.message_warning = mean_ALM_A(sttDriver.PRESENT_ALARM);
        rospb.publish(driverRespond);
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Driver_control");
    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle("~");
    ros::Rate rate(200); // ROS Rate at 5Hz

    driver self = driver(&nh, &private_node_handle);
    // str config
    string strConfig = self.BAUDRATE + "E1";
    // khoi tao modbus
    Master MB (Rtu, self.PORT, strConfig);
    MB.rtu().setSerialMode(Rs485);
    
    // add slave
    Slave &drive1 = MB.addSlave(self.ID_driver1);
    Slave &drive2 = MB.addSlave(self.ID_driver2);

    while (!MB.open())
    {
        cout << "Unable to open MODBUS connection to " << self.PORT << " : " << MB.lastError() << endl;
        sleep(5);
    }

    cout << "Open MODBUS connection ^-^" << endl;

    // driver 1
    self.config_drive(drive1, MB);
    self.getStatus_drive(drive1, MB, 1, self.statusDriver_1);
    self.reset_drive(drive1, MB, self.statusDriver_1);
    self.getStatus_drive(drive1, MB, 1, self.statusDriver_1);
    self.writeHoldingRegister(drive1, MB, self.RegDriver.reg_inputCommandLower, 1);

    // driver 2
    self.config_drive(drive2, MB);
    self.getStatus_drive(drive2, MB, 1, self.statusDriver_2);
    self.reset_drive(drive2, MB, self.statusDriver_2);
    self.getStatus_drive(drive2, MB, 1, self.statusDriver_2);
    self.writeHoldingRegister(drive2, MB, self.RegDriver.reg_inputCommandLower, 1);

    while (ros::ok()) {
        self.run(drive1, MB, self.revert_1, self.driverRespond_1, self.driverQuery_1, self.statusDriver_1, self.pub_driverRespond_1);
        self.run(drive2, MB, self.revert_2, self.driverRespond_2, self.driverQuery_2, self.statusDriver_2, self.pub_driverRespond_2);
        rate.sleep();
        ros::spinOnce(); 
    }
    return 0;
}
