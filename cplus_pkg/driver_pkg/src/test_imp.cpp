// #include "ros/ros.h"
// #include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <string>
#include <modbuspp.h>
#include <ctime>

using namespace std;
using namespace Modbus;

// -- 
int perimeter = 0.47;
int transmission_ratio = 30;

int accelerationRate = 1000; // [1 to 1,000,000] ms
int decelerationRate = 200;// [1 to 1,000,000] ms
int torqueLimiting = 200; // 200    // 0 to 10,000 (1=0.1%)
// -- 
int reg_communicationTimeout = 5003; // P222
int reg_CommunicationErrorDetection = 5005; // 
// -- Registers
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
int reg_operationDataNo0_decelerationTime = 6159;//
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

// --------
int bitInputCommand_FW_JOG = 0;
int bitInputCommand_FV_JOG = 1;
int bitInputCommand_FW_SPD = 2;
int bitInputCommand_FV_SPD = 3;
int bitInputCommand_HOME = 4;
int bitInputCommand_NOT5 = 5;
int bitInputCommand_START = 6;
int bitInputCommand_SSTART = 7;
int bitInputCommand_M0 = 8;
int bitInputCommand_M1 = 9;
int bitInputCommand_M2 = 10;
int bitInputCommand_M3 = 11;
int bitInputCommand_M4 = 12;
int bitInputCommand_M5 = 13;
int bitInputCommand_M6 = 14;
int bitInputCommand_M7 = 15;
// -- 
int bitInputCommand_S_ON = 0;
int bitInputCommand_PLOOP_MODE = 1;
int bitInputCommand_TRQ_LMT = 2;
int bitInputCommand_CLR   = 3;
int bitInputCommand_QSTOP = 4;
int bitInputCommand_STOP  = 5;
int bitInputCommand_FREE  = 6;
int bitInputCommand_ALM_RST = 7;
int bitInputCommand_D_SEL0  = 8;
int bitInputCommand_D_SEL1  = 9;
int bitInputCommand_D_SEL2  = 10;
int bitInputCommand_D_SEL3  = 11;
int bitInputCommand_D_SEL4  = 12;
int bitInputCommand_D_SEL5  = 13;
int bitInputCommand_D_SEL6  = 14;
int bitInputCommand_D_SEL7  = 15;
// --
int bitOutputStatus_INFO     = 0;
int bitOutputStatus_INFO_MNT = 1;
int bitOutputStatus_DRVTMP   = 2;
int bitOutputStatus_MTRTMP   = 3;
int bitOutputStatus_INFO_TRQ = 4;
int bitOutputStatus_INFO_WATT   = 5;
int bitOutputStatus_INFO_VOLTH  = 6;
int bitOutputStatus_INFO_VOLTL  = 7;
int bitOutputStatus_CONST_OFF24 = 8;
int bitOutputStatus_CONST_OFF25 = 9;
int bitOutputStatus_CONST_OFF26 = 10;
int bitOutputStatus_CONST_OFF27 = 11;
int bitOutputStatus_CONST_OFF28 = 12;
int bitOutputStatus_CONST_OFF29 = 13;
int bitOutputStatus_USR_OUT0    = 14;
int bitOutputStatus_USR_OUT1    = 15;
// --
int bitOutputStatus_SON_MON   = 0;
int bitOutputStatus_PLOOP_MON = 1;
int bitOutputStatus_TRQ_LMTD  = 2;
int bitOutputStatus_RDY_DD = 3;
int bitOutputStatus_ABSPEN = 4;
int bitOutputStatus_STOP_R = 5;
int bitOutputStatus_FREE_R = 6;
int bitOutputStatus_ALM_A  = 7;
int bitOutputStatus_SYS_BSY  = 8;
int bitOutputStatus_IN_POS   = 9;
int bitOutputStatus_RDY_HOME = 10;
int bitOutputStatus_RDY_FWRV = 11;
int bitOutputStatus_RDY_SD   = 12;
int bitOutputStatus_MOVE = 13;
int bitOutputStatus_VA   = 14;
int bitOutputStatus_TLC  = 15;

// - biến lưu thời gian
int time_readStatus = 2;
clock_t lastTime_readStatus = clock();

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
            cout << ret << " registers written (16-bit)." << endl;
        }
        mb.close();

    }else{
        cerr << "Unable to open MODBUS connection, because " << mb.lastError() << endl;
        // exit (EXIT_FAILURE);
    }
    return outValue;
}

int resetAlarm(Slave &drive1, Master &mb){
    int numError = 0;
    if(writeHoldingRegister(drive1, mb, reg_resetAlarm, 1) < 0){numError++;} 
    if(writeHoldingRegister(drive1, mb, reg_resetAlarm, 0) < 0){numError++;} 
    return numError;
}

void getStatus_all(Slave &drive1, Master &mb, int type){
    int numError = 0;
    int value = readHoldingRegister(drive1, mb, reg_presentAlarm);
    cout << value << endl;
    sleep(0.1);
}

void configAll(Slave &drive1, Master &mb){
    int numError = 0;
    // time out
    if(writeHoldingRegister(drive1, mb, reg_communicationTimeout, 500) < 0){numError++;} 
    if(writeHoldingRegister(drive1, mb, reg_CommunicationErrorDetection, 3) < 0){numError++;} 
    if(writeHoldingRegister(drive1, mb, reg_operationData, 2) < 0){numError++;} 
    if(writeHoldingRegister(drive1, mb, reg_operationDataNo2_type, 16) < 0){numError++;} 
    if(writeHoldingRegister(drive1, mb, reg_operationDataNo2_accelerationTime, accelerationRate) < 0){numError++;} 
    if(writeHoldingRegister(drive1, mb, reg_operationDataNo2_decelerationTime, decelerationRate) < 0){numError++;} 
    if(writeHoldingRegister(drive1, mb, reg_inputCommandUpper, 0) < 0){numError++;} 
    if(writeHoldingRegister(drive1, mb, reg_inputCommandLower, 0) < 0){numError++;} 
    //-
}


int main(int argc, char** argv)
{
    // param
    string PORT = "/dev/ttyUSB0";
    string BAUDRATE = "19200";

    int ID_driver1 = 1;
    int ID_driver2 = 2;

    int maxRPM = 3800;
    int minRPM = maxRPM*(-1);
    // str config
    string strConfig = BAUDRATE + "E1";
    // khoi tao modbus
    Master MB (Rtu, PORT, "19200E1");
    MB.rtu().setSerialMode(Rs485);
    // add slave
    Slave &drive1 = MB.addSlave(ID_driver1);
    Slave &drive2 = MB.addSlave(ID_driver2);

    while (!MB.open())
    {
        cout << "Unable to open MODBUS connection to " << PORT << " : " << MB.lastError() << endl;
        sleep(5);
    }

    cout << "Open MODBUS connection ^-^" << endl;
    configAll(drive1, MB);
    getStatus_all(drive1, MB, 1);
    resetAlarm(drive1, MB);
    getStatus_all(drive1, MB, 1);

    // turn S-ON
    writeHoldingRegister(drive1, MB, reg_inputCommandLower, 1);

    while (true)
    {
        getStatus_all(drive1, MB, 1);
    }

    return 0;
}