#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include "hardwareconfig.h"
#include "controll_config.h"
#include "CAN_manager.h"
#include <WiFi.h>  

struct Controll_OC{
    uint8_t CAN_address;
    uint8_t command = DO_NOTHING;
    uint8_t status = have_sent;
    bool resetError = false;

    Controll_OC(int adr) {CAN_address = adr;}
    ~Controll_OC();
};

struct Status_OC{ // - OC
    uint8_t CAN_address;
    uint8_t command = DO_NOTHING;
    uint8_t commandStatus = COMMAND_DONE;
    uint8_t sensorBit_status = 0;
    uint8_t error = ISOK;

    Status_OC(uint8_t adr) {CAN_address = adr;}
    ~Status_OC(){}
};

struct Control_HC{
    uint8_t CAN_address;
    uint8_t RGB1 = 0;
    uint8_t RGB2 = 0;

    Control_HC(int adr) {CAN_address = adr;}
    ~Control_HC();
};

struct Status_HC{ // - HC
    uint8_t CAN_address;
    uint8_t status = 0;
    uint8_t zone_sick_ahead = 0;
    uint8_t zone_sick_behind = 0;
    uint8_t vacham = 0;

    Status_HC(uint8_t adr) {CAN_address = adr;}
    ~Status_HC(){}
};


struct Control_Main{ // Main
    uint8_t CAN_address;
    bool sound_enb = 0;
    uint8_t sound_type = 0;
    bool charge_write = 0;
    bool EMG_reset = 0;
    bool EMG_write = 0;
    Control_Main(uint8_t adr) {CAN_address = adr;}
    ~Control_Main(){}
};

struct Status_Main {
    uint8_t CAN_address;
    uint8_t byte0_analogVoltage = 0;
    uint8_t byte1_analogVoltage = 0;
    uint8_t byte0_analogCurrent = 0;
    uint8_t byte1_analogCurrent = 0;
    bool stsButton_reset = 0;
    bool stsButton_power = 0;
    bool EMG_status = 0;
    Status_Main(uint8_t adr) {CAN_address = adr;}
    ~Status_Main(){}
};

class Main_controller
{
    private:
        
        CAN_manager* CAN_main = NULL;

        unsigned long saveTime_checkHC = 0;
        unsigned long saveTime_checkOC = 0;
        unsigned long saveTime_checkMain = 0;

        unsigned long saveTime_checkCAN = 0;
        unsigned long saveTime_checkReset = 0;

        // - for reset EMG
        int flag_reseting = 0;
        unsigned long saveTime_checkReseted = 0;
        unsigned long saveTime_sendCAN = 0;
        // --
        bool switch_sendCAN = 0;

    public:
        Main_controller(CAN_manager*);
        ~Main_controller();

        Status_HC* HC_status = new Status_HC(ID_HC);
        Control_HC* HC_comd = new Control_HC(ID_HC);

        Status_Main* Main_status = new Status_Main(ID_MAIN);
        Control_Main* Main_comd = new Control_Main(ID_MAIN);

        Controll_OC* OC_comd = new Controll_OC(ID_OC);
        Status_OC* OC_status = new Status_OC(ID_OC);

        bool stsSend_CAN = 0;
        bool stsRev_CAN_HC = 0;
        bool stsRev_CAN_Main = 0;
        bool stsRev_CAN_OC = 0;
        bool stsRev_CAN = 0;
        bool statusBitSensor[8];

        void led_start();

        void CANReceiveHandle();
        bool CANTransmitHandle();

        void init_main();
        void loopMainCtr();

        bool getBit_fromInt(int, int);
        int bytes_to_int(uint8_t, uint8_t);
        uint8_t int_to_byte0(int);
        uint8_t int_to_byte1(int);
};
#endif