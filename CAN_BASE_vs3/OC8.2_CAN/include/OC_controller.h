#ifndef OC_CONTROLLER_H
#define OC_CONTROLLER_H

#include "CAN_manager.h"
#include "hardwareconfig.h"
#include "controll_config.h"
class OC_controller
{
private:
    unsigned long time_gui;
    /**
    * @brief          : luu gia tri nhan/gui cua mang giao tiep CAN
    * @details        : can_id: id can cua mach
                        cmd,cmd_status: function va trang thai (0:lo lung || 1:o tren || 2:o duoi)
                        status_r: hoan thanh lenh cmd
                        err_lx: bao loi ban nang
    * @note           : 
    */
    unsigned char   cmd_request = commands::DO_NOTHING,
                    cmd_resetError = 0,
                    cmd_status = processStatus::COMMAND_DONE,
                    pre_cmd_request = commands::DO_NOTHING,
                    err_lx = ISOK;        
    CAN_manager* CANctr = NULL;

	int step_LiftUp;	// Lưu bước nâng bàn nâng.
	int step_LiftDown;	// Lưu bước hạ bàn nâng.
	int speed_LiftUp;	// Lưu tốc độ nâng bàn nâng.	
	int speed_LiftDown;	// Lưu tốc độ nâng hạ nâng.
	unsigned long timeStart_LiftUp = 0;  	// Lưu thời gian bắt đầu nâng.
	unsigned long timeStart_LiftDown = 0;	// Lưu thời gian bắt đầu hạ.
    int timeCheckError = 40000; // ms - Do thoi gian loi.
    // --
    int FREQUENCY_sendCAN = 8;
    unsigned long preTime_sendCAN = 0;;
    // - 
	int channelPWM_LiftUp = 0;
	int channelPWM_LiftDown = 4;
    uint8_t statusRev_CAN = 0;
    unsigned long saveTime_checkCAN_Rev = 0;

public:
    OC_controller(CAN_manager*);
    ~OC_controller();
    void setCommand(unsigned char _cmd) {cmd_request = _cmd;}
    void setCommandStatus(unsigned char _cmd_status) {cmd_status = _cmd_status;}
    void setError(unsigned char _err_lx) {err_lx = _err_lx;}

    unsigned char getError() {return err_lx;}
    unsigned char getCommandStatus() {return cmd_status;}
    unsigned char getCommand() {return cmd_request;}
    bool getSensor(int _pin) {return digitalRead(_pin);}
    
    bool the_three_timer(unsigned int timer_num, unsigned int set_value){
        static unsigned int timer_value[3]={0,0,0};
        int t = millis();
        if (t- timer_value[timer_num] >= set_value){
            timer_value[timer_num] = t;
            return true;
        }
        else{
            return false;
        }
    }

    void liftUp_vs2();
    
    void liftDown_vs2();

    void ConveyorIn();
    void ConveyorOut();

    void stopAndReset();
    void setupConv();
    void OC_CAN_Transmit();
    
    void OC_CAN_Receive();
    void OC_CAN_Receive_vs2();

    void OC_CAN_send();

    void OC_loop_vs2();

    void debuge();
};


#endif