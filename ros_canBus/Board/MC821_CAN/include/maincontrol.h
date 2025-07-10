/*==================================================================================================
*   Project              :  MAIN AI CONTROL
*   Doccument            :  ESP32S 
*   FileName             :  modbuscontrol.h
*   File Description     :  Khai bao cac ham su dung chinh
*
==================================================================================================*/
/*==================================================================================================
Revision History:
Modification     
    Author                  	Date D/M/Y     Description of Changes
----------------------------	----------     ------------------------------------------
    Do Xuan An              	30/07/2020     Tao file
----------------------------	----------     ------------------------------------------
==================================================================================================*/
#ifndef __MAIN_CONTROL_H
#define __MAIN_CONTROL_H 
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "cancontrol.h"
#include "hardwareconfig.h"
/*==================================================================================================
*                                        FILE VERSION 
==================================================================================================*/

/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/
 
/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

/*==================================================================================================
*                                             ENUMS
==================================================================================================*/

/*==================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

/*==================================================================================================
*                                             CLASS
==================================================================================================*/
struct Control_motor{ // Main
    uint8_t dir = 0;
    uint8_t speed = 0;
    
    Control_motor() {}
    ~Control_motor(){}
};

struct Status_motor{ // Main
    uint8_t alamp = 0;
    uint8_t dir = 0;
    uint8_t speed = 0;
    
    Status_motor() {}
    ~Status_motor(){}
};

class MainControl
{
    private:
        uint8_t sickSelectFeild = 0;
        CAN_manager *Can_Main = new CAN_manager(CAN_SPEED_125KBPS, GPIO_NUM_5, GPIO_NUM_4, CAN_frame_std, ID_MC, 8);
        unsigned long preTime_sendCAN = 0;
        unsigned long preTime_recCAN = 0;
        unsigned long preTime_control = 0;

        uint8_t modeOperate = 3; // 3
        uint8_t mode_unkown = 0;    // - Không xác định: Ưu tiên dừng ngay lập tức.
        uint8_t mode_running = 1;   // - Quay.
        uint8_t mode_stopInsta = 2; // - Dừng lập tức.
        uint8_t mode_stopDecel = 3; // - Dừng từ từ.
        // - 
        uint8_t flagReset = 0;
        uint8_t flagReset_old = 0;
        uint8_t flag_lostCAN = 0;
        uint8_t EMG_status = 0;
        uint8_t flag_stop = 0;

        Control_motor* controlMotor1 = new Control_motor();
        Control_motor* controlMotor2 = new Control_motor();
        Control_motor* controlMotor1_write = new Control_motor();
        Control_motor* controlMotor2_write = new Control_motor();

        Status_motor* statusMotor1 = new Status_motor();
        Status_motor* statusMotor2 = new Status_motor();

    public:
        MainControl();
        ~MainControl();
        void MainInit(void);
        void CanRecevieHandle(void);
        bool CanTransmitHandle(void); 
        void Moving(void);
        void Run(void);
        void Test_GPIO(void);
        void setRPM1(float rpm){
            statusMotor1->speed = rpm;
        };
        void setRPM2(float rpm){
            statusMotor2->speed = rpm;
        }
        void testMove(uint8_t analog);

};
#endif /*<!__MAIN_CONTROL_H>*/
//------------------------------------------END FILE----------------------------------------------//
