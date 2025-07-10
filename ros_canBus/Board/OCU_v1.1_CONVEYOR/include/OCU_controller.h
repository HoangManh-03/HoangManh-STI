#ifndef OCU_CONTROLLER_H
#define OCU_CONTROLLER_H

#include "CAN_manager.h"
#include "hardwareconfig.h"
#include "controll_config.h"
#include "driver_control.h"
#include "gpio_board.h"

struct CAN_SEND{ // - OC
    uint8_t mode_driver_1 = 0;              // - Byte_0
    uint8_t status_driver_1 = 0;            // - Byte_1
    uint8_t mode_driver_2 = 0;              // - Byte_2
    uint8_t status_driver_2 = 0;            // - Byte_3
    uint8_t status_sensor = 0;              // - Byte_4
    uint8_t status_can = 0;                 // - Byte_5

    CAN_SEND() {}
    ~CAN_SEND(){}
};

struct CAN_RECEIVED{ // - OC
    uint8_t mission1 = 0;
    uint8_t speed1 = 0;
    uint8_t mission2 = 0;
    uint8_t speed2 = 0;

    CAN_RECEIVED() {}
    ~CAN_RECEIVED(){}
};


class OCU_controller
{
private:
    CAN_manager* CANctr = NULL;
    // --
    int FREQUENCY_sendCAN = 8;
    unsigned long preTime_sendCAN = 0;
    // -- 
    unsigned long saveTime_checkCAN = 0;
    uint8_t statusRev_CAN = 0;
    // - 
	int channelPWM_1 = 0;
	int channelPWM_2 = 4;
    int channelPWM_3 = 8;
	int channelPWM_4 = 12;

    DriverBoard DriverOne;
    DriverBoard DriverSecond;

    GPIOBoard sensor1;
    GPIOBoard sensor2;
    GPIOBoard sensor3;
    GPIOBoard sensor4;
    GPIOBoard sensor5;
    GPIOBoard sensor6;
    GPIOBoard sensor7;

public:
    OCU_controller(CAN_manager*);
    ~OCU_controller();

    // --
    CAN_SEND* CAN_sendData = new CAN_SEND();
    CAN_RECEIVED* CAN_receivedData = new CAN_RECEIVED();
    CAN_RECEIVED* CAN_command = new CAN_RECEIVED();

    // --
    void setupBegin();
    void stopAndReset();
    // --     
    void OCU_CAN_Transmit();
    void OCU_CAN_Receive();
    void OCU_CAN_send();

    // --
    void OCU_loop();

    // -- TEST
    void Test1();
};


#endif