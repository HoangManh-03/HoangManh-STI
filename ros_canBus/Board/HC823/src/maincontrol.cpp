/*==================================================================================================
*   Project              :  MAIN AI CONTROL
*   Doccument            :  ESP32S 
*   FileName             :  maincontrol.cpp
*   File Description     :  Dinh nghia cac ham thu vien su dung trong maincontrol.h
*
==================================================================================================*/
/*==================================================================================================
Revision History:
Modification     
    Author                  Date D/M/Y     Description of Changes
------------------------    -----------    ---------------------------------------------------------
    Do Xuan An              30/07/2020     Tao file
------------------------    -----------    ---------------------------------------------------------
    Hoang van Quang         10/05/2023     Edit
------------------------    -----------    ---------------------------------------------------------
    Ho Van Hoang            10/05/2024     Update
------------------------    -----------    ---------------------------------------------------------

==================================================================================================*/
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "maincontrol.h"
#include "hardwareconfig.h"
#include "controll_config.h"
/*==================================================================================================
*                                     FILE VERSION CHECKS
==================================================================================================*/

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/
/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
PCF8574 pcf8574(PCF8574_ADDRESS);
// PCF8574 pcf8574(0x38);

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
/*==================================================================================================
*                                        LOCAL FUNCTION 
==================================================================================================*/
/*==================================================================================================
*                                      GLOBAL FUNCTIONS
==================================================================================================*/
MainControl::MainControl()
{

}

MainControl::~MainControl()
{

}

/* Setup chân điều khiển */
void MainControl::MainInit() {
    
    delay(10);
    pcf8574.begin();
    pinMode(BADERSHOCK_ISR, INPUT_PULLUP);

    pinMode(hcSickTopSelOut, OUTPUT);
    pinMode(hcSickBotSelOut, OUTPUT);

    delay(10);
    Can_Main->CAN_prepare();
    LOG_MESS("Begin");

    digitalWrite(hcSickTopSelOut, 1);
    digitalWrite(hcSickBotSelOut, 1);
}

/* Hàm nhận CAN */
void MainControl::CanRecevieHandle() {
    if (Can_Main->CAN_ReceiveFrom(ID_RTC)) {
        if (ID_HC == Can_Main->GetByteReceived(0)){
            ledControl_1    = Can_Main->GetByteReceived(1);
            ledControl_2    = Can_Main->GetByteReceived(2);
            sickSelectFeild_1 = Can_Main->GetByteReceived(3);
            sickSelectFeild_2 = Can_Main->GetByteReceived(4);

            preTime_checkCAN = millis();
        }
    }
}

/* Loop điều khiển LED */
void MainControl::LedControler()
{
    static uint8_t dimmer1 = 0, dimmer2 = 0;
    static bool flager1 = true, flager2 = true;

    // - Led 1
    if (ledControl_1 == red_blink){ // error : sang nhanh, nhay do 
        if ((millis() - preTime_led1) > 1)
        {
            preTime_led1 = millis();
            if (dimmer1 >= 254){
                flager1 = false;
            }
            else if (dimmer1 <= 1){
                flager1 = true;
            }

            if(flager1){
                dimmer1 ++;
            }
            else{
                dimmer1 --;
            }

            ledFCControlBefor->setColor(dimmer1, 0, 0);
        }
    }else if (ledControl_1 == green_fading){ // di chuyen : sang cham ,xanh la 
        if ((millis() - preTime_led1) > 10) {
            preTime_led1 = millis();
            if (dimmer1 >= 254){
                flager1 = false;
            }else if (dimmer1 <= 1){
                flager1 = true;
            }

            if(flager1){
                dimmer1 ++;
            }else{
                dimmer1 --;
            }

            ledFCControlBefor->setColor(0, dimmer1, 0);
        }

    }else if (ledControl_1 == green_blink){ // tag : sang nhanh , xanh la 
        if ((millis() - preTime_led1) < 500)
        {   
            ledFCControlBefor->setColor(0, 254, 0);
        }
        if (((millis() - preTime_led1) > 500) && (millis() - preTime_led1) < 700)
        {   
            ledFCControlBefor->setColor(0, 0, 0);

        }
        if ((millis() - preTime_led1) > 700)
        {   
            preTime_led1 = millis();
        }

    }else if (ledControl_1 == WhiteBlue_fading){ // nang, ha : sang cham , xanh duong 
        if ((millis() - preTime_led1) > 10){
            preTime_led1 = millis();
            if (dimmer1 >= 254){
                flager1 = false;
            }else if (dimmer1 <= 1){
                flager1 = true;
            }

            if(flager1){
                dimmer1 ++;
            }else{
                dimmer1 --;
            }

            ledFCControlBefor->setColor(0, dimmer1, dimmer1);
        }

    }else if (ledControl_1 == yellow_fading){ // sac: sang cham , vang 
        if ((millis() - preTime_led1) > 10){
            preTime_led1 = millis();
            if (dimmer1 >= 254){
                flager1 = false;
            }else if (dimmer1 <= 1){
                flager1 = true;
            }

            if(flager1){
                dimmer1 ++;
            }else{
                dimmer1 --;
            }

            ledFCControlBefor->setColor(dimmer1, dimmer1, 0);
        }

    }else if (ledControl_1 == purple_blink){ //
        if ((millis() - preTime_led1) < 200){   
            ledFCControlBefor->setColor(254, 0, 254);
        }
        if (((millis() - preTime_led1) > 200) && (millis() - preTime_led1) < 300){   
            ledFCControlBefor->setColor(0, 0, 0);
        }
        if ((millis() - preTime_led1) > 300){   
            preTime_led1 = millis();
        }

    }else if (ledControl_1 == yellow_blink){
        if ((millis() - preTime_led1) < 500)
        {   
            ledFCControlBefor->setColor(0, 0, 254);
        }
        if (((millis() - preTime_led1) > 500) && (millis() - preTime_led1) < 700)
        {   
            ledFCControlBefor->setColor(0, 0, 0);

        }
        if ((millis() - preTime_led1) > 700)
        {   
            preTime_led1 = millis();
        }
    }else{
        ledFCControlBefor->setColor(0, 0, 0);
    }

    // - Led 2
    if (ledControl_2 == red_blink){ // error : sang nhanh, nhay do 
        if ((millis() - preTime_led2) > 1)
        {
            preTime_led2 = millis();
            if (dimmer2 >= 254){
                flager2 = false;
            }
            else if (dimmer2 <= 1){
                flager2 = true;
            }

            if(flager2){
                dimmer2 ++;
            }
            else{
                dimmer2 --;
            }

            ledFCControlApter->setColor(dimmer2, 0, 0);
        }
    }else if (ledControl_2 == green_fading){ // di chuyen : sang cham ,xanh la 
        if ((millis() - preTime_led2) > 10) {
            preTime_led2 = millis();
            if (dimmer2 >= 254){
                flager2 = false;
            }else if (dimmer2 <= 1){
                flager2 = true;
            }

            if(flager2){
                dimmer2 ++;
            }else{
                dimmer2 --;
            }

            ledFCControlApter->setColor(0, dimmer2, 0);
        }

    }else if (ledControl_2 == green_blink){ // tag : sang nhanh , xanh la 
        if ((millis() - preTime_led2) < 500)
        {   
            ledFCControlApter->setColor(0, 254, 0);
        }
        if (((millis() - preTime_led2) > 500) && (millis() - preTime_led2) < 700)
        {   
            ledFCControlApter->setColor(0, 0, 0);

        }
        if ((millis() - preTime_led2) > 700)
        {   
            preTime_led2 = millis();
        }


    }else if (ledControl_2 == WhiteBlue_fading){ // nang, ha : sang cham , xanh duong 
        if ((millis() - preTime_led2) > 10){
            preTime_led2 = millis();
            if (dimmer2 >= 254){
                flager2 = false;
            }else if (dimmer2 <= 1){
                flager2 = true;
            }

            if(flager2){
                dimmer2 ++;
            }else{
                dimmer2 --;
            }

            ledFCControlApter->setColor(0, dimmer2, dimmer2);
        }
    }else if (ledControl_2 == yellow_fading){ // sac: sang cham , vang 
        if ((millis() - preTime_led2) > 10){
            preTime_led2 = millis();
            if (dimmer2 >= 254){
                flager2 = false;
            }else if (dimmer2 <= 1){
                flager2 = true;
            }

            if(flager2){
                dimmer2 ++;
            }else{
                dimmer2 --;
            }

            ledFCControlApter->setColor(dimmer2, dimmer2, 0);
        }
    }else if (ledControl_2 == purple_blink){ //
        if ((millis() - preTime_led2) < 200){   
            ledFCControlApter->setColor(254, 0, 254);
        }
        if (((millis() - preTime_led2) > 200) && (millis() - preTime_led2) < 300){   
            ledFCControlApter->setColor(0, 0, 0);
        }
        if ((millis() - preTime_led2) > 300){   
            preTime_led2 = millis();
        }
    }else if (ledControl_2 == yellow_blink){
        if ((millis() - preTime_led2) < 500)
        {   
            ledFCControlApter->setColor(0, 0, 254);
        }
        if (((millis() - preTime_led2) > 500) && (millis() - preTime_led2) < 700)
        {   
            ledFCControlApter->setColor(0, 0, 0);

        }
        if ((millis() - preTime_led2) > 700)
        {   
            preTime_led2 = millis();
        }
    }else{
        ledFCControlApter->setColor(0, 0, 0);
    }
}

/* Hàm gửi CAN */
bool MainControl::CanTransmitHandle()
{    
    Can_Main->SetByteTransmit(statusRev_CAN, 0);
    Can_Main->SetByteTransmit(sickRespon_before, 1);
    Can_Main->SetByteTransmit(sickRespon_after, 2);
    Can_Main->SetByteTransmit(status_blsock, 3);

    if(Can_Main->CAN_Send())
    {
        // LOG_NUM(sickRespon_before);
        // time_ = millis();
        return true;
    }
    // LOG_MESS_STRING("CAN_Send NOT OK");
    return false;
}

/* Loop test led */
void MainControl::LedTest(){
    // LOG_MESS_STRING("Start Test");
    ledFCControlApter->ledByPWM(255,0,0);
    delay(3000);
    ledFCControlApter->ledByPWM(0,255,0);
    delay(3000);
    ledFCControlApter->ledByPWM(0,0,255);
    delay(3000);
    ledFCControlApter->ledByPWM(255,255,255);
    delay(3000);
    ledFCControlApter->ledByPWM(0,0,0);
    delay(3000);
}

/* Đọc giá trị chân cảm biến vùng SICK */
void MainControl::read_SICK(){
    sickRespon_before = pcf8574.readButton(hcSickTopPin1In) | (pcf8574.readButton(hcSickTopPin2In) << 1) | (pcf8574.readButton(hcSickTopPin3In) << 2) | (pcf8574.readButton(hcSickTopPin4In) << 3);
    sickRespon_after  = pcf8574.readButton(hcSickBotPin1In) | (pcf8574.readButton(hcSickBotPin2In) << 1) | (pcf8574.readButton(hcSickBotPin3In) << 2) | (pcf8574.readButton(hcSickBotPin4In) << 3);

    // Serial.print(sickRespon_before);
    // Serial.print(" | ");
    // Serial.println(sickRespon_after);
}

/* Kiểm tra lỗi badershock */
void MainControl::BarderShockChecker(){ 
    if(digitalRead(BADERSHOCK_ISR) == 0){
        status_blsock = 1;
    }else{
        status_blsock = 0;
    }
}

/* Loop điều khiển HC */
void MainControl::Run()
{
    // - Check received CAN.
    if ((millis() - preTime_checkCAN) < 1500){
        statusRev_CAN = 1;
    }else {
        statusRev_CAN = 0;
    }
    
    // -- SEND
    if ((millis() - preTime_sendCAN) > (1000/ FREQUENCY_sendCAN)){   
        preTime_sendCAN = millis();
        BarderShockChecker();
        read_SICK();
        CanTransmitHandle();
    }

    CanRecevieHandle();

    // - Sick Select Feild
    // if (sickSelectFeild_1 == 0){
    //     digitalWrite(hcSickTopSelOut, 1);
    // }else{
    //     digitalWrite(hcSickTopSelOut, 0);
    // }

    // if (sickSelectFeild_2 == 0){
    //     digitalWrite(hcSickBotSelOut, 1);
    // }else{
    //     digitalWrite(hcSickBotSelOut, 0);
    // }
}
//------------------------------------------END FILE----------------------------------------------//
