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
==================================================================================================*/
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "maincontrol.h"
#include "hardwareconfig.h"
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
// PCF8574 pcf8574(0x20);
PCF8574 pcf8574(PCF_address);
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

    // pinMode(BADERSHOCK_ISR, INPUT_PULLUP);

    /* ---- OUTPUT ---- */
    pinMode(analogOut_1, OUTPUT);
    pinMode(analogOut_2, OUTPUT);

    pinMode(enable_2, OUTPUT);
    pinMode(run_2, OUTPUT);

    delay(10);

    /* ---- INPUT ---- */
    pinMode(alampRead_1, INPUT_PULLUP);
    pinMode(alampRead_2, INPUT_PULLUP);
    pinMode(EMG, INPUT_PULLUP);
    pinMode(SENSOR_1BIT, INPUT_PULLUP);

    /* ---- PCF ---- */
    pcf8574.write(direction_1, 0);
    pcf8574.write(enable_1, 0); 
    pcf8574.write(run_1, 0);
    pcf8574.write(direction_2, 0); 
    pcf8574.write(resetAlamp, 0); 

    /* ---- DAC ---- */
    dacWrite(analogOut_1, 0);
    dacWrite(analogOut_2, 0);

    Can_Main->CAN_prepare();
    LOG_MESS("Begin");

    // - Launch
    // - SPEED
    dacWrite(analogOut_1, 0);
    dacWrite(analogOut_2, 0);

    // - START/STOP
    pcf8574.write(enable_1, 0); 
    digitalWrite(enable_2, 0);

    // - RUN/BRAKE
    pcf8574.write(run_1, 1); 
    digitalWrite(run_2, 1);

    // - ALARM-RESET
    pcf8574.write(resetAlamp, 1); // - DE Active
}

/* Hàm nhận CAN */
void MainControl::CanRecevieHandle() {
    if(Can_Main->CAN_ReceiveFrom(ID_RTC)) {
        // LOG_MESS_STRING("CAN REC OK");
        if(ID_MC == Can_Main->GetByteReceived(receByte_id)){
            controlMotor1->dir   = Can_Main->GetByteReceived(receByte_dir1);
            controlMotor2->dir   = Can_Main->GetByteReceived(receByte_dir2);
            controlMotor1->speed = Can_Main->GetByteReceived(receByte_speed1);
            controlMotor2->speed = Can_Main->GetByteReceived(receByte_speed2);
            modeOperate          = Can_Main->GetByteReceived(receByte_mode);
            flagReset            = Can_Main->GetByteReceived(receByte_reset);
            preTime_recCAN = millis();
            // LOG_MESS_STRING("CAN REC MC OK");
            // LOG_MESS_STRING("OK");
        }
    }
}

/* Hàm gửi CAN */
bool MainControl::CanTransmitHandle()
{    
    // Can_Main->SetByteTransmit(statusMotor1->alamp, sendByte_alamp1);
    // Can_Main->SetByteTransmit(statusMotor2->alamp, sendByte_alamp2);
    // Can_Main->SetByteTransmit(statusMotor1->dir,   receByte_dir1);
    // Can_Main->SetByteTransmit(statusMotor2->dir,   receByte_dir2);
    // Can_Main->SetByteTransmit(statusMotor1->speed, sendByte_speed1);
    // Can_Main->SetByteTransmit(statusMotor2->speed, sendByte_speed2);
    // Can_Main->SetByteTransmit(modeOperate,         sendByte_mode);
    // Can_Main->SetByteTransmit(modeOperate,         sendByte_EMG);
    uint8_t status_1bit = digitalRead(SENSOR_1BIT);
    uint8_t byte7 = flag_stop | status_1bit << 1;

    Can_Main->SetByteTransmit(statusMotor1->alamp, 0);
    Can_Main->SetByteTransmit(statusMotor2->alamp, 1);
    Can_Main->SetByteTransmit(statusMotor1->dir,   2);
    Can_Main->SetByteTransmit(statusMotor2->dir,   3);
    Can_Main->SetByteTransmit(statusMotor1->speed, 4);
    Can_Main->SetByteTransmit(statusMotor2->speed, 5);
    Can_Main->SetByteTransmit(modeOperate,         6);
    Can_Main->SetByteTransmit(byte7,          7);

    // LOG_NUM(statusMotor1->speed);
    // LOG_NUM(EMG_status);

    if(Can_Main->CAN_Send()) {
        return true;
    }
    return false;
}

/* Hàm điều khiển tốc độ, chiều */
void MainControl::Moving(){
    if (modeOperate == mode_unkown){
        flag_stop = 0;
        controlMotor1_write->speed = 0;
        controlMotor2_write->speed = 0;
        // - SPEED
        dacWrite(analogOut_1, controlMotor1_write->speed);
        dacWrite(analogOut_2, controlMotor2_write->speed);

        // - START/STOP
        pcf8574.write(enable_1, 1); // - De Active
        digitalWrite(enable_2, 1);

        // - RUN/BRAKE
        pcf8574.write(run_1, 1); // - De Active
        digitalWrite(run_2, 1);
        
    }else if (modeOperate == mode_stopDecel){
        flag_stop = 0;
        controlMotor1_write->speed = 0;
        controlMotor2_write->speed = 0;
        // - SPEED
        dacWrite(analogOut_1, controlMotor1_write->speed);
        dacWrite(analogOut_2, controlMotor2_write->speed);

        // - START/STOP
        pcf8574.write(enable_1, 1); 
        digitalWrite(enable_2, 1);

        // - RUN/BRAKE
        pcf8574.write(run_1, 0); // - Active
        digitalWrite(run_2, 0);

    }else if (modeOperate == mode_stopInsta){
        flag_stop = 0;
        controlMotor1_write->speed = 0;
        controlMotor2_write->speed = 0;
        // - SPEED
        dacWrite(analogOut_1, controlMotor1_write->speed);
        dacWrite(analogOut_2, controlMotor2_write->speed);

        // - START/STOP
        pcf8574.write(enable_1, 0); // - Active
        digitalWrite(enable_2, 0);

        // - RUN/BRAKE
        pcf8574.write(run_1, 1); // - De Active
        digitalWrite(run_2, 1);

    }else if (modeOperate == mode_running){
        // - CW/CCW
        if (controlMotor1_write->dir != controlMotor1->dir){
            controlMotor1_write->dir = controlMotor1->dir;
            pcf8574.write(direction_1, controlMotor1_write->dir);
        }

        if (controlMotor2_write->dir != controlMotor2->dir){
            controlMotor2_write->dir = controlMotor2->dir;
            pcf8574.write(direction_2, controlMotor2_write->dir);
        }

        // -------- SPEED -------- //
        // - Stop
        if (EMG_status == 1 || flag_lostCAN == 1 || statusMotor1->alamp == 1 || statusMotor2->alamp == 1){ // - Active
            flag_stop = 1;
            controlMotor1_write->speed = 0;
            controlMotor2_write->speed = 0;
            dacWrite(analogOut_1, controlMotor1_write->speed);
            dacWrite(analogOut_2, controlMotor2_write->speed);

            // - START/STOP
            pcf8574.write(enable_1, 1); 
            digitalWrite(enable_2, 1);

            // - RUN/BRAKE
            pcf8574.write(run_1, 0); // - Active
            digitalWrite(run_2, 0);
            
        }else{
            flag_stop = 0;
            if (controlMotor1_write->speed != controlMotor1->speed){
                controlMotor1_write->speed = controlMotor1->speed;
                dacWrite(analogOut_1, controlMotor1_write->speed);
            }

            if (controlMotor2_write->speed != controlMotor2->speed){
                controlMotor2_write->speed = controlMotor2->speed;
                dacWrite(analogOut_2, controlMotor2_write->speed);
            }

            // - START/STOP
            pcf8574.write(enable_1, 0); // - Active
            digitalWrite(enable_2, 0);

            // - RUN/BRAKE
            pcf8574.write(run_1, 0); 
            digitalWrite(run_2, 0);
        }
        
    }else{ // - Set mode_stopDecel 
        flag_stop = 0;
        controlMotor1_write->speed = 0;
        controlMotor2_write->speed = 0;
        // - SPEED
        dacWrite(analogOut_1, controlMotor1_write->speed);
        dacWrite(analogOut_2, controlMotor2_write->speed);

        // - START/STOP
        pcf8574.write(enable_1, 1); // - De Active
        digitalWrite(enable_2, 1);

        // - RUN/BRAKE
        pcf8574.write(run_1, 0); // - Active
        digitalWrite(run_2, 0);
    }

    // - ALARM-RESET
    if (flagReset_old != flagReset){
        if (flagReset == 1){
            pcf8574.write(resetAlamp, 0); // - Active
        }else{
            pcf8574.write(resetAlamp, 1);
        }
        flagReset_old = flagReset;
    }
    
}

/* Loop điều khiển MC */
void MainControl::Run()
{
    // -- Send
    if ((millis() - preTime_sendCAN) > (1000/ FREQUENCY_sendCAN)){   

        statusMotor1->speed = controlMotor1_write->speed;
        statusMotor2->speed = controlMotor2_write->speed;

        statusMotor1->dir = controlMotor1->dir;
        statusMotor2->dir = controlMotor2->dir;

        preTime_sendCAN = millis();
        CanTransmitHandle();
    }

    // -- Control
    if ((millis() - preTime_control) > (1000/ FREQUENCY_control)){ 
        preTime_control = millis(); 

        // - Check Lost Recived CAN.
        if ((millis() - preTime_recCAN) > 400){ // - 250 ms.
            flag_lostCAN = 1;
        }else{
            flag_lostCAN = 0;
        }

        if (digitalRead(EMG) == 1){
            EMG_status = 0;
        }else{
            EMG_status = 1;
        }

        // -- Read
        if (digitalRead(alampRead_1) == 0){
            statusMotor1->alamp = 0;
        }else{
            statusMotor1->alamp = 1;
        }
        
        if (digitalRead(alampRead_2) == 0){
            statusMotor2->alamp = 0;
        }else{
            statusMotor2->alamp = 1;
        }

        // - Move.
        Moving();
    }
    // Test_GPIO();
}


/* TEST */
void MainControl::Test_GPIO()
{
    // - START/STOP
    pcf8574.write(enable_1, 0); 
    digitalWrite(enable_2, 0);

    // - RUN/BRAKE
    pcf8574.write(run_1, 1); 
    digitalWrite(run_2, 1);
    delay(1000);

    // - START/STOP
    pcf8574.write(enable_1, 1); 
    digitalWrite(enable_2, 1);

    // - RUN/BRAKE
    pcf8574.write(run_1, 0); 
    digitalWrite(run_2, 0);
    delay(1000);

    // pcf8574.write(direction_1, 1);
    // pcf8574.write(resetAlamp, 1); 
    // pcf8574.write(enable_1, 1); 
    // pcf8574.write(run_1, 1);
    // pcf8574.write(direction_2, 1); 
    // digitalWrite(enable_2, 0);
    // digitalWrite(run_2, 1);

    // dacWrite(analogOut_1, 0);
    // dacWrite(analogOut_2, 0);
    // delay(10000);

    // dacWrite(analogOut_1, 10);
    // dacWrite(analogOut_2, 10);
    // delay(10000);

    // dacWrite(analogOut_1, 20);
    // dacWrite(analogOut_2, 20);
    // delay(10000);

    // dacWrite(analogOut_1, 30);
    // dacWrite(analogOut_2, 30);
    // delay(10000);

    // dacWrite(analogOut_1, 40);
    // dacWrite(analogOut_2, 40);
    // delay(10000);

    // dacWrite(analogOut_1, 50);
    // dacWrite(analogOut_2, 50);
    // delay(10000);

    // dacWrite(analogOut_1, 60);
    // dacWrite(analogOut_2, 60);
    // delay(10000);

    // dacWrite(analogOut_1, 150);
    // dacWrite(analogOut_2, 150);
    // delay(10000);

    // dacWrite(analogOut_1, 255);
    // dacWrite(analogOut_2, 255);
    // delay(10000);
}

void MainControl::testMove(uint8_t ana){
    // - CW/CCW
    pcf8574.write(direction_1, 1);
    pcf8574.write(direction_2, 1);

    // -------- SPEED -------- //
    dacWrite(analogOut_1, ana);
    dacWrite(analogOut_2, ana);

    // - START/STOP
    pcf8574.write(enable_1, 0); // - Active
    digitalWrite(enable_2, 0);

    // - RUN/BRAKE
    if (ana == 0){
        pcf8574.write(run_1, 1); 
        digitalWrite(run_2, 1);
    }
    else{
        pcf8574.write(run_1, 0); 
        digitalWrite(run_2, 0);
    }

    // - Stop
    // if (EMG_status == 1 || flag_lostCAN == 1 || statusMotor1->alamp == 1 || statusMotor2->alamp == 1){ // - Active
    //     flag_stop = 1;
    //     controlMotor1_write->speed = 0;
    //     controlMotor2_write->speed = 0;
    //     dacWrite(analogOut_1, controlMotor1_write->speed);
    //     dacWrite(analogOut_2, controlMotor2_write->speed);

    //     // - START/STOP
    //     pcf8574.write(enable_1, 1); 
    //     digitalWrite(enable_2, 1);

    //     // - RUN/BRAKE
    //     pcf8574.write(run_1, 0); // - Active
    //     digitalWrite(run_2, 0);
        
    // }else{
    //     flag_stop = 0;
    //     if (controlMotor1_write->speed != controlMotor1->speed){
    //         controlMotor1_write->speed = controlMotor1->speed;
    //         dacWrite(analogOut_1, controlMotor1_write->speed);
    //     }

    //     if (controlMotor2_write->speed != controlMotor2->speed){
    //         controlMotor2_write->speed = controlMotor2->speed;
    //         dacWrite(analogOut_2, controlMotor2_write->speed);
    //     }

    //     // - START/STOP
    //     pcf8574.write(enable_1, 0); // - Active
    //     digitalWrite(enable_2, 0);

    //     // - RUN/BRAKE
    //     pcf8574.write(run_1, 0); 
    //     digitalWrite(run_2, 0);
    // }
}
//------------------------------------------END FILE----------------------------------------------//
