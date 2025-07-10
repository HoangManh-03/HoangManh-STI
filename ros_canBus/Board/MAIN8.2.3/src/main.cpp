#include <Arduino.h>
#include "CAN_manager.h"
#include "Main_controller.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

void create_task();
TaskHandle_t Task_Run_ControlLed; // Task_Run_CAN

CAN_manager* MainCAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
Main_controller* MainCtrl = new Main_controller(MainCAN);

void LedHandle_Run( void* pvParameters ){
    for(;;){
        // led control
        MainCtrl->loop_controlLed();
        vTaskDelay(4);
    }
}

void create_task() {   
    xTaskCreatePinnedToCore(
        LedHandle_Run, 	/* Task function. */
        "Task_Run_ControlLed", /* name of task. */
        10000,          /* Stack size of task */
        NULL,         	/* parameter of the task */
        1,            	/* priority of the task */
        &Task_Run_ControlLed,  /* Task handle to keep track of created task */
        1);           	/* pin task to core 0 */              
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    MainCtrl->init_main();
    delay(50);
    // -
    MainCAN->CAN_prepare();
    delay(100);

    MainCtrl->setCoefficient_voltage(0.7415, 129);
    MainCtrl->setCoefficient_current(6.4103, -19384.615);

    create_task();

}

void loop() {
    MainCtrl->loopMainCtr();
}
