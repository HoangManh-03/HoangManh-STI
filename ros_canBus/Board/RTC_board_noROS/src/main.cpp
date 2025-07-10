#include <Arduino.h>
#include "CAN_manager.h"
#include "Main_controller.h"
#include "can_serial.h"

#define FREQUENCY_PUB_POWER_INFO 4
#define FREQUENCY_PUB_OC_INFO    4
#define FREQUENCY_PUB_HC_INFO    24

unsigned long saveTime_pub = 0;
unsigned long saveTime_sub = 0;
unsigned long debugTime = 0;

TaskHandle_t Task_Run_CAN; // Task_Run_CAN
CAN_manager* MainCAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
Main_controller* MainCtrl = new Main_controller(MainCAN);
CanSerial CSerial(Serial);

DataPacket data_recieve_can_serial; 
DataPacket data_send_can_serial;

void create_task();
void CanHandle_Run( void* pvParameters ){
  for(;;){
    MainCtrl->Run_Send_CAN();
    vTaskDelay(4);
  }
}

void setup() {
    MainCtrl->init_main();
    MainCtrl->led_start();

    CSerial.begin(115200);

    MainCAN->CAN_prepare();
    delay(1000);

    create_task();
}

void loop() {

    MainCtrl->CANReceiveHandle();
    /* -- -- -- -- CAN RECEIVED -- -- -- -- */
    data_recieve_can_serial.id = MainCtrl->received_CAN->idSend;
    data_recieve_can_serial.data[0] = MainCtrl->received_CAN->byte0;
    data_recieve_can_serial.data[1] = MainCtrl->received_CAN->byte1;
    data_recieve_can_serial.data[2] = MainCtrl->received_CAN->byte2;
    data_recieve_can_serial.data[3] = MainCtrl->received_CAN->byte3;
    data_recieve_can_serial.data[4] = MainCtrl->received_CAN->byte4;
    data_recieve_can_serial.data[5] = MainCtrl->received_CAN->byte5;
    data_recieve_can_serial.data[6] = MainCtrl->received_CAN->byte6;
    data_recieve_can_serial.data[7] = MainCtrl->received_CAN->byte7;

    if (MainCtrl->is_receivedCAN == true){
        MainCtrl->is_receivedCAN = false;
        MainCtrl->sts_LED_ROS_SEND = true;
        // CSerial.sendPacket(data_recieve_can_serial);

        if (CSerial.sendPacket(data_recieve_can_serial) == false){
            Serial.println("send false");
        }
    }

    if (CSerial.readPacket(data_send_can_serial)){
        MainCtrl->send_CAN->id = data_send_can_serial.id;
        MainCtrl->send_CAN->byte0 = data_send_can_serial.data[0];
        MainCtrl->send_CAN->byte1 = data_send_can_serial.data[1];
        MainCtrl->send_CAN->byte2 = data_send_can_serial.data[2];
        MainCtrl->send_CAN->byte3 = data_send_can_serial.data[3];
        MainCtrl->send_CAN->byte4 = data_send_can_serial.data[4];
        MainCtrl->send_CAN->byte5 = data_send_can_serial.data[5];
        MainCtrl->send_CAN->byte6 = data_send_can_serial.data[6];
        MainCtrl->send_CAN->byte7 = data_send_can_serial.data[7];
        MainCtrl->is_sendCAN = true;
        MainCtrl->sts_LED_ROS_RECEIVED = true;
    }

    MainCtrl->led_loop();
    delay(1);
}

void create_task() {   
    xTaskCreatePinnedToCore(
        CanHandle_Run, 	/* Task function. */
        "Task_Run_CAN", /* name of task. */
        10000,          /* Stack size of task */
        NULL,         	/* parameter of the task */
        1,            	/* priority of the task */
        &Task_Run_CAN,  /* Task handle to keep track of created task */
        1);           	/* pin task to core 0 */              
}