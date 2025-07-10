#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "CAN_manager.h"
#include "OCU_controller.h"
#include "WiFi.h"

TaskHandle_t Task0;

CAN_manager* OCU_CAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
OCU_controller* OCU_Ctrl = new OCU_controller(OCU_CAN);

void Task0_code( void * pvParameters ){
  for(;;){
    OCU_Ctrl->OCU_CAN_Receive();
    vTaskDelay(2);
  }
}

void create_task() {
  xTaskCreatePinnedToCore(
                    Task0_code,   /* Task function. */
                    "Task0",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task0,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */
}

void setup() {
  WiFi.mode(WIFI_OFF);
  delay(100);
  OCU_CAN->CAN_prepare();
  delay(50);
  OCU_Ctrl->setupBegin();
  btStop();
  delay(50);
  create_task();
  delay(100);
}

void loop() {
  OCU_Ctrl->OCU_loop();
  // OCU_Ctrl->Test1();
  
  // OC_Ctrl->debug();
}