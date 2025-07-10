#include <Arduino.h>
#include "maincontrol.h"
#include "WiFi.h"
// #include "getspeed.h"

MainControl Main;
TaskHandle_t Task0;

int rpm1 = 0;

void Task0_code( void * pvParameters ){
  for(;;){
    Main.CanRecevieHandle();
    vTaskDelay(4);
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
  LOG_BEGIN(115200);
  WiFi.mode(WIFI_OFF);
  btStop();
  // init_speed(SPEED_MOTOR_1, SPEED_MOTOR_2);
  Main.MainInit();
  create_task();
}

void loop(){
  // Main.CanRecevieHandle();
  // getRPM1(&rpm1);
  // getRPM2();

  Main.Run();
  // Main.Test_GPIO();
  // delayMicroseconds(70);
}
