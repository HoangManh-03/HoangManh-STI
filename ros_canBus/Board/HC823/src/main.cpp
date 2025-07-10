#include <Arduino.h>
#include "maincontrol.h"
#include "WiFi.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

MainControl Main;
TaskHandle_t Task0;

void Task0_code( void * pvParameters ){
  for(;;){
    // Main.Send485();
    vTaskDelay(pdMS_TO_TICKS(2)); // Delay 1000ms (1 giây)
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
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  LOG_BEGIN(57600);
  WiFi.mode(WIFI_OFF);
  btStop();
  Main.MainInit();
  // create_task();
}

void loop(){
  Main.Run();
}
