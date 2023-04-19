#include <Arduino.h>
#include "CAN_manager.h"
#include "Main_controller.h"

#include "ros.h"
#include "ros/time.h"
#include "sti_msgs/POWER_info.h"
#include "sti_msgs/POWER_request.h"

#include "sti_msgs/Lift_status.h"
#include "sti_msgs/Lift_control.h"

#include "sti_msgs/HC_info.h"
#include "sti_msgs/HC_request.h"

#define FREQUENCY_PUB_POWER_INFO 4
#define FREQUENCY_PUB_OC_INFO    4
#define FREQUENCY_PUB_HC_INFO    24

void create_task();
void POWER_controlCallback(const sti_msgs::POWER_request &data);
void OC_controlCallback(const sti_msgs::Lift_control &data);
void HC_Callback(const sti_msgs::HC_request &data);
void HC_fieldCallback(const std_msgs::Int8 &data);

ros::NodeHandle nodeHandle;
sti_msgs::POWER_info power_info;
sti_msgs::POWER_request power_request;

sti_msgs::Lift_status lift_status;
sti_msgs::Lift_control lift_control;

sti_msgs::HC_info hc_info;
sti_msgs::HC_request hc_request;   
std_msgs::Int8 hc_fieldRequest;

ros::Publisher power_pub("POWER_info", &power_info);         	
ros::Subscriber<sti_msgs::POWER_request> power_sub("POWER_request", POWER_controlCallback);

ros::Publisher lift_pub("lift_status", &lift_status);
ros::Subscriber<sti_msgs::Lift_control> lift_sub("lift_control", OC_controlCallback);

ros::Publisher hc_pub("HC_info", &hc_info);       	
ros::Subscriber<sti_msgs::HC_request> hc_sub("HC_request", HC_Callback);
ros::Subscriber<std_msgs::Int8> hc_subSelectField("HC_fieldRequest", HC_fieldCallback);

unsigned long saveTime_pub_main = 0;
unsigned long saveTime_pub_oc = 0;
unsigned long saveTime_pub_hc = 0;
unsigned long debugTime = 0;

int sts_OC;
int sts_HC;
int sts_Main;

TaskHandle_t Task_Receive_CAN; // Task_Receive_CAN
CAN_manager* MainCAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
Main_controller* MainCtrl = new Main_controller(MainCAN);

void POWER_controlCallback(const sti_msgs::POWER_request &data)
{
    power_request = data; 
} 

void OC_controlCallback(const sti_msgs::Lift_control &data)
{
    lift_control = data;
} 

void HC_Callback(const sti_msgs::HC_request &data)
{
    hc_request = data; 
}

void HC_fieldCallback(const std_msgs::Int8 &data)
{
    // hc_fieldRequest = data;
    // digitalWrite(selectFieldPin , data.data);
}

void CanHandle_Receive( void* pvParameters ){
    for(;;){
        MainCtrl->CANReceiveHandle();
        vTaskDelay(2);
    }
}

bool communication_check(){
	if (!nodeHandle.connected()){ // Mất kết nối với ROS -> phát yêu cầu dừng động cơ -> Restart Esp.
		delay(200);
        ESP.restart();
		return 1;
	}	
	return 0;
}

void setup() {
	MainCtrl->led_start();
    MainCtrl->init_main();
    delay(500);
    // - ROS
    nodeHandle.initNode();
    nodeHandle.getHardware()->setBaud(57600); 
    delay(10);
    nodeHandle.advertise(power_pub);
    nodeHandle.subscribe(power_sub);
    delay(10);
    nodeHandle.advertise(lift_pub);
	nodeHandle.subscribe(lift_sub);
    delay(10);
    nodeHandle.advertise(hc_pub);
    nodeHandle.subscribe(hc_sub);
    nodeHandle.subscribe(hc_subSelectField);
    delay(10);
    // -
    MainCAN->CAN_prepare();
    delay(6000);

    create_task();
    // - ROS
    while (!nodeHandle.connected())
    {   
        delay(10);
        nodeHandle.spinOnce();
    }
}

void loop() {
	communication_check();
    /* -- -- -- -- Main UPDATE -- -- -- -- */
    if (MainCtrl->stsRev_CAN == 0 || MainCtrl->stsRev_CAN_Main == 0){ // - LOST CAN HC
        sts_Main = -1;
    }else{ // - OK
        sts_Main = 0;
    }
	
    power_info.voltages_analog.data = MainCtrl->bytes_to_int(MainCtrl->Main_status->byte0_analogVoltage, MainCtrl->Main_status->byte1_analogVoltage);
    power_info.charge_analog.data   = MainCtrl->bytes_to_int(MainCtrl->Main_status->byte0_analogCurrent, MainCtrl->Main_status->byte1_analogCurrent);
    power_info.stsButton_reset.data = MainCtrl->Main_status->stsButton_reset;
    power_info.stsButton_power.data = MainCtrl->Main_status->stsButton_power;
    power_info.EMC_status.data 		= MainCtrl->Main_status->EMG_status;
    power_info.CAN_status.data 		= MainCtrl->stsSend_CAN;

    // - Control main
    MainCtrl->Main_comd->sound_enb 	  = power_request.sound_on.data;
    MainCtrl->Main_comd->sound_type   = power_request.sound_type.data;
    MainCtrl->Main_comd->charge_write = power_request.charge.data;
    MainCtrl->Main_comd->EMG_write    = power_request.EMC_write.data;
    MainCtrl->Main_comd->EMG_reset    = power_request.EMC_reset.data;

    /* -- -- -- -- OC UPDATE -- -- -- -- */
    if (MainCtrl->stsRev_CAN == 0 || MainCtrl->stsRev_CAN_OC == 0){ // - LOST OC
        sts_OC = -2;
    }else{ // - OK
        if (MainCtrl->OC_status->error == 1) // - ERROR OC
            sts_OC = -1;
        else
            sts_OC = MainCtrl->OC_status->commandStatus;
    }

    lift_status.status.data 	= sts_OC;
    lift_status.sensorLift.data = MainCtrl->getBit_fromInt(MainCtrl->OC_status->sensorBit_status, BIT_SENSOR_Lift);
    lift_status.sensorUp.data   = MainCtrl->getBit_fromInt(MainCtrl->OC_status->sensorBit_status, BIT_SENSOR_UP);
    lift_status.sensorDown.data = MainCtrl->getBit_fromInt(MainCtrl->OC_status->sensorBit_status, BIT_SENSOR_DOWN);
    // -- 
    MainCtrl->OC_comd->command 	  = lift_control.control.data;
    MainCtrl->OC_comd->resetError = lift_control.reset.data;

    /* -- -- -- -- HC UPDATE -- -- -- -- */
    if (MainCtrl->stsRev_CAN == 0 || MainCtrl->stsRev_CAN_HC == 0){ // - LOST CAN HC
        sts_HC = -1;
    }else{ // - OK
        sts_HC = 0;
    }
    hc_info.status.data 		  = sts_HC;
    hc_info.zone_sick_ahead.data  = MainCtrl->HC_status->zone_sick_ahead;
    hc_info.zone_sick_behind.data = MainCtrl->HC_status->zone_sick_behind;
    hc_info.vacham.data 		  = MainCtrl->HC_status->vacham;
    // --
    MainCtrl->HC_comd->RGB1 = hc_request.RBG1.data;
    MainCtrl->HC_comd->RGB2 = hc_request.RBG2.data;

	// -- RUN.....
	MainCtrl->loopMainCtr();
    /* -- -- -- -- ROS PUB -- -- -- -- */
    if ((millis() - saveTime_pub_main) > (1000 / FREQUENCY_PUB_POWER_INFO))
    {   
        saveTime_pub_main = millis();
        power_pub.publish(&power_info); 
    }

    if ((millis() - saveTime_pub_oc) > (1000 / FREQUENCY_PUB_OC_INFO))
    {   
        saveTime_pub_oc = millis();
        lift_pub.publish(&lift_status);
    }

    if ((millis() - saveTime_pub_hc) > (1000 / FREQUENCY_PUB_HC_INFO))
    {   
        saveTime_pub_hc = millis();
        hc_pub.publish(&hc_info);
    }

	nodeHandle.spinOnce();
}

void create_task() {   
    xTaskCreatePinnedToCore(
                CanHandle_Receive, /* Task function. */
                "Task_Receive_CAN",    /* name of task. */
                10000,        /* Stack size of task */
                NULL,         /* parameter of the task */
                1,            /* priority of the task */
                &Task_Receive_CAN,     /* Task handle to keep track of created task */
                1);           /* pin task to core 0 */              
}