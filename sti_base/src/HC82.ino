/*
Name:  	STI HC82.ino
Author: Tran Anh Tuan   - 23/11/2020 
Info : Mach HC (Head Control ): 
    + led RBG
    + led Camera
    + va cham

*/

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "ros.h"
#include "ros/time.h"
#include "sti_msgs/HC_info.h"
#include "sti_msgs/HC_request.h"
#include "std_msgs/Int8.h"
#include "WiFi.h"

#include <PCF8574.h>
PCF8574 pcf(0x20); // 0x20 0x38
/*
SICK 2
24V | GND | P7 | P6 | P5 | p4 | IN1

SICK 1
24V | GND | P3 | P2 | P1 | p0 | IN2
*/

// led RGB1
int r1Pin = 23 ;
int g1Pin = 19 ;
int b1Pin = 18 ;
int r1Change = 0 ;
int g1Change = 1 ;
int b1Change = 2 ;

// led RGB1
int r2Pin = 27 ;
int g2Pin = 26 ;
int b2Pin = 25 ;
int r2Change = 7 ;
int g2Change = 8 ;
int b2Change = 9 ;

//va cham
int vachamPIn = 34 ;
int selectFieldPin = 33;

// - zone_sick_2 - PCF
int zone1_pin = 3;
int zone2_pin = 2;
int zone3_pin = 1;
int zone_sick = 0;
int d_zone1 = 0;

// - zone_sick_1 - PCF
int zone1s_pin = 6;
int zone2s_pin = 7;
int zone3s_pin = 5;
int zones_sick = 0;
int d_zone2 = 0;

// time 
unsigned long pre_led1_time = 0;
unsigned long pre_led2_time = 0;
unsigned long pre_stt_time = 0;

// -- Frequence
#define PUB_LED1_FREQUENCY 100
#define PUB_LED2_FREQUENCY 100
#define PUB_STT_FREQUENCY  24

// ----- Error-----
bool flag_requir_ESP_reset;
bool is_command_pc = 0;    

ros::NodeHandle nh;
sti_msgs::HC_info hc_info;
sti_msgs::HC_request hc_request;   
std_msgs::Int8 hc_fieldRequest;

void HC_Callback(const sti_msgs::HC_request &data)
{
    hc_request = data ; 
}

void HC_fieldCallback(const std_msgs::Int8 &data)
{
    // hc_fieldRequest = data;
    digitalWrite(selectFieldPin , data.data);
}

ros::Publisher hc_pub("HC_info", &hc_info);       	
ros::Subscriber<sti_msgs::HC_request> hc_sub("HC_request", HC_Callback);
ros::Subscriber<std_msgs::Int8> hc_subSelectField("HC_fieldRequest", HC_fieldCallback);

void led_init(int rPin,int gPin,int bPin,int rChange,int gChange,int bChange){
    ledcSetup(rChange,5000,8);
    ledcAttachPin(rPin,rChange);
  
    ledcSetup(gChange,5000,8);
    ledcAttachPin(gPin,gChange);

    ledcSetup(bChange,5000,8);
    ledcAttachPin(bPin,bChange);
}

void setColor2(uint8_t rVal,uint8_t gVal,uint8_t bVal)
{
    ledcWrite(r1Change,rVal);
    ledcWrite(g1Change,gVal);
    ledcWrite(b1Change,bVal);
}

void setColor1(uint8_t rVal,uint8_t gVal,uint8_t bVal)
{
    ledcWrite(r2Change,rVal);
    ledcWrite(g2Change,gVal);
    ledcWrite(b2Change,bVal);
}

void ledRGB1_old(int color)
{
    static uint8_t dimmer = 0;
    static bool flager = true;
    
    if ((millis() - pre_led1_time) > (1000 / PUB_LED1_FREQUENCY))
    {
		pre_led1_time = millis();
        if (dimmer >= 254){
            flager = false;
        }
        else if (dimmer <= 1){
            flager = true;
        }

        if(flager){
            dimmer ++;
        }
        else{
            dimmer --;
        }
        
        if (color == 1) setColor1(dimmer,0,0); // red
        if (color == 2) setColor1(0,dimmer,0); // green
        if (color == 3) setColor1(0,0,dimmer); // blue

    }
}

void ledRGB1(int color){

    static uint8_t dimmer = 0;
    static bool flager = true;

    if(color == 1){ // error : sang nhanh, nhay do 
        if ((millis() - pre_led1_time) > 1)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            setColor1(dimmer,0,0); // red
            setColor2(dimmer,0,0); // red
        }
    }
    if(color == 2){ // di chuyen : sang cham ,xanh la 
        if ((millis() - pre_led1_time) > 10)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            setColor1(0,dimmer,0); // green
            setColor2(0,dimmer,0); // green
        }
    }
    if(color == 3){ // tag : sang nhanh , xanh la 
        if ((millis() - pre_led1_time) < 200)
        {   
            setColor1(0,254,0); // green
            setColor2(0,254,0); // green
        }
        if (((millis() - pre_led1_time) > 200) && (millis() - pre_led1_time) < 300)
        {   
            setColor1(0,0,0); // green
            setColor2(0,0,0); // green

        }
        if ((millis() - pre_led1_time) > 300)
        {   
            pre_led1_time = millis();
        }


    }
    if(color == 4){ // nang, ha : sang cham , xanh duong 
        if ((millis() - pre_led1_time) > 10)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            setColor1(0,dimmer,dimmer); // g+b
            setColor2(0,dimmer,dimmer); // g+b
        }
    }
    if(color == 5){ // sac: sang cham , vang 
        if ((millis() - pre_led1_time) > 10)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            setColor1(dimmer,dimmer,0); // r + g
            setColor2(dimmer,dimmer,0); // r + g
        }
    }
    if(color == 6){ //
        if ((millis() - pre_led1_time) < 200)
        {   
            setColor1(250, 0, 250); // 
            setColor2(250, 0, 250); // 
        }
        if (((millis() - pre_led1_time) > 200) && (millis() - pre_led1_time) < 300)
        {   
            setColor1(0, 0, 0); // 
            setColor2(0, 0, 0); // 
        }
        if ((millis() - pre_led1_time) > 300)
        {   
            pre_led1_time = millis();
        }
    }
    if(color == 7){ //
        if ((millis() - pre_led1_time) < 200)
        {   
            setColor1(250, 250, 0); // 
            setColor2(250, 250, 0); // 
        }
        if (((millis() - pre_led1_time) > 200) && (millis() - pre_led1_time) < 300)
        {   
            setColor1(0, 0, 0); // 
            setColor2(0, 0, 0); // 
        }
        if ((millis() - pre_led1_time) > 300)
        {   
            pre_led1_time = millis();
        }
    }
    if(color == 8){ //
        if ((millis() - pre_led1_time) > 10)
        {
            pre_led1_time = millis();
            if (dimmer >= 254){
                flager = false;
            }
            else if (dimmer <= 1){
                flager = true;
            }

            if(flager){
                dimmer ++;
            }
            else{
                dimmer --;
            }

            setColor1(dimmer, 0, dimmer); // r + g
            setColor2(dimmer, 0, dimmer); // r + g
        }
    }
}

void ledRGB2(uint8_t dimmer)
{
    setColor2(dimmer,dimmer,dimmer);
}

// Kiểm tra kết nối với PC: Nếu sau thời gian T ko nhận được lệnh -> lỗi.
bool communication_check(){
	if (!nh.connected()){ // Mất kết nối với ROS -> Restart Esp.
		delay(200);
        ESP.restart();
		return 1;
	}	
	return 0;
}

void IRAM_ATTR isr() {
    hc_info.vacham.data = int(digitalRead(vachamPIn)) ;
    hc_pub.publish(&hc_info);
}

void setup()
{   
    // tat wifi , blutooth
    WiFi.mode(WIFI_OFF);
    btStop();

    nh.initNode();

    // pinMode(selectFieldPin, OUTPUT);
    pinMode(vachamPIn,1);
    attachInterrupt(vachamPIn, isr ,FALLING);

    nh.getHardware()->setBaud(57600); 

    nh.advertise(hc_pub);
    nh.subscribe(hc_sub);
    nh.subscribe(hc_subSelectField);

    // -- init PCF
    pcf.begin();
    digitalWrite(selectFieldPin , 1);

    //led init
    led_init(r1Pin,g1Pin,b1Pin,r1Change,g1Change,b1Change);
    led_init(r2Pin,g2Pin,b2Pin,r2Change,g2Change,b2Change);

    // setColor1(200,0,0); // red
    // setColor2(200,0,0); // red

	delay(50);
    while (!nh.connected())
    {   
        nh.spinOnce();
    }

    nh.loginfo("STI vietnam - HC 8.2 OK ('_') ");
}

void read_SICK(){
    bool z1t = 0;
    bool z2t = 0;
    bool z3t = 0;
    bool z1s = 0;
    bool z2s = 0;
    int value = 0;
    z1t = pcf.readButton(zone1_pin);
    z2t = pcf.readButton(zone2_pin);
    z3t = pcf.readButton(zone3_pin);
    zone_sick = z1t*100 + z2t*10 + z3t;
    if(z1t==1){
        d_zone1 = 1;
    }else if (z2t==1){
        d_zone1 = 2;
    }else if (z3t==1){
        d_zone1 = 3;
    }else{
        d_zone1 = 0;
    }
    
    z1s = pcf.readButton(zone1s_pin);
    z2s = pcf.readButton(zone2s_pin);
    zones_sick = z1s*10 + z2s;

    if(z1s==0){
        d_zone2 = 1;
    }else if (z2s==0){
        d_zone2 = 2;
    }else{
        d_zone2 = 0;
    }
}

void loop(){
    // kiem tra ket noi PC.
    communication_check();
    
    // LED  bao hieu
    ledRGB1(int(hc_request.RBG1.data));

    // LED camera 
    // ledRGB2(uint8_t(hc_request.RBG2.data));

    // vacham = 0 
    hc_info.vacham.data = int(digitalRead(vachamPIn)) ;

    if ((millis() - pre_stt_time) > (1000 / PUB_STT_FREQUENCY))
    {
		pre_stt_time = millis();
        read_SICK();
        hc_info.status.data = 1 ;
        hc_info.zone_sick_ahead.data = d_zone1;
        hc_info.zone_sick_behind.data = d_zone2;
        hc_pub.publish(&hc_info);
    }

	nh.spinOnce();
}


