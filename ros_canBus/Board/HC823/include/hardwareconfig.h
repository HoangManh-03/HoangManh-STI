/**
 * File : hardwareconfig.h
 * Version : 8.2.0
 * Date    : 17/06/2020
 * Author  : AnDX
 * Description :
 * 
 *******************************************************/

#ifndef __HARDWARECONFIG_H
#define __HARDWARECONFIG_H

#include <Arduino.h>
#include "HardwareSerial.h"
#include "CAN_config.h"
#include "PCF8574.h"

#define DEBUG_BY_SERIAL
#if defined(DEBUG_BY_SERIAL)
#define debugSerial Serial
#define LOG_BEGIN(BAUD) debugSerial.begin(BAUD);\
                        debugSerial.println("Log start!!  ε=ε=(づ￣ 3￣)づ  ε=ε=ε=┏(゜ロ゜;)┛ ")
#define LOG_MESS(...) debugSerial.println(__VA_ARGS__)
#define LOG_MESS_STRING(...) debugSerial.println((String) __VA_ARGS__)
#define LOG_NUM(NUM) debugSerial.println((String) #NUM + " = " + NUM)
#else
#define LOG_BEGIN(BAUD)
#define LOG_MESS(...)
#define LOG_MESS_STRING(...)
#define LOG_NUM(NUM)
#endif

#define ID_HC   0x03
#define ID_RTC  0x01

#define ID_RFID_HEAD    1
#define ID_RFID_BEHIND  2

#define BADERSHOCK_ISR  34
#define FREQUENCY_sendCAN 25

// #define DEBUG_SERIAL_ENABLE

#ifdef DEBUG_SERIAL_ENABLE 
#define hcDebugSrial Serial
#endif

// #define hcRFIDSerial Serial1
// #define hcRS485Dir      15

#define hcSickTopPin1In 3
#define hcSickTopPin2In 2
#define hcSickTopPin3In 1
#define hcSickTopPin4In 0
#define hcSickTopSelOut 33

#define hcSickBotPin1In 7
#define hcSickBotPin2In 6
#define hcSickBotPin3In 5
#define hcSickBotPin4In 4
#define hcSickBotSelOut 32

#define PCF8574_ADDRESS 0x20 // 0x38 0x20

// - Effect
#define red_blink        1
#define green_fading     2
#define green_blink      3
#define WhiteBlue_fading 4
#define yellow_fading    5
#define purple_blink     6
#define yellow_blink     7

// - Effect for DID
#define did_led_running     3
#define did_led_turnleft    4
#define did_led_turnright   5
#define did_led_error       6
#define did_led_waiting     7

#define FREQUENCY_controlLever 20

#define MODBUS_SERIAL Serial1
#define DE485_PIN 15
#define RO485_PIN 13
#define DI485_PIN 14

#define MODBUS_BUAD 9600
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1

#define MODBUS_TIMEOUT 100

#endif