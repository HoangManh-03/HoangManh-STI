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

// --------------------------
#define ID_RTC 0x01
#define ID_MC  0x02

#define FREQUENCY_sendCAN 10
#define FREQUENCY_control 15
// --------------------------

#define alampRead_1     35
#define analogOut_1     25
#define direction_1     4 // - PCF
#define enable_1        7 // - PCF
#define run_1           6 // - PCF

#define alampRead_2     34
#define analogOut_2     26
#define direction_2     5 // - PCF
#define enable_2        27
#define run_2           32

#define resetAlamp      0 // - PCF

#define EMG             36
#define SENSOR_1BIT     39
// ------------ ENCODER 
#define MOTOR1_ENCODER_A 18
#define MOTOR1_ENCODER_B 23 

#define MOTOR2_ENCODER_A 19
#define MOTOR2_ENCODER_B 33

#define SPEED_MOTOR_1   14
#define SPEED_MOTOR_2   13

// --------------------------
#define is_revert1 1 // 
#define is_revert2 1 // 

// --------------------------
#define PCF_address 0x20 // 0x38
#define PCF_SDA 21
#define PCF_SCL 22

// --------------------------
#define sendByte_alamp1 0
#define sendByte_alamp2 1
#define sendByte_dir1   2
#define sendByte_dir2   3
#define sendByte_speed1 4
#define sendByte_speed2 5
#define sendByte_mode   6
#define sendByte_EMG    7

#define receByte_id     0
#define receByte_dir1   1
#define receByte_dir2   2
#define receByte_speed1 3
#define receByte_speed2 4
#define receByte_mode   5
#define receByte_reset  6
#define receByte_       7
#endif