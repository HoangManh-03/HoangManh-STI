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

#define DEBUG_SERIAL_ENABLE

#ifdef DEBUG_SERIAL_ENABLE
    #define DebugSrial Serial 
#endif

#define ID_HC       0x01
#define ID_MAIN     0x02
#define ID_OC       0x03
#define ID_Convert  0x04

#define CAN_BAUD_SPEED  CAN_SPEED_125KBPS
#define CAN_TX          GPIO_NUM_5
#define CAN_RX          GPIO_NUM_4
#define CAN_FRAME       CAN_frame_std
#define CAN_ID          ID_OC
#define CAN_SEND_SIZE   8

#define CAN_TIMER       60

#define TIME_OUT        50000
#define SENSOR_1        13 //1
#define SENSOR_2        12 //1
#define SENSOR_3        15 //1
#define SENSOR_4        18 //1
#define SENSOR_5        21 //1
#define SENSOR_6        22 //1
#define SENSOR_7        23 //1
#define E1              19 //1
#define EN_LIFTER       14 //1
#define EN_CONVAYER     25 //1
#define LIFTER_H        26 //1 or 0
#define LIFTER_L        27 //0 or 1
#define CONVAYER_H      33 //1 or 0
#define CONVAYER_L      32 //0 or 1

#define CHECKING_RACK_SR    SENSOR_1
#define LIFTER_UP_SR        SENSOR_2
#define LIFTER_DOWN_SR      SENSOR_3

#endif