#ifndef __GETSPEED_H
#define __GETSPEED_H

#include <Arduino.h>
#include "MedianFilter.h"

MedianFilter<float, 7> rpm1Filter;
MedianFilter<float, 7> rpm2Filter;

unsigned char M1_SP;
unsigned char M2_SP;

hw_timer_t * timer0 = NULL;  
hw_timer_t * timer1 = NULL;  

volatile unsigned long elapsedTime1 = 0;
volatile unsigned long lastPulseTime1 = 0;

volatile unsigned long elapsedTime2 = 0;
volatile unsigned long lastPulseTime2 = 0;

void IRAM_ATTR pulse_ISR_1() {
    unsigned long currentTime = micros(); // Thời gian hiện tại (μs)
    elapsedTime1 = long(currentTime - lastPulseTime1); // Thời gian giữa 2 xung
    lastPulseTime1 = currentTime;
}

void IRAM_ATTR pulse_ISR_2() {
    unsigned long currentTime = micros(); // Thời gian hiện tại (μs)
    elapsedTime2 = long(currentTime - lastPulseTime2); // Thời gian giữa 2 xung
    lastPulseTime2 = currentTime;
}

void init_speed(unsigned char m1_sp, unsigned char m2_sp) {
	M1_SP = m1_sp;
	M2_SP = m2_sp;

    pinMode(M1_SP, INPUT_PULLUP);
    // pinMode(M2_SP, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(M1_SP), pulse_ISR_1, FALLING);
    // attachInterrupt(digitalPinToInterrupt(M2_SP), pulse_ISR_2, FALLING);
}

void getRPM1(int * _rpm) {
    static unsigned long lastCheck = 0;

    if ((long)(micros() - lastPulseTime1) > 100000){
        rpm1Filter = MedianFilter<float, 7>();
        *_rpm = 0;
    }
    else
    {
        if (millis() - lastCheck > 20) {
            lastCheck = millis();

            noInterrupts();
            unsigned long intervalCopy = elapsedTime1;
            interrupts();

            if (intervalCopy > 0) {
                float T = intervalCopy / 1e6; // Chuyển từ micro giây sang giây
                float freq = 1.0 / T;
                float rpm = freq * 2; // Theo công thức

                if (rpm < 4000.){
                    rpm1Filter.addValue(rpm);
                    float filteredRPM1 = rpm1Filter.getMedian();
                    *_rpm = int(filteredRPM1); 
                }
            }
        }

    }
}

void getRPM2(int * _rpm) {
    static unsigned long lastCheck = 0;

    if ((long)(micros() - lastPulseTime2) > 100000){
        rpm2Filter = MedianFilter<float, 7>();
        *_rpm = 0;
    }
    else
    {
        if (millis() - lastCheck > 20) {
            lastCheck = millis();

            noInterrupts();
            unsigned long intervalCopy = elapsedTime2;
            interrupts();

            if (intervalCopy > 0) {
                float T = intervalCopy / 1e6; // Chuyển từ micro giây sang giây
                float freq = 1.0 / T;
                float rpm = freq * 2; // Theo công thức

                if (rpm < 4000.){
                    rpm1Filter.addValue(rpm);
                    float filteredRPM2 = rpm1Filter.getMedian();
                    *_rpm = int(filteredRPM2); 
                }
            }
        }

    }
}

#endif