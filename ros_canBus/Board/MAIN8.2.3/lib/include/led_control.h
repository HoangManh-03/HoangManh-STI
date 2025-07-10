#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <FastLED.h>
#include "hardwareconfig.h"

#define NUMS_LED_EYE            14
#define NUMS_EYE                4
#define NUMS_LED_TOTAL          (NUMS_LED_EYE * NUMS_EYE)

#define BRIGHTNESS              255
#define LED_TYPE                WS2812B
#define COLOR_ORDER             GRB

enum LedEffect
{
    L_OFF_LED                = 0, /* tắt led */
	L_MOVE_AUTO              = 1, /* agv di chuyển bình thường */
    L_TURNLEFT               = 2, /* agv rẽ trái */
    L_TURNRIGHT              = 3, /* agv rẽ phải */
    L_PARKING                = 4, /* agv di chuyển vào vị trí đặc biệt (lùi lấy hàng, trả hàng băng tải) */
    L_COLLISION_DETECTION    = 5, /* agv gặp vật cản */
    L_LIFTING                = 6, /* agv thực hiện nhiệm vụ (nâng/hạ hàng,...) */
    L_WARNING                = 7, /* agv cảnh báo */
    L_ERROR                  = 8, /* agv báo lỗi */
    L_CHARGE                 = 9, /* agv báo có lệnh sạc */
    L_CHARGING               = 10, /* agv báo đang sạc (có dòng sạc) */
    L_VOL_BATTERIES          = 11, /* agv hiển thị mức pin */
    L_WAITING_START_UP       = 12, /* chờ hệ thống */
    L_BY_HAND                = 13,
};

class Led_controller
{
    private:
        CRGB leds[NUMS_LED_TOTAL];
        unsigned long pre_led_time = 0;
        unsigned long pre_led_time_test = 0;
        uint8_t gHue0 = 0;
        uint8_t gHue1 = 0;
        uint8_t gHue2 = 0;
        uint8_t c_status = 0;
        uint8_t t_status = 1;

        uint8_t value_led1 = 0;
        bool flag_led1 = false;

    public:
        uint8_t led_request = L_WAITING_START_UP;
        uint8_t led_now;
        bool is_data = false;
        uint8_t level_voltage = 3;

        void fadeall(int scale) { for(int i = 0; i < NUMS_LED_TOTAL; i++) { leds[i].nscale8(scale); } } // - 250

        void init(){
            FastLED.addLeds<LED_TYPE, PIN_LED, COLOR_ORDER>(leds, NUMS_LED_TOTAL);
            FastLED.setBrightness(BRIGHTNESS); // - 84
            effect_off();
            effect_init(20);
        }

        void effect_init(int wait){
            // red
            for (int i = 0; i < 255; i+=5) {
                for (int j = 0; j < NUMS_LED_EYE; j++) {
                leds[j].fadeToBlackBy(5);
                leds[j].r = i;
                }
                FastLED.show();
                delay(wait);
            }
            // yellow
            for (int i = 0; i < 255; i+=5) {
                for (int j = NUMS_LED_EYE; j < NUMS_LED_EYE*2; j++) {
                leds[j].fadeToBlackBy(5);
                leds[j].r = i;
                leds[j].g = i;
                }
                FastLED.show();
                delay(wait);
            }
            // green
            for (int i = 0; i < 255; i+=5) {
                for (int j = NUMS_LED_EYE*2; j < NUMS_LED_EYE*3; j++) {
                leds[j].fadeToBlackBy(5);
                leds[j].g = i;
                }
                FastLED.show();
                delay(wait);
            }
            // blue
            for (int i = 0; i < 255; i+=5) {
                for (int j = NUMS_LED_EYE*3; j < NUMS_LED_TOTAL; j++) {
                leds[j].fadeToBlackBy(5);
                leds[j].b = i;
                }
                FastLED.show();
                delay(wait);
            }

            delay(1000);

            // red
            for (int i = 255; i >= 0; i-=5) {
                for (int j = 0; j < NUMS_LED_EYE; j++) {
                leds[j].fadeToBlackBy(5);
                leds[j].r = i;
                }
                FastLED.show();
                delay(wait);
            }

            // yellow
            for (int i = 255; i >= 0; i-=5) {
                for (int j = NUMS_LED_EYE; j < NUMS_LED_EYE*2; j++) {
                leds[j].fadeToBlackBy(5);
                leds[j].r = i;
                leds[j].g = i;
                }
                FastLED.show();
                delay(wait);
            }

            // green
            for (int i = 255; i >= 0; i-=5) {
                for (int j = NUMS_LED_EYE; j < NUMS_LED_EYE*3; j++) {
                leds[j].fadeToBlackBy(5);
                leds[j].g = i;
                }
                FastLED.show();
                delay(wait);
            }

            // blue
            for (int i = 255; i >= 0; i-=5) {
                for (int j = NUMS_LED_EYE*3; j < NUMS_LED_TOTAL; j++) {
                leds[j].fadeToBlackBy(5);
                leds[j].b = i;
                }
                FastLED.show();
                delay(wait);
            }
        }
        /* tat led */
        void effect_off(){
            fill_solid(leds, NUMS_LED_TOTAL, CRGB::Black);
            FastLED.show();
        }

        void setAllLEDs(CRGB color) {
            for(int i = 0; i < NUMS_LED_TOTAL; i++) {
                leds[i] = color;
            }
        }

        /**/
        void effect_normal(int time_wait, CRGB color){
            if (millis() - pre_led_time > time_wait){
                pre_led_time = millis();
                
                if (value_led1 >= 255){
                    flag_led1 = false;
                }
                else if (value_led1 <= 0){
                    flag_led1 = true;
                }

                if(flag_led1){
                    value_led1 += 5;
                }
                else{
                    value_led1 -= 5;
                }

                for (int j = 0; j < NUMS_LED_TOTAL; j++) {
                    leds[j].fadeToBlackBy(5);
                    leds[j].g = value_led1;
                }
                FastLED.show();
                delay(time_wait);
            }
        }

        /**/
        void effect_turn_left(int time_wait, CRGB color){
            if (millis() - pre_led_time > time_wait){
                if (c_status == 0){
                    c_status = 1;
                }else{
                    c_status = 0;
                }
                pre_led_time = millis();
            } 

            if (c_status == 1){
                for (int i = 0; i < NUMS_LED_TOTAL; i++){
                    if (i < NUMS_LED_EYE || i > NUMS_LED_EYE * 3 - 2){
                        leds[i] = color;
                    }
                    else{
                        leds[i] = CRGB::Black;
                    }
                }
                FastLED.show();
            }
            else{
                effect_off();
            }
        }

        /**/
        void effect_turn_right(int time_wait, CRGB color){
            if (millis() - pre_led_time > time_wait){
                if (c_status == 0){
                    c_status = 1;
                }else{
                    c_status = 0;
                }
                pre_led_time = millis();
            } 

            if (c_status == 1){
                for (int i = 0; i < NUMS_LED_TOTAL; i++){
                    if (i < NUMS_LED_EYE || i > NUMS_LED_EYE * 3 - 2){
                        leds[i] = CRGB::Black;
                    }
                    else{
                        leds[i] = color;
                    }
                }
                FastLED.show();
            }
            else{
                effect_off();
            }
        }

        /**/
        void effect_parking(int time_wait, CRGB color){
            if (millis() - pre_led_time > time_wait){
                if (c_status == 0){
                    c_status = 1;
                }else{
                    c_status = 0;
                }
                pre_led_time = millis();
            } 

            if (c_status == 1){
                fill_solid(leds, NUMS_LED_TOTAL, color);
                FastLED.show();
            }
            else{
                effect_off();
            }
        }

        /**/
        void effect_coll_detec(int time_wait, CRGB color){
            if (millis() - pre_led_time > time_wait){
                if (c_status == 0){
                    c_status = 1;
                }else{
                    c_status = 0;
                }
                pre_led_time = millis();
            } 

            if (c_status == 1){
                fill_solid(leds, NUMS_LED_TOTAL, color);
                FastLED.show();
            }
            else{
                effect_off();
            }
        }

        /**/
        void effect_lifting(int time_wait, CRGB color){
            if (millis() - pre_led_time > time_wait){
                if (c_status == 0){
                    c_status = 1;
                }else{
                    c_status = 0;
                }
                pre_led_time = millis();
            }
            
            for (int i = 0; i < NUMS_LED_TOTAL; i++){
                if (i%2==c_status){
                    leds[i] = color;
                }
                else{
                    leds[i] = CRGB::Black;
                }
            }
            FastLED.show();
        }

        /**/
        void effect_warning(int time_wait, CRGB color){
            if (millis() - pre_led_time > time_wait){
                if (c_status == 0){
                    c_status = 1;
                }else{
                    c_status = 0;
                }
                pre_led_time = millis();
            } 

            if (c_status == 1){
                fill_solid(leds, NUMS_LED_TOTAL, color);
                FastLED.show();
            }
            else{
                effect_off();
            }
        }

        /**/
        void effect_error(int time_wait, CRGB color){
            if (millis() - pre_led_time > time_wait){
                if (c_status == 0){
                    c_status = 1;
                }else{
                    c_status = 0;
                }
                pre_led_time = millis();
            } 

            if (c_status == 1){
                fill_solid(leds, NUMS_LED_TOTAL, color);
                FastLED.show();
            }
            else{
                effect_off();
            }
        }

        /**/
        void effect_charge(int time_wait, CRGB color){
            if (millis() - pre_led_time > time_wait){
                if (c_status == 0){
                    c_status = 1;
                }else{
                    c_status = 0;
                }
                pre_led_time = millis();
            } 

            if (c_status == 1){
                fill_solid(leds, NUMS_LED_TOTAL, color);
                FastLED.show();
            }
            else{
                effect_off();
            }
        }

        /**/
        void effect_charging(){
            static uint16_t sPseudotime = 0;
            static uint16_t sLastMillis = 0;
            static uint16_t sHue16 = 0;
            
            uint8_t sat8 = beatsin88( 87, 220, 250);
            uint8_t brightdepth = beatsin88( 341, 96, 224);
            uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
            uint8_t msmultiplier = beatsin88(147, 23, 60);

            uint16_t hue16 = sHue16;//gHue * 256;
            uint16_t hueinc16 = beatsin88(113, 1, 3000);
            
            uint16_t ms = millis();
            uint16_t deltams = ms - sLastMillis ;
            sLastMillis  = ms;
            sPseudotime += deltams * msmultiplier;
            sHue16 += deltams * beatsin88( 400, 5,9);
            uint16_t brightnesstheta16 = sPseudotime;
            
            for( uint16_t i = 0 ; i < NUMS_LED_TOTAL; i ++) {
                hue16 += hueinc16;
                uint8_t hue8 = hue16 / 256;

                brightnesstheta16  += brightnessthetainc16;
                uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

                uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
                uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
                bri8 += (255 - brightdepth);
                
                CRGB newcolor = CHSV( hue8, sat8, bri8);
                
                uint16_t pixelnumber = i;
                pixelnumber = (NUMS_LED_TOTAL-1) - pixelnumber;
                
                nblend( leds[pixelnumber], newcolor, 64);
            }
            FastLED.show();
        }

        /**/
        void effect_volt_batteries(){
            ;
        }

        /**/
        void effect_waiting_start_up(int wait){
            uint8_t num_led_tri = 6;
            static uint8_t hue = 0;
            for(int i = 0; i < NUMS_LED_TOTAL; i++) {
                leds[i] = CHSV(hue++, 255, 255);
                FastLED.show(); 
                fadeall(250);
                delay(wait);
            } 
            for(int i = (NUMS_LED_TOTAL)-1; i >= 0; i--) {
                leds[i] = CHSV(hue++, 255, 255);
                FastLED.show();
                fadeall(250);
                delay(wait);
            }
        }

        void effect_by_hand(int time_wait){
            // Fade in
            for(int brightness = 0; brightness <= 255; brightness+=5) {
                setAllLEDs(CRGB(brightness, brightness, brightness));
                FastLED.show();
                delay(time_wait);
            }
            
            // Fade out
            for(int brightness = 255; brightness >= 0; brightness-=5) {
                setAllLEDs(CRGB(brightness, brightness, brightness));
                FastLED.show();
                delay(time_wait);
            }
        }

        void clearStrip() {
            for (int i = 0; i < NUMS_LED_TOTAL; i++) {
                leds[i] = CRGB::Black;  // Tắt đèn
            }
            FastLED.show();
        }

        // void turnSignalLeft(int time_wait, CRGB color) {
        //     clearStrip();
        //     for (int i = NUMS_LED_TOTAL - NUMS_LED_OFFSET_EYE - NUMS_LED_EYE; i < NUMS_LED_TOTAL - NUMS_LED_OFFSET_EYE; i++) {
        //         leds[i] = color; // CRGB(255, 165, 0);  // Màu cam
        //         FastLED.show();
        //         delay(time_wait);  // Thời gian delay giữa mỗi LED sáng
        //     }
        // }

        // void turnSignalRight(int time_wait, CRGB color) {
        //     clearStrip();
        //     for (int i = NUMS_LED_OFFSET_EYE + NUMS_LED_EYE; i > NUMS_LED_OFFSET_EYE; i--) {
        //         leds[i] = color; // CRGB(255, 165, 0);  // Màu cam
        //         FastLED.show();
        //         delay(time_wait);  // Thời gian delay giữa mỗi LED sáng
        //     }
        // }

        void fcontrolLed(uint8_t cmd_){
            if (is_data == true){
                led_request = cmd_;
                is_data = false;
            }

            if (led_now != led_request){
                effect_off();
                led_now = led_request;
            }

            if (led_now == L_OFF_LED){
                effect_off();
            }else if (led_now == L_MOVE_AUTO){
                effect_normal(50, CRGB::Green);
            }else if (led_now == L_TURNLEFT){
                effect_turn_left(300, CRGB::DarkOrange);
            }else if (led_now == L_TURNRIGHT){
                effect_turn_right(300, CRGB::DarkOrange);
            }else if (led_now == L_PARKING){
                effect_parking(500, CRGB::Aqua);
            }else if (led_now == L_COLLISION_DETECTION){
                effect_coll_detec(400, CRGB::Purple);
            }else if (led_now == L_LIFTING){
                effect_lifting(500, CRGB::Green);
            }else if (led_now == L_WARNING){
                effect_warning(400, CRGB::DarkOrange);
            }else if (led_now == L_ERROR){
                effect_error(400, CRGB::Red);
            }else if (led_now == L_CHARGE){
                effect_charge(500, CRGB::Yellow);
            }else if (led_now == L_CHARGING){
                effect_charging();
            }else if (led_now == L_VOL_BATTERIES){
                effect_volt_batteries();
            }else if (led_now == L_WAITING_START_UP){
                effect_waiting_start_up(30);
            }else{
                effect_by_hand(50);
            }
        }

        void testEffect(){
            if ((millis() - pre_led_time_test) > 10000){
                t_status = t_status + 1;
                if (t_status > 12){
                    t_status = 0;
                }
                pre_led_time_test = millis();
                Serial.println(t_status);
            }
            if (t_status == 0){
                effect_off();
            }else if (t_status == 1){
                effect_normal(300, CRGB::Green);
            }else if (t_status == 2){
                effect_turn_left(300, CRGB::Orange);
            }else if (t_status == 3){
                effect_turn_right(300, CRGB::Orange);
            }else if (t_status == 4){
                effect_parking(500, CRGB::Aqua);
            }else if (t_status == 5){
                effect_coll_detec(400, CRGB::Purple);
            }else if (t_status == 6){
                effect_lifting(500, CRGB::Green);
            }else if (t_status == 7){
                effect_warning(400, CRGB::Orange);
            }else if (t_status == 8){
                effect_error(400, CRGB::Red);
            }else if (t_status == 9){
                effect_charge(500, CRGB::Yellow);
            }else if (t_status == 10){
                effect_charging();
            }else if (t_status == 11){
                effect_volt_batteries();
            }else if (t_status == 12){
                effect_waiting_start_up(40);
            }else{
                
            }
        }
        
};
#endif