#ifndef SOUND_CONTROLLER_H
#define SOUND_CONTROLLER_H

#include "hardwareconfig.h"

enum SoundType
{
    S_OFF                    = 0, /* tắt loa */
    S_INIT                   = 1, /* âm thanh khi khởi tạo*/
    S_WAITING_START_UP       = 2, /* chờ khởi động*/
	S_NORMAL                 = 3,  /* di chuyển bình thường*/
    S_TURNLEFT               = 4,  /* thực hiện rẽ trái */
    S_TURNRIGHT              = 5,  /* thực hiện rẽ phải */
    S_PARKING                = 6,  /* lùi vào kệ hoặc tinh chỉnh vào vị trí đặc biệt */
    S_COLLISION_DETECTION    = 7,  /* cảnh báo vướng vật cản*/
    S_DOING_JOB              = 8,  /* thưc hiện nhiệm vụ (nâng/hạ/băng tải)*/
    S_WARNING                = 9,  /* cảnh báo*/
    S_ERROR_EMG              = 10,  /* lỗi emg */
    S_ERROR_SYSTEM           = 11,  /* lỗi hệ thống*/
    S_CHARGE                 = 12,  /* sạc pin*/
    S_SPARE_1                = 13,  /* âm dự phòng 1*/
    S_SPARE_2                = 14,  /* âm dự phòng 2*/
    S_SPARE_3                = 15,  /* âm dự phòng 3*/
};

class Sound_controller
{
    public:
        uint8_t sound_request = S_WAITING_START_UP;
        uint8_t sound_now;
        bool is_data = false;

        void init(){
            pinMode(PIN_SOUND_SP, OUTPUT);
            digitalWrite(PIN_SOUND_SP, LOW);

            pinMode(PIN_SOUND_1, OUTPUT);
            pinMode(PIN_SOUND_2, OUTPUT);
            pinMode(PIN_SOUND_3, OUTPUT);
            pinMode(PIN_SOUND_4, OUTPUT);

            /* sound init */
            // digitalWrite(PIN_SOUND_1, LOW);
            // digitalWrite(PIN_SOUND_2, HIGH);
            // digitalWrite(PIN_SOUND_3, HIGH);
            // digitalWrite(PIN_SOUND_4, HIGH);

            digitalWrite(PIN_SOUND_1, HIGH);
            digitalWrite(PIN_SOUND_2, HIGH);
            digitalWrite(PIN_SOUND_3, HIGH);
            digitalWrite(PIN_SOUND_4, LOW);
        }

        void play_sound(uint8_t cmd_){
            if (is_data == true){
                sound_request = cmd_;
                is_data = false;
            }

            if (sound_now != sound_request){
                sound_now = sound_request;
            }

            uint8_t sound_invert_bit = ~sound_now;
            uint8_t is_play = 1;
            if (sound_now > 0 && sound_now <= S_SPARE_3){
                is_play = 0;
            }
            
            uint8_t bit1 = sound_invert_bit&1;
            uint8_t bit2 = (sound_invert_bit>>1)&1;
            uint8_t bit3 = (sound_invert_bit>>2)&1;
            uint8_t bit4 = (sound_invert_bit>>3)&1;

            digitalWrite(PIN_SOUND_SP, is_play);

            // digitalWrite(PIN_SOUND_1, bit1);
            // digitalWrite(PIN_SOUND_2, bit2);
            // digitalWrite(PIN_SOUND_3, bit3);
            // digitalWrite(PIN_SOUND_4, bit4);

            digitalWrite(PIN_SOUND_1, bit4);
            digitalWrite(PIN_SOUND_2, bit3);
            digitalWrite(PIN_SOUND_3, bit2);
            digitalWrite(PIN_SOUND_4, bit1);
        }

};
#endif