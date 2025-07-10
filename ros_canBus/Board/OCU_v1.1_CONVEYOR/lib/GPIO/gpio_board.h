#ifndef GPIO_BOARD
#define GPIO_BOARD

#include "Arduino.h"

class GPIOBoard
{
    private:
        uint8_t pin;
        unsigned long timeStart_sensor;
        int timeCheck_sensor = 150;

    public:
        GPIOBoard(uint8_t _pin, uint8_t _mode){
            pin = _pin;
            pinMode(pin, _mode);
        }

        void init(){
            pinMode(pin, INPUT_PULLUP);
        }

        bool ReadStatus(){
            return digitalRead(pin);
        }

        bool GetStatus(bool _statusRight){
            if (digitalRead(pin) == _statusRight){
                ;
            }else{
                timeStart_sensor = millis();
            }

            if (millis() - timeStart_sensor > timeCheck_sensor){
                return 1;
            }else{
                return 0;
            }
        }
};

#endif