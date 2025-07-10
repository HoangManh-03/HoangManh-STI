#include "driver_control.h"

DriverBoard::DriverBoard(uint8_t _enable_pin, uint8_t _pwm_H_pin, uint8_t _pwm_L_pin, uint8_t _channel_pwm_H, uint8_t _channel_pwm_L){
    CTR_ENABLE = _enable_pin;
    CTR_PWM_H = _pwm_H_pin;
    CTR_PWM_L = _pwm_L_pin;
    channelPWM_H = _channel_pwm_H;
    channelPWM_L = _channel_pwm_L;



    SetCommandStatus(MODE_RESET);
}

DriverBoard::~DriverBoard(){;}

void DriverBoard::SetupBegin(){
    pinMode(CTR_ENABLE, OUTPUT);
    pinMode(CTR_PWM_H,  OUTPUT);
    pinMode(CTR_PWM_L,  OUTPUT);

    digitalWrite(CTR_ENABLE, LOW);
    digitalWrite(CTR_PWM_H,  LOW);
    digitalWrite(CTR_PWM_L,  LOW);

    ledcSetup(channelPWM_H, 1000, 10);
    ledcSetup(channelPWM_L, 1000, 10);

    ledcAttachPin(CTR_PWM_H, channelPWM_H);
    ledcAttachPin(CTR_PWM_L, channelPWM_L);

    // Serial.printf("CTR_ENABLE = %d, CTR_PWM_H = %d, CTR_PWM_L = %d, channelPWM_H = %d, channelPWM_L = %d\n", CTR_ENABLE, CTR_PWM_H, CTR_PWM_L, channelPWM_H, channelPWM_L);
}

void DriverBoard::ControlDriver_modeLift(uint8_t _stt_sensor_stop, uint8_t _dir){
    uint8_t CTR_PWM_OFF = CTR_PWM_L;
    uint8_t channelPWM = channelPWM_H;

    if (_dir == 1){
        CTR_PWM_OFF = CTR_PWM_H;
        channelPWM = channelPWM_L;
    }

    if (step_modeLift == 0){
        timeStart_controlDriver = millis();
        digitalWrite(CTR_ENABLE, HIGH);
		digitalWrite(CTR_PWM_OFF, LOW);
		speed_control = 0;
		ledcWrite(channelPWM, speed_control);
		step_modeLift = 1;
        SetCommandStatus(MODE_LIFT_START);
        
    } else if (step_modeLift == 1){
        if (_stt_sensor_stop == 1){ // Bắt được cảm biến giới hạn.
			step_modeLift = 3;
		}

        if (speed_control >= 1022){ // Đạt tốc độ giới hạn.
            step_modeLift = 2;
            speed_control = 1022;
            ledcWrite(channelPWM, speed_control);
        }else{						// Tăng dần tốc độ.
            speed_control += 20;
            ledcWrite(channelPWM, speed_control);
            delay(1);
        }

        SetCommandStatus(MODE_LIFT_RUNNING);

    } else if (step_modeLift == 2){
        if(millis() - timeStart_controlDriver > timeCheckError_modeLift){
			speed_control = 0;
			ledcWrite(channelPWM, speed_control);
            SetCommandStatus(MODE_LIFT_ERROR);
		}

		if (_stt_sensor_stop == 1){ // Bắt được cảm biến giới hạn.
			speed_control = 0;
			step_modeLift = 3;
		}

    } else if (step_modeLift == 3){
        speed_control -= 600;
        ledcWrite(channelPWM, speed_control);

		if(speed_control <= 0){
			speed_control = 0;
			step_modeLift = 4;
			digitalWrite(CTR_ENABLE, LOW);
		}

    } else if (step_modeLift == 4){ // Hoàn thành.	
        SetCommandStatus(MODE_LIFT_DONE);
	}
}

void DriverBoard::ControlDriver_modeConveyor_transfer(uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed){
    uint8_t CTR_PWM_OFF = CTR_PWM_L;
    uint8_t CTR_PWM_ON = CTR_PWM_H;
    uint8_t channelPWM = channelPWM_H;

    if (_dir == 1){
        CTR_PWM_OFF = CTR_PWM_H;
        CTR_PWM_ON = CTR_PWM_L;
        channelPWM = channelPWM_L;
    }

    int speedMax = 0;
    if (_speed >= 100){
        speedMax = 1024;
    }else if (_speed < 50){
        speedMax = 600;
    }else{
        speedMax = 1024*(_speed/100.);
    }

    // speedMax = 900;
    // - Quay thuan
    if (step_modeConveyorTransfer == 0){ // điều khiển chiều.
        timeStart_controlDriver = millis();
        speed_control = 0;
        ledcWrite(channelPWM_H, speed_control);
        ledcWrite(channelPWM_L, speed_control);
        digitalWrite(CTR_PWM_H, LOW);
        digitalWrite(CTR_PWM_L, LOW);
        digitalWrite(CTR_ENABLE, HIGH);
        step_modeConveyorTransfer = 1;
        SetCommandStatus(MODE_CONVEYOR_TRANSFER_START);

    }else if (step_modeConveyorTransfer == 1){ // - Tăng dần tốc độ.		
        if (_stt_sensor_behind == 1 && _stt_sensor_ahead == 1){ // - Thoát được cảm biến giới hạn Trước và Sau.
            if (speed_control == 0){
                step_modeConveyorTransfer = 5;
            }else{
                step_modeConveyorTransfer = 4;
            }
        }
        // -
        speed_control += 6;
        if (speed_control >= speedMax){ // - Đạt tốc độ giới hạn.
            step_modeConveyorTransfer = 2;
            speed_control = speedMax;
        }

        SetCommandStatus(MODE_CONVEYOR_TRANSFER_RUNNING);

    }else if (step_modeConveyorTransfer == 2){ // Đợi bắt được cảm biến.
        if (millis() - timeStart_controlDriver > timeCheckError_modeConveyor){ // - Kiểm tra lỗi theo thời gian vận hành.
            step_modeConveyorTransfer = 6;
        }

        if (_stt_sensor_behind == 1 && _stt_sensor_ahead == 1){ // - Thoát được cảm biến giới hạn Trước và Sau.
            step_modeConveyorTransfer = 3;
            timeStart_controlDriver = millis();
        }

    }else if (step_modeConveyorTransfer == 3){ // Chay them 1 thoi gian.
        if (millis() - timeStart_controlDriver > 1000){ // -
            step_modeConveyorTransfer = 4;
        }

    }else if (step_modeConveyorTransfer == 4){ // Giảm dần tốc độ về 0.
        speed_control -= 6;
        if(speed_control <= 0){
            speed_control = 0;
            step_modeConveyorTransfer = 5;
        }

    }else if (step_modeConveyorTransfer == 5){ // - Hoàn thành.	
        speed_control = 0;
        ledcWrite(channelPWM, speed_control);
        digitalWrite(CTR_PWM_ON, LOW);
        digitalWrite(CTR_ENABLE, LOW);

        SetCommandStatus(MODE_CONVEYOR_TRANSFER_DONE);

    }else if (step_modeConveyorTransfer == 6){ // - ERROR - Giam dan toc do.
        speed_control -= 6;
        if(speed_control <= 0){
            speed_control = 0;
            step_modeConveyorTransfer = 7;
        }

    }else if (step_modeConveyorTransfer == 7){ // - ERROR - Stop
        speed_control = 0;
        ledcWrite(channelPWM, speed_control);
        digitalWrite(CTR_PWM_ON, LOW);
        digitalWrite(CTR_ENABLE, LOW);

        SetCommandStatus(MODE_CONVEYOR_TRANSFER_ERROR);
    }

    ledcWrite(channelPWM, speed_control);
}

void DriverBoard::ControlDriver_modeConveyor_recieve(uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed){
    uint8_t CTR_PWM_OFF = CTR_PWM_L;
    uint8_t CTR_PWM_ON = CTR_PWM_H;
    uint8_t channelPWM = channelPWM_H;

    if (_dir == 1){
        CTR_PWM_OFF = CTR_PWM_H;
        CTR_PWM_ON = CTR_PWM_L;
        channelPWM = channelPWM_L;
    }

    int speedMax = 0;
    if (_speed >= 100){
        speedMax = 1024;
    }else if (_speed < 50){
        speedMax = 600;
    }else{
        speedMax = 1024*(_speed/100.);
    }

    // speedMax = 900;
    // - Quay thuan
    if (step_modeConveyorRecieve == 0){ // điều khiển chiều.
        timeStart_controlDriver = millis();
        speed_control = 0;
        ledcWrite(channelPWM_H, speed_control);
        ledcWrite(channelPWM_L, speed_control);
        digitalWrite(CTR_PWM_H, LOW);
        digitalWrite(CTR_PWM_L, LOW);
        digitalWrite(CTR_ENABLE, HIGH);
        step_modeConveyorRecieve = 1;

        SetCommandStatus(MODE_CONVEYOR_RECIEVE_START);

    }else if (step_modeConveyorRecieve == 1){ // - Tăng dần tốc độ.		
        if (_stt_sensor_behind == 1){ // - Thoát được cảm biến giới hạn Trước và Sau.
            if (speed_control == 0){
                step_modeConveyorRecieve = 5;
            }else{
                step_modeConveyorRecieve = 4;
            }
        }
        // -
        speed_control += 6;
        if (speed_control >= speedMax){ // - Đạt tốc độ giới hạn.
            step_modeConveyorRecieve = 2;
            speed_control = speedMax;
        }

        SetCommandStatus(MODE_CONVEYOR_RECIEVE_RUNNING);

    }else if (step_modeConveyorRecieve == 2){ // Đợi bắt được cảm biến.
        if (millis() - timeStart_controlDriver > timeCheckError_modeConveyor){ // - Kiểm tra lỗi theo thời gian vận hành.
            step_modeConveyorRecieve = 6;
        }

        if (_stt_sensor_behind == 1){ // - Thoát được cảm biến giới hạn Trước và Sau.
            step_modeConveyorRecieve = 3;
            timeStart_controlDriver = millis();
        }

    }else if (step_modeConveyorRecieve == 3){ // Chay them 1 thoi gian.
        if (millis() - timeStart_controlDriver > 100){ // -
            step_modeConveyorRecieve = 4;
        }

    }else if (step_modeConveyorRecieve == 4){ // Giảm dần tốc độ về 0.
        speed_control -= 6;
        if(speed_control <= 0){
            speed_control = 0;
            step_modeConveyorRecieve = 5;
        }

    }else if (step_modeConveyorRecieve == 5){ // - Hoàn thành.	
        speed_control = 0;
        ledcWrite(channelPWM, speed_control);
        digitalWrite(CTR_PWM_ON, LOW);
        digitalWrite(CTR_ENABLE, LOW);

        SetCommandStatus(MODE_CONVEYOR_RECIEVE_DONE);

    }else if (step_modeConveyorRecieve == 6){ // - ERROR - Giam dan toc do.
        speed_control -= 6;
        if(speed_control <= 0){
            speed_control = 0;
            step_modeConveyorRecieve = 7;
        }

    }else if (step_modeConveyorRecieve == 7){ // - ERROR - Stop
        speed_control = 0;
        ledcWrite(channelPWM, speed_control);
        digitalWrite(CTR_PWM_ON, LOW);
        digitalWrite(CTR_ENABLE, LOW);

        SetCommandStatus(MODE_CONVEYOR_RECIEVE_ERROR);
    }

    ledcWrite(channelPWM, speed_control);
}

void DriverBoard::ControlDriver_modeNormal(uint8_t _dir, uint8_t _speed, uint8_t _sst_sensor_stop){
    uint8_t CTR_PWM_ON = CTR_PWM_H;
    uint8_t channelPWM = channelPWM_H;

    if (_dir == 1){
        CTR_PWM_ON = CTR_PWM_L;
        channelPWM = channelPWM_L;
    }

    int speedMax = 0;
    if (_speed >= 100)
        speedMax = 1023; 
    else if (_speed <= 20)
        speedMax = 0;
    else
        speedMax = 1023*(_speed/100.);

    if (saveSpeedNormal != speedMax){
        saveSpeedNormal = speedMax;
        reach_speed = 0;
    }

    if (step_modeNormal == 0){
        speed_control = 0;
        ledcWrite(channelPWM_H, speed_control);		
        ledcWrite(channelPWM_L, speed_control);
		digitalWrite(CTR_PWM_H, LOW);
        digitalWrite(CTR_PWM_L, LOW);
        digitalWrite(CTR_ENABLE, HIGH);
        step_modeNormal = 1;

        SetCommandStatus(MODE_NORMAL_START);

    } else if (step_modeNormal == 1){
        if (reach_speed == 0){
            if (speed_control < speedMax){
                speed_control += 6;
                if (speed_control >= speedMax){
                    speed_control = speedMax;
                    reach_speed = 1;
                }
            }
            else if (speed_control > speedMax){
                speed_control -= 6;
                if (speed_control <= speedMax){
                    speed_control = speedMax;
                    reach_speed = 1;
                }
            }
        }	

        SetCommandStatus(MODE_NORMAL_RUNNING);
    }

    // Serial.printf("CTR_PWM_ON = %d, channelPWM = %d, speed_control = %d\n", CTR_PWM_ON, channelPWM, speed_control);

    ledcWrite(channelPWM, speed_control);
}

void DriverBoard::StopAndReset(){
    speed_control = 0;
    digitalWrite(CTR_PWM_L, LOW);
    digitalWrite(CTR_PWM_H, LOW);
    ledcWrite(channelPWM_L, speed_control);
    ledcWrite(channelPWM_H, speed_control);
    digitalWrite(CTR_ENABLE, LOW);

    step_modeLift = 0;
    step_modeConveyorTransfer = 0;
    step_modeConveyorRecieve = 0;
    step_modeNormal = 0;

    reach_speed = 0;
    saveSpeedNormal = 0;

    SetCommandStatus(MODE_RESET);
}

