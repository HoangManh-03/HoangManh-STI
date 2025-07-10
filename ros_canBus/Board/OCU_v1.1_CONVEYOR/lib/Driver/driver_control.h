#ifndef DRIVER_CONTROL
#define DRIVER_CONTROL

#include "Arduino.h"

enum processStatus
{
    MODE_RESET = 0,

    // Mode lift
    MODE_LIFT_START = 1,
    MODE_LIFT_RUNNING = 2,
    MODE_LIFT_ERROR = 3,
    MODE_LIFT_TIMEOUT = 4,
    MODE_LIFT_DONE = 5,

    // Mode Conveyor Transfer
    MODE_CONVEYOR_TRANSFER_START = 6,
    MODE_CONVEYOR_TRANSFER_RUNNING = 7,
    MODE_CONVEYOR_TRANSFER_ERROR = 8,
    MODE_CONVEYOR_TRANSFER_TIMEOUT = 9,
    MODE_CONVEYOR_TRANSFER_DONE = 10,

    // Mode Conveyor Recieve
    MODE_CONVEYOR_RECIEVE_START = 11,
    MODE_CONVEYOR_RECIEVE_RUNNING = 12,
    MODE_CONVEYOR_RECIEVE_ERROR = 13,
    MODE_CONVEYOR_RECIEVE_TIMEOUT = 14,
    MODE_CONVEYOR_RECIEVE_DONE = 15,

    // MODE NORMAL
    MODE_NORMAL_START = 16,
    MODE_NORMAL_RUNNING = 17
};

class DriverBoard
{
    private:
        uint8_t CTR_PWM_H;
        uint8_t CTR_PWM_L;
        uint8_t CTR_ENABLE;
        uint8_t channelPWM_H;
        uint8_t channelPWM_L;

        int timeCheckError_modeLift = 40000; // ms - Do thoi gian loi.
        int timeCheckError_modeConveyor = 40000; // ms - Do thoi gian loi.
        int timeCheckSensor = 150; // ms - Do thoi gian cam bien tac dong.

        int speed_control = 0;
        processStatus currentStatus;

        int step_modeLift = 0;
        int step_modeConveyorTransfer = 0;
        int step_modeConveyorRecieve = 0;
        int step_modeNormal = 0;

        uint8_t reach_speed = 0;
        int saveSpeedNormal = 0;
        unsigned long timeStart_controlDriver = 0;  	// Lưu thời gian bắt đầu.


    public:
        DriverBoard(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

        int GetSpeed(){ return speed_control;}
        processStatus GetStatus(){ return currentStatus;}

        void SetCommandStatus(processStatus _stt){
            currentStatus = _stt;
        }

        void SetupBegin();

        /**
         * @brief Điều khiển Driver chế độ như bàn nâng, tay gạt.
         * 
         * Hàm này nhận các tham số như tín hiệu cảm biến giới hạn, chiều quay
         * 
         * @param stt_sensor_stop cảm biến giới hạn
         * @param dir Chiều quay (0 hoặc 1)
         * 
         * @note Quy định mức 1 là tích cực, 0 là không tích cực
         */
        void ControlDriver_modeLift(uint8_t, uint8_t);

        /**
         * @brief Điều khiển Driver chế độ như băng tải chuyển hàng.
         * 
         * Hàm này nhận các tham số như tín hiệu cảm biến giới hạn,chiều quay, tốc độ
         * 
         * @param stt_sensor_behind Cảm biến giới hạn phía sau (0 hoặc 1)
         * @param stt_sensor_ahead Cảm biến giới hạn phía trước (0 hoặc 1)
         * @param dir Chiều quay (0 hoặc 1)
         * @param speed Tốc độ (0-100%)
         * 
         * @note Quy định mức 1 là tích cực, 0 là không tích cực
         */
        void ControlDriver_modeConveyor_transfer(uint8_t, uint8_t, uint8_t, uint8_t);

         /**
         * @brief Điều khiển Driver chế độ như băng tải nhận hàng.
         * 
         * Hàm này nhận các tham số như tín hiệu cảm biến giới hạn,chiều quay, tốc độ
         * 
         * @param stt_sensor_behind Cảm biến giới hạn phía sau (0 hoặc 1)
         * @param stt_sensor_ahead Cảm biến giới hạn phía trước (0 hoặc 1)
         * @param dir Chiều quay (0 hoặc 1)
         * @param speed Tốc độ (0-100%)
         * 
         * @note Quy định mức 1 là tích cực, 0 là không tích cực
         */
        void ControlDriver_modeConveyor_recieve(uint8_t, uint8_t, uint8_t, uint8_t);

         /**
         * @brief Điều khiển Driver với điều kiện dừng
         * 
         * Hàm này nhận các tham số như tín hiệu cảm biến giới hạn, chiều quay, tốc độ
         * 
         * @param dir Chiều quay (0 hoặc 1)
         * @param speed Tốc độ (0-100%)
         * @param stt_sensor_stop Cảm biến xác định dừng (0 hoặc 1)
         * 
         * @note Quy định mức 1 là tích cực, 0 là không tích cực
         */
        void ControlDriver_modeNormal(uint8_t, uint8_t, uint8_t);

         /**
         * @brief Dừng driver và reset các biến
         */
        void StopAndReset();

        ~DriverBoard();
};

#endif