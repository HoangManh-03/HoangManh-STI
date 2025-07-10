#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#define receive_all true
#define receive_one false

#include "Arduino.h"
#include "ESP32CAN.h"
#include "CAN_config.h"

#define CAN_CHECK_LOST_FREQ 10

class CAN_manager
{
private:
    CAN_speed_t         _baud_speed;
    gpio_num_t          _tx;
    gpio_num_t          _rx;
    CAN_frame_format_t  _frame;
    uint32_t            _can_id;
    uint8_t             _send_size ;

public:

    CAN_manager(CAN_speed_t ,gpio_num_t ,gpio_num_t , CAN_frame_format_t ,uint32_t , uint8_t );

    void CAN_prepare(void);
    bool CAN_ReceiveFrom(int);
    uint8_t GetByteReceived(unsigned char);


    bool CAN_Send();
    void SetByteTransmit(uint8_t,unsigned char);

    ~CAN_manager();
};

#endif