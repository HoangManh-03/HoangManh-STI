#ifndef CONTROLL_CONFIG
#define CONTROLL_CONFIG

enum CAN_TransmitDataType{
    POS_mode_driver1        = 0,
    POS_status_driver1      = 1,
    POS_mode_driver2        = 2,
    POS_status_driver2      = 3,
    POS_status_sensor       = 4,
    POS_status_can          = 5
};

enum CAN_ReceiveDataType{
    POS_ID          = 0,
    POS_MISSION1    = 1,
    POS_SPEED1      = 2,
    POS_MISSION2    = 3,
    POS_SPEED2      = 4
};

enum commands
{
    //MAIN commands
    SOUND_1                     = 0x11,/* Âm còi */
    SOUND_2                     = 0x12,/* Âm còi */
    SOUND_3                     = 0x13,/* Âm còi */
    SOUND_EMC                   = 0x14,/* Âm còi báo động */
    POWER_CHARGING              = 0x06,/* Bật sạc */
    POWER_UNCHARGING            = 0x05,/* Tắt sạc */
    ENABLE_SOFT_EMC             = 0x07,/* Bật EMC mềm */
    DISABLE_SOFT_EMC            = 0x08,/* Tắt EMC mềm */

    //MC commands
    MODE_STRAIGHT               = 0x21, //33
    TURN_LEFT                   = 0x22, //34
    TURN_RIGHT                  = 0x23, //35
    MODE_TURNING_BACKWARD       = 0x24, //36

    MODE_1_BIT_CATCH            = 0x25, //37
    MODE_HORIZON_LINE           = 0x26, //38
    MODE_RIGHT_DEVIATION        = 0x27, //39
    MODE_LEFT_DEVIATION         = 0x28, //40
    MODE_MOVE_BACKWARD          = 0x29, //41
    MODE_BREAK                  = 0x2A, //42
    MODE_BACKWARD_HORIZON_LINE  = 0x2B, //43
    MODE_BACKWARD_1_BIT_CATCH   = 0x2C, //44
    DO_NOTHING                  = 0x99, //153

    //HC commands
    LED_MODE_1                  = 0x31, //49
    LED_MODE_2                  = 0x32, //50
    LED_MODE_3                  = 0X33, //51
    LED_MODE_EMC                = 0x34, //52
    REQUEST_HEAD_FRONT          = 0x35, //53
    REQUEST_HEAD_BEHIND         = 0x36, //54

    // - OC commands
    MISSION_RESET                   = 0x00,
    MISSION_LIFT_UP                 = 0x01,
    MISSION_LIFT_DOWN               = 0x02,
    MISSION_RECEIVE_DIR1            = 0x03,
    MISSION_RECEIVE_DIR2            = 0x04,
    MISSION_TRANSMISSION_DIR1       = 0x05,
    MISSION_TRANSMISSION_DIR2       = 0x06,
    MISSION_TURN_DIR1               = 0x07,
    MISSION_TURN_DIR2               = 0x08
};


#endif