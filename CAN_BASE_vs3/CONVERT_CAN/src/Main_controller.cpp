#include "Main_controller.h"

Main_controller::Main_controller(CAN_manager* Main)
{
    CAN_main = Main;
}

Main_controller::~Main_controller()
{
    delete OC_comd;
    delete OC_status;
    delete HC_comd;
    delete HC_status;
    delete Main_comd;
    delete Main_status;
}

bool Main_controller::CANTransmitHandle() {
    if (switch_sendCAN == 0){
        CAN_main->SetByteTransmit(HC_comd->CAN_address, HC_ID);
        CAN_main->SetByteTransmit(HC_comd->RGB1,        HC_send_RGB1);
        CAN_main->SetByteTransmit(HC_comd->RGB2,        HC_send_RGB2);
        switch_sendCAN = 1;

    }else if (switch_sendCAN == 1){ 
        CAN_main->SetByteTransmit(OC_comd->CAN_address, OC_ID);
        CAN_main->SetByteTransmit(OC_comd->command,     OC_send_commandRequire);
        CAN_main->SetByteTransmit(OC_comd->resetError,  OC_send_commandReset);
        switch_sendCAN = 2;

    }else{
        CAN_main->SetByteTransmit(Main_comd->CAN_address,   MAIN_ID);
        CAN_main->SetByteTransmit(Main_comd->sound_enb,     MAIN_send_sound_enb);
        CAN_main->SetByteTransmit(Main_comd->sound_type,    MAIN_send_sound_type);
        CAN_main->SetByteTransmit(Main_comd->charge_write,  MAIN_send_charge_write);
        CAN_main->SetByteTransmit(Main_comd->EMG_reset,     MAIN_send_EMG_reset);
        CAN_main->SetByteTransmit(Main_comd->EMG_write,     MAIN_send_EMG_write);
        switch_sendCAN = 0;
    }

    if (CAN_main->CAN_Send()) {
        return 1;
    }else{
        return 0;
    }
}

void Main_controller::CANReceiveHandle() {
    uint32_t addRevFromCanBus;
    uint8_t stsAll_CAN = 0;
    // -------
    addRevFromCanBus = CAN_main->CAN_ReceiveFrom();
    if(addRevFromCanBus != 0x55) // 0x55 | 85
    {
        saveTime_checkCAN = millis();
    }

    // -------
    if (addRevFromCanBus == ID_HC){
        HC_status->status           = CAN_main->GetByteReceived(7);
        HC_status->zone_sick_ahead  = CAN_main->GetByteReceived(HC_rec_ZONE_AHEAD);
        HC_status->zone_sick_behind = CAN_main->GetByteReceived(HC_rec_ZONE_BEHIND);
        HC_status->vacham           = CAN_main->GetByteReceived(HC_rec_conllision);
        
        saveTime_checkHC = millis();
    }

    // -------
    if (addRevFromCanBus == ID_MAIN){
        Main_status->byte0_analogVoltage = CAN_main->GetByteReceived(MAIN_rec_analogVoltage0);
        Main_status->byte1_analogVoltage = CAN_main->GetByteReceived(MAIN_rec_analogVoltage1);
        Main_status->byte0_analogCurrent = CAN_main->GetByteReceived(MAIN_rec_analogCurrent0);
        Main_status->byte1_analogCurrent = CAN_main->GetByteReceived(MAIN_rec_analogCurrent1);
        Main_status->stsButton_reset     = CAN_main->GetByteReceived(MAIN_rec_stsButton_reset);
        Main_status->stsButton_power     = CAN_main->GetByteReceived(MAIN_rec_stsButton_power);
        Main_status->EMG_status          = CAN_main->GetByteReceived(MAIN_rec_EMG_status);

        saveTime_checkMain = millis();
    }

    // -------
    if (addRevFromCanBus == ID_OC){
        OC_status->command          = CAN_main->GetByteReceived(OC_rec_FeedbackCommand);
        OC_status->commandStatus    = CAN_main->GetByteReceived(OC_rec_commandStatus);
        OC_status->sensorBit_status = CAN_main->GetByteReceived(OC_rec_sensorsData);
        OC_status->error            = CAN_main->GetByteReceived(OC_rec_error_check);
        
        saveTime_checkOC = millis();
    }

    // --
    if ((millis() - saveTime_checkHC) > 320 )
    {   
        stsRev_CAN_HC = 0;
    }else{
        stsRev_CAN_HC = 1;
    }

    if ((millis() - saveTime_checkMain) > 1200 )
    {   
        stsRev_CAN_Main = 0;
    }else{
        stsRev_CAN_Main = 1;
    }

    if ((millis() - saveTime_checkOC) > 1200 )
    {   
        stsRev_CAN_OC = 0;
    }else{
        stsRev_CAN_OC = 1;
    }

    if ((millis() - saveTime_checkCAN) > 2000 )
    {   
        stsRev_CAN = 0;
    }else{
        stsRev_CAN = 1;
    }
}


void Main_controller::led_start(){
	pinMode(27, OUTPUT);
	digitalWrite(27, 1);
	delay(100);
	digitalWrite(27, 0);
	delay(100);
	digitalWrite(27, 1);
	delay(100);
	digitalWrite(27, 0);
	delay(100);
	digitalWrite(27, 1);
	delay(100);
	digitalWrite(27, 0);
	delay(100);
	digitalWrite(27, 1);
	delay(100);
	digitalWrite(27, 0);
	delay(100);	
}

void Main_controller::init_main(){
    WiFi.mode(WIFI_OFF);
    btStop();
    //-------------------------------------------- OUT ------------------ 
}

void Main_controller::loopMainCtr(){
    // - send CAN
    if ( (millis() - saveTime_sendCAN) > 80){ // 12.5 HZ
        saveTime_sendCAN = millis();
        stsSend_CAN = CANTransmitHandle();
    }
}

int Main_controller::bytes_to_int(uint8_t byte0, uint8_t byte1){ // for 2 bytes
    return byte0 + byte1*256;
}

uint8_t Main_controller::int_to_byte0(int value){ // for 2 bytes
    
    return value - (value/256)*256;
}

uint8_t Main_controller::int_to_byte1(int value){ // for 2 bytes
    
    return value/256;
}

bool Main_controller::getBit_fromInt(int val, int pos){
    bool bit;
    int val_now = val;
    for (int i = 0; i < 8; i++){
        bit = val_now%2;
        val_now /= 2;

        if (i == pos){
            return bit;
        }

        if (val_now < 1)
            return 0;
    }
    return 0;
}
