#include "Main_controller.h"

Main_controller::Main_controller(CAN_manager* Main)
{
    CAN_main = Main;
}

Main_controller::~Main_controller()
{
    delete Main_status;
    delete Main_comd;
}

bool Main_controller::CANTransmitHandle() {
    CAN_main->SetByteTransmit(Main_status->byte0_analogVoltage, 0);
    CAN_main->SetByteTransmit(Main_status->byte1_analogVoltage, 1);
    CAN_main->SetByteTransmit(Main_status->byte0_analogCurrent, 2);
    CAN_main->SetByteTransmit(Main_status->byte1_analogCurrent, 3);
    CAN_main->SetByteTransmit(Main_status->stsButton_reset, 4);
    CAN_main->SetByteTransmit(Main_status->stsButton_power, 5);
    CAN_main->SetByteTransmit(Main_status->EMG_status, 6);
    CAN_main->SetByteTransmit(Main_status->stsRev_CAN, 7);

    if (CAN_main->CAN_Send()) {
        return 1;
    }else{
        return 0;
    }
}

void Main_controller::CANReceiveHandle() {
    if (CAN_main->CAN_ReceiveFrom(ID_Convert) )
    {
        if (CAN_main->GetByteReceived(0) == ID_MAIN)
        {
            saveTime_checkCAN_Rev = millis();
            Main_comd->sound_enb    = CAN_main->GetByteReceived(1);
            Main_comd->sound_type   = CAN_main->GetByteReceived(2);
            Main_comd->charge_write = CAN_main->GetByteReceived(3);
            Main_comd->EMG_reset    = CAN_main->GetByteReceived(4);
            Main_comd->EMG_write    = CAN_main->GetByteReceived(5);
            // Serial.printf("Received: command - %d | reset - %d\n", cmd_request, cmd_resetError);
        }
    }
    
    if ((millis() - saveTime_checkCAN_Rev) > 1000)
    {   
        statusRev_CAN = 0;
    }else{
        statusRev_CAN = 1;
    }
    Main_status->stsRev_CAN = statusRev_CAN;
}

void Main_controller::soundControl(uint8_t enb, uint8_t comd){
    //  debugSerial.println((String) "[SND] "+ comd);
    if (enb == 0){
        digitalWrite(PIN_SOUND_SP, HIGH);
        digitalWrite(PIN_SOUND_1, HIGH);
        digitalWrite(PIN_SOUND_2, HIGH);
        digitalWrite(PIN_SOUND_3, HIGH);
        digitalWrite(PIN_SOUND_4, HIGH);
    }else{
        digitalWrite(PIN_SOUND_SP , LOW);
        switch(comd){
            case commands::SOUND_OE_OE:
                digitalWrite(PIN_SOUND_1,HIGH);
                digitalWrite(PIN_SOUND_2,LOW);
                digitalWrite(PIN_SOUND_3,HIGH);
                digitalWrite(PIN_SOUND_4,HIGH);
                break;
            case commands::SOUND_FUR_ELISE:
                digitalWrite(PIN_SOUND_1,LOW);
                digitalWrite(PIN_SOUND_2,HIGH);
                digitalWrite(PIN_SOUND_3,HIGH);
                digitalWrite(PIN_SOUND_4,HIGH);
                break;
            case commands::SOUND_3: /* Ambulance */
                digitalWrite(PIN_SOUND_1,HIGH);
                digitalWrite(PIN_SOUND_2,HIGH);
                digitalWrite(PIN_SOUND_3,HIGH);
                digitalWrite(PIN_SOUND_4,LOW);
                break;
            case commands::SOUND_EMC:
                digitalWrite(PIN_SOUND_1,HIGH);
                digitalWrite(PIN_SOUND_2,HIGH);
                digitalWrite(PIN_SOUND_3,LOW);
                digitalWrite(PIN_SOUND_4,HIGH);
                break;
            default:
                digitalWrite(PIN_SOUND_1,HIGH);
                digitalWrite(PIN_SOUND_2,HIGH);
                digitalWrite(PIN_SOUND_3,HIGH);
                digitalWrite(PIN_SOUND_4,HIGH);
                break;
        }
    }
}

void Main_controller::init_main(){
    WiFi.mode(WIFI_OFF);
    btStop();
    //-------------------------------------------- OUT ------------------ 
    pinMode(PIN_WRITE_POWER, OUTPUT);
    digitalWrite(PIN_WRITE_POWER, LOW); 

    pinMode(PIN_EMC_READ, INPUT_PULLUP);
    pinMode(PIN_EMC_WRITE, OUTPUT);

    digitalWrite(PIN_EMC_WRITE, HIGH);

    pinMode(PIN_EMC_RESET, OUTPUT);
    digitalWrite(PIN_EMC_RESET, LOW);

    pinMode(PIN_ADC_VOLTAGE, INPUT);
    pinMode(PIN_ADC_CURRENT, INPUT);

    pinMode(PIN_SOUND_SP, OUTPUT);
    digitalWrite(PIN_SOUND_SP, LOW);

    pinMode(PIN_SOUND_1, OUTPUT);
    pinMode(PIN_SOUND_2, OUTPUT);
    pinMode(PIN_SOUND_3, OUTPUT);
    pinMode(PIN_SOUND_4, OUTPUT);

    digitalWrite(PIN_SOUND_1, HIGH);
    digitalWrite(PIN_SOUND_2, HIGH);
    digitalWrite(PIN_SOUND_3, HIGH);
    digitalWrite(PIN_SOUND_4, HIGH);

    pinMode(PIN_READ_PC, INPUT);
    pinMode(PIN_CHARGE, OUTPUT);

    pinMode(ENABLE_5v, OUTPUT);
    pinMode(ENABLE_22v, OUTPUT);

    pinMode(BUTTON_SHUTDOWN, INPUT_PULLUP);
    pinMode(BUTTON_EMC_RES, INPUT_PULLUP);

    // -- Giu nguon
    digitalWrite(ENABLE_5v, LOW);
    digitalWrite(ENABLE_22v, LOW);

    pinMode(PIN_L1, OUTPUT);
    pinMode(PIN_L2, OUTPUT);
    digitalWrite(PIN_L1, LOW);
    digitalWrite(PIN_L2, LOW);

}

void Main_controller::EMG_detect(){
    if (digitalRead(PIN_EMC_READ) == LOW) {
        // SetEMG(true);
        Main_status->EMG_status = 1;
    }else{
        Main_status->EMG_status = 0;
    }
}

void Main_controller::read_voltage(){
    // - read voltage pin 
    uint16_t voltage_analog;
    voltage_analog = analogRead(PIN_ADC_VOLTAGE);
    voltageAnalog = filter_analogVoltages(voltage_analog);

    Main_status->byte0_analogVoltage = int_to_byte0(voltageAnalog);
    Main_status->byte1_analogVoltage = int_to_byte1(voltageAnalog);
}

void Main_controller::read_currentCharge(){
    // - read current pin 
    currentAnalog = analogRead(PIN_ADC_CURRENT);
    Main_status->byte0_analogCurrent = int_to_byte0(currentAnalog);
    Main_status->byte1_analogCurrent = int_to_byte1(currentAnalog);
}

void Main_controller::readButton_Shutdown(){
    // B2 in board
    if (digitalRead(BUTTON_SHUTDOWN) == 0){
        saveTime_checkShutdown = millis();
        Main_status->stsButton_power = 1;
    }else{
        Main_status->stsButton_power = 0;
    }

    if (millis() - saveTime_checkShutdown > 6000){
        digitalWrite(ENABLE_5v, HIGH);
        digitalWrite(ENABLE_22v, HIGH);
    }
}

void Main_controller::resetEMG(){
    bool sts;
    // - B1 in board || control EMG reset
    sts = digitalRead(BUTTON_EMC_RES);
    if (sts == false && Main_comd->EMG_reset == false){
        saveTime_checkReset = millis();
    }

    if (sts == false){
        Main_status->stsButton_reset = false;
    }else{
        Main_status->stsButton_reset = true;
    }
    
    if (flag_reseting == 0){
        if (millis() - saveTime_checkReset > 250){
            SetEMG(false);
            digitalWrite(PIN_EMC_RESET, HIGH);
            flag_reseting = 1;
            saveTime_checkReseted = millis();
        }
    }else if (flag_reseting == 1){
        if (millis() - saveTime_checkReseted > 150){
            digitalWrite(PIN_EMC_RESET, LOW);
            flag_reseting = 2;
            saveTime_checkReset = millis();
        }
    }else if (flag_reseting == 2){
        if (sts == false && Main_comd->EMG_reset == false){
            flag_reseting = 0;
            saveTime_checkReset = millis();
        }
    }
}

void Main_controller::shutdown_voltageLow(){
    if (voltageAnalog > 2500){
    // if (22 > 21.){
        saveTime_checkLowVoltage = millis();
    }

    if ((millis() - saveTime_checkLowVoltage) > 20000){
        SetEMG(true);
        digitalWrite(ENABLE_5v  , HIGH);
        digitalWrite(ENABLE_22v , HIGH);
    }
}

void Main_controller::loopMainCtr(){
    // - send CAN
    if ( (millis() - saveTime_sendCAN) > 200){ // - 5 HZ
        saveTime_sendCAN = millis();
        CANTransmitHandle();
    }
    
    // - EMG
    EMG_detect();

    // - Voltage
    read_voltage();

    // - Current charge
    read_currentCharge();

    // - Read button shutdown
    readButton_Shutdown();

    // - Reset EMG - Button reset - Command reset
    resetEMG();

    // - Check low voltages
    shutdown_voltageLow();

    // --------------------------------------
    // - Speaker
    soundControl(Main_comd->sound_enb, Main_comd->sound_type);

    // - Charge
    if (Main_comd->charge_write == true){
        // - charge ON
        digitalWrite(PIN_CHARGE , HIGH);
    }else{
        // - charge OFF
        digitalWrite(PIN_CHARGE , LOW);
    }
    
    // - EMG write
    if (Main_comd->EMG_write == true){
        SetEMG(true);
    }
    else{
        SetEMG(false);
    }
    
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

int Main_controller::bytes_to_int(uint8_t byte0, uint8_t byte1){ // for 2 bytes
    return byte0 + byte1*256;
}

uint8_t Main_controller::int_to_byte0(int value){ // for 2 bytes
    
    return value - (value/256)*256;
}

uint8_t Main_controller::int_to_byte1(int value){ // for 2 bytes
    
    return value/256;
}