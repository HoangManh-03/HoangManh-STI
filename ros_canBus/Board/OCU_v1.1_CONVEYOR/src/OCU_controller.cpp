#include "OCU_controller.h"

OCU_controller::OCU_controller(CAN_manager* _CANctr)
    :DriverOne(CTR_ENABLE1, CTR_PWM_H1, CTR_PWM_L1, channelPWM_1, channelPWM_2),
    DriverSecond(CTR_ENABLE2, CTR_PWM_H2, CTR_PWM_L2, channelPWM_3, channelPWM_4),
    sensor1(SENSOR_1, INPUT_PULLUP),
    sensor2(SENSOR_2, INPUT_PULLUP),
    sensor3(SENSOR_3, INPUT_PULLUP),
    sensor4(SENSOR_4, INPUT_PULLUP),
    sensor5(SENSOR_5, INPUT_PULLUP),
    sensor6(SENSOR_6, INPUT_PULLUP),
    sensor7(SENSOR_7, INPUT_PULLUP)
{
    CANctr = _CANctr;
}

OCU_controller::~OCU_controller()
{
  delete CAN_sendData;
  delete CAN_receivedData;
  delete CAN_command;
}

void OCU_controller::setupBegin(){
    Serial.begin(115200);
    sensor1.init();
    sensor2.init();
    sensor3.init();
    sensor4.init();
    sensor5.init();
    sensor6.init();
    sensor7.init();

    // -
    pinMode(EMERGENCY_OUT, OUTPUT);
    digitalWrite(EMERGENCY_OUT, LOW);

    // Serial.println("dfatataa");
    // delay(2000);

    // - 
    DriverOne.SetupBegin();
    DriverSecond.SetupBegin();
}

void OCU_controller::OCU_CAN_Receive(){ // - OK
  if (CANctr->CAN_ReceiveFrom(ID_RTC)){
    if (CANctr->GetByteReceived(POS_ID) == ID_OCU_CONVEYOR){
      CAN_receivedData->mission1 = CANctr->GetByteReceived(POS_MISSION1);
      CAN_receivedData->speed1   = CANctr->GetByteReceived(POS_SPEED1);

    //   CAN_receivedData->mission2 = CANctr->GetByteReceived(POS_MISSION2);
    //   CAN_receivedData->speed2   = CANctr->GetByteReceived(POS_SPEED2);
    }
  }
}

void OCU_controller::OCU_CAN_Transmit(){ // - OK
  // - SetByteTransmit(data, position)
  CANctr->SetByteTransmit(CAN_sendData->mode_driver_1, POS_mode_driver1);
  CANctr->SetByteTransmit(CAN_sendData->status_driver_1,  POS_status_driver1);
  // -
  CANctr->SetByteTransmit(CAN_sendData->mode_driver_2, POS_mode_driver2);
  CANctr->SetByteTransmit(CAN_sendData->status_driver_2,   POS_status_driver2);
  // - 
  CANctr->SetByteTransmit(CAN_sendData->status_sensor, POS_status_sensor);
  CANctr->SetByteTransmit(CAN_sendData->status_can,  POS_status_can);

  if (CANctr->CAN_Send()){
    // Serial.println("OC CAN_Send - OKE");
  }else{
    Serial.println("OC CAN_Send - ERROR");
  }
}

void OCU_controller::OCU_CAN_send(){ // - OK
  if ((millis() - preTime_sendCAN) > (1000 / FREQUENCY_sendCAN))
  {   
    preTime_sendCAN = millis();
    OCU_CAN_Transmit();
  }
}

void OCU_controller::OCU_loop(){
    // Duoc trang thai cam bien hien tai
    uint8_t status_sensor = 0;
    status_sensor = sensor1.ReadStatus() |
                    sensor2.ReadStatus() << 1 |
                    sensor3.ReadStatus() << 2 |
                    sensor4.ReadStatus() << 3 |
                    sensor5.ReadStatus() << 4 |
                    sensor6.ReadStatus() << 5 |
                    sensor7.ReadStatus() << 6 ;

    // Serial.print("Binary: ");
    // Serial.println(status_sensor, BIN);
    CAN_sendData->status_sensor = status_sensor;

    // Lấy thông tin CMD từ CAN - > Điều khiển driver
    if (CAN_command->mission1 != CAN_receivedData->mission1){
        CAN_command->mission1 = CAN_receivedData->mission1;
        CAN_command->speed1 = CAN_receivedData->speed1;  
        DriverOne.StopAndReset();
    }

    // if (CAN_command->mission2 != CAN_receivedData->mission2){
    //     CAN_command->mission2 = CAN_receivedData->mission2;
    //     CAN_command->speed2 = CAN_receivedData->speed2;
    //     DriverSecond.StopAndReset();
    // }

    // ------ Driver 1 ------
    if (CAN_command->mission1 == MISSION_RESET){
        // Serial.println("CY1: MISSION_RESET");
        DriverOne.StopAndReset();

    }else if (CAN_command->mission1 == MISSION_RECEIVE_DIR1){
        // Serial.println("CY1: MISSION_RECEIVE");
        /* uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed */
        DriverOne.ControlDriver_modeConveyor_recieve(sensor4.GetStatus(0), sensor3.GetStatus(0), 0, CAN_command->speed1);

    }else if (CAN_command->mission1 == MISSION_RECEIVE_DIR2){
        // Serial.println("CY1: MISSION_RECEIVE");
        /* uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed */
        DriverOne.ControlDriver_modeConveyor_recieve(sensor3.GetStatus(0), sensor4.GetStatus(0), 1, CAN_command->speed1);

    }else if (CAN_command->mission1 == MISSION_TRANSMISSION_DIR1){
        // Serial.println("CY1: MISSION_RECEIVE");
        /* uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed */
        DriverOne.ControlDriver_modeConveyor_transfer(sensor4.GetStatus(1), sensor3.GetStatus(1), 0, CAN_command->speed1);

    }else if (CAN_command->mission1 == MISSION_TRANSMISSION_DIR2){
        // Serial.println("CY1: MISSION_RECEIVE");
        /* uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed */
        DriverOne.ControlDriver_modeConveyor_transfer(sensor3.GetStatus(1), sensor4.GetStatus(1), 1, CAN_command->speed1);

    }else if (CAN_command->mission1 == MISSION_TURN_DIR1){
        // Serial.println("CY1: MISSION_RECEIVE");
        /* uint8_t _dir, uint8_t _speed, uint8_t _sst_sensor_stop */
        CAN_command->speed1 = CAN_receivedData->speed1; 
        DriverOne.ControlDriver_modeNormal(0, CAN_command->speed1, 0);

    }else if (CAN_command->mission1 == MISSION_TURN_DIR2){
        // Serial.println("CY1: MISSION_RECEIVE");
        /* uint8_t _dir, uint8_t _speed, uint8_t _sst_sensor_stop */
        CAN_command->speed1 = CAN_receivedData->speed1; 
        DriverOne.ControlDriver_modeNormal(1, CAN_command->speed1, 0);

    }else{
        // Serial.println("CY1: UNK");
        DriverOne.StopAndReset();
    }

    // -- Get status Driver 1
    CAN_sendData->mode_driver_1 = CAN_command->mission1;
    CAN_sendData->status_driver_1 = DriverOne.GetStatus();


    // ------ Driver 2 ------
    // if (CAN_command->mission2 == MISSION_RESET){
    //     // Serial.println("CY1: MISSION_RESET");
    //     DriverSecond.StopAndReset();

    // }else if (CAN_command->mission2 == MISSION_RECEIVE_DIR1){
    //     // Serial.println("CY1: MISSION_RECEIVE");
    //     /* uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed */
    //     DriverSecond.ControlDriver_modeConveyor_recieve(sensor2.GetStatus(0), sensor1.GetStatus(0), 0, CAN_command->speed2);

    // }else if (CAN_command->mission2 == MISSION_RECEIVE_DIR2){
    //     // Serial.println("CY1: MISSION_RECEIVE");
    //     /* uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed */
    //     DriverSecond.ControlDriver_modeConveyor_recieve(sensor1.GetStatus(0), sensor2.GetStatus(0), 1, CAN_command->speed2);

    // }else if (CAN_command->mission2 == MISSION_TRANSMISSION_DIR1){
    //     // Serial.println("CY1: MISSION_RECEIVE");
    //     /* uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed */
    //     DriverSecond.ControlDriver_modeConveyor_transfer(sensor2.GetStatus(1), sensor1.GetStatus(1), 0, CAN_command->speed2);

    // }else if (CAN_command->mission2 == MISSION_TRANSMISSION_DIR2){
    //     // Serial.println("CY1: MISSION_RECEIVE");
    //     /* uint8_t _stt_sensor_behind, uint8_t _stt_sensor_ahead, uint8_t _dir, uint8_t _speed */
    //     DriverSecond.ControlDriver_modeConveyor_transfer(sensor1.GetStatus(1), sensor2.GetStatus(1), 1, CAN_command->speed2);

    // }else if (CAN_command->mission2 == MISSION_TURN_DIR1){
    //     // Serial.println("CY1: MISSION_RECEIVE");
    //     /* uint8_t _dir, uint8_t _speed, uint8_t _sst_sensor_stop */
    //     CAN_command->speed2 = CAN_receivedData->speed2;
    //     DriverSecond.ControlDriver_modeNormal(0, CAN_command->speed2, 0);

    // }else if (CAN_command->mission2 == MISSION_TURN_DIR2){
    //     // Serial.println("CY1: MISSION_RECEIVE");
    //     /* uint8_t _dir, uint8_t _speed, uint8_t _sst_sensor_stop */
    //     CAN_command->speed2 = CAN_receivedData->speed2;
    //     DriverSecond.ControlDriver_modeNormal(1, CAN_command->speed2, 0);

    // }else{
    //     // Serial.println("CY1: UNK");
    //     DriverSecond.StopAndReset();
    // }

    // // -- Get status Driver 2
    // CAN_sendData->mode_driver_2 = CAN_command->mission2;
    // CAN_sendData->status_driver_2 = DriverSecond.GetStatus();

    // -- SEND CAN
    OCU_CAN_send();
}

void OCU_controller::Test1(){
    DriverOne.ControlDriver_modeNormal(0, 50, 0);
    DriverSecond.ControlDriver_modeNormal(1, 100, 0);
}
