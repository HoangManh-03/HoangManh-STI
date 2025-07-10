#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: HOANG VAN QUANG - BEE
# DATE: 08/18/2022

from message_pkg.msg import *
from sti_msgs.msg import *
from geometry_msgs.msg import Twist
import time
import rospy
from std_msgs.msg import Int8
from ros_canBus.msg import *

class CAN_ROS():
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('CAN_ROS', anonymous=False)
        self.rate = rospy.Rate(50)

        # ------------- PARAMETER ------------- #
        self.ID_RTC  = rospy.get_param('ID_RTC', 2)
        self.ID_RTC  = 1
        self.ID_HC   = rospy.get_param('ID_HCU', 1)
        self.ID_HC   = 2
        self.ID_MAIN = rospy.get_param('ID_MAIN', 3)
        self.ID_MAIN = 3
        self.ID_PSU = rospy.get_param('ID_PSU', 4)
        self.ID_PSU = 4
        self.ID_OC = rospy.get_param('ID_OC', 5)
        self.ID_OC = 5
        # ------------- ROS ------------- #
        # - PUBLISH

        # -- HCU BOARD
        self.pub_statusHCU = rospy.Publisher("/HCU_info", HCU_info, queue_size = 200)
        self.HCU_status = HCU_info()
        # -- MCU BOARD
        self.pub_statusMCU = rospy.Publisher("/MCU_info", MCU_info, queue_size = 20)
        self.MCU_status = MCU_info()
        # -- PSU BOARD
        self.pub_statusPSU = rospy.Publisher("/PSU_info", PSU_info, queue_size = 20)
        self.PSU_status = PSU_info()
        # -- OC BOARD
        self.pub_statusOC = rospy.Publisher("/lift_status", Lift_status, queue_size = 20)
        self.OC_status = Lift_status()

        # ---
        self.pub_statusPOWER = rospy.Publisher("/POWER_info", POWER_info, queue_size = 200)
        self.POWER_info = POWER_info()
        # --
        self.pub_statusHC = rospy.Publisher("/HC_info", HC_info, queue_size = 20)
        self.hc_info = HC_info()

        # --
        self.pub_sendCAN = rospy.Publisher("/CAN_send", CAN_send, queue_size= 40)
        self.CAN_dataSend = CAN_send()
        self.CAN_frequenceSend = 14. # - Hz
        self.CAN_saveTimeSend = time.time()
        self.CAN_sortSend = 0

        rospy.sleep(1)  # wait for connections

        # - SUBCRIBER
        rospy.Subscriber("/CAN_received", CAN_received, self.callback_dataCAN)
        self.CAN_dataReceived = CAN_received()

        rospy.Subscriber("/HC_request", HC_request, self.callback_controlHC)
        self.HC_control = HC_request()

        rospy.Subscriber("/HC_fieldRequest", Int8, self.callback_controlFieldHC)
        self.data_controlFieldSick = Int8()

        rospy.Subscriber("/POWER_request", POWER_request, self.callback_controlPOWER)
        self.POWER_control = POWER_request()

        rospy.Subscriber("/lift_control", Lift_control, self.callback_controlLift)
        self.lift_control = Lift_control()


    # -------------------
    def callback_controlHC(self, data):
        self.HC_control = data

    def callback_controlFieldHC(self, data):
        self.data_controlFieldSick = data

    def callback_controlPOWER(self, data):
        self.POWER_control = data

    def callback_controlLift(self, data):
        self.lift_control = data

    def callback_dataCAN(self, data):
        self.CAN_dataReceived = data
        self.analysisFrame_receivedCAN()

    def convert_4byte_int(self, byte0, byte1, byte2, byte3):
        int_out = 0
        int_out = byte0 + byte1*256 + byte2*256*256 + byte3*256*256*256
        if int_out > pow(2, 32)/2.:
            int_out = int_out - pow(2, 32)
        return int_out

    def convert_16bit_int(self, bitArr):
        int_out = 0
        for i in range(16):
            int_out += bitArr[i]*pow(2, i)
        return int_out

    def getByte_fromInt16(self, valueIn, pos):
        byte1 = int(valueIn/256)
        byte0 =  valueIn - byte1*256
        if (pos == 0):
            return byte0
        else:
            return byte1

    def getBit_fromInt8(self, value_in, pos):
        bit_out = 0
        value_now = value_in
        for i in range(8):
            bit_out = value_now%2
            value_now = value_now/2
            if (i == pos):
                return bit_out

            if (value_now < 1):
                return 0
        return 0

    def getBit_fromInt16(self, value_in, pos):
        bit_out = 0
        value_now = value_in
        for i in range(16):
            bit_out = value_now%2
            value_now = value_now/2
            if (i == pos):
                return int(bit_out)

            if (value_now < 1):
                return 0
        return 0

    def syntheticFrame_Sick(self, data):
        zone1 = data&1
        zone2 = (data>>1)&1
        zone3 = (data>>2)&1
        zone4 = (data>>3)&1

        if zone1 == 1:
            return 1
        elif zone2 == 1:
            return 2
        elif zone3 == 1:
            return 3
        elif zone4 == 1:
            return 4
        else:
            return 0

    def syntheticFrame_sendCAN(self):
        # -- HCU
        if (self.CAN_sortSend == 0):
            self.CAN_dataSend.id = self.ID_RTC
            self.CAN_dataSend.byte0 = self.ID_HC
            self.CAN_dataSend.byte1 = self.HC_control.RBG1
            self.CAN_dataSend.byte2 = self.HC_control.RBG2

            modeSound = self.POWER_control.sound_type
            if self.POWER_control.sound_on == 0:
                modeSound = 0
            self.CAN_dataSend.byte3 = modeSound
            self.CAN_dataSend.byte4 = 0b00000000
            self.CAN_dataSend.byte5 = 0
            self.CAN_dataSend.byte6 = self.data_controlFieldSick.data
            self.CAN_dataSend.byte7 = 0
            self.CAN_sortSend = 1

        # -- MCU
        elif (self.CAN_sortSend == 1):
            self.CAN_dataSend.id = self.ID_RTC
            self.CAN_dataSend.byte0 = self.ID_MAIN
            self.CAN_dataSend.byte1 = self.POWER_control.EMC_reset
            self.CAN_dataSend.byte2 = self.POWER_control.EMC_write
            self.CAN_dataSend.byte3 = 0
            self.CAN_dataSend.byte4 = 0
            self.CAN_dataSend.byte5 = 0
            self.CAN_dataSend.byte6 = 0
            self.CAN_dataSend.byte7 = 0
            self.CAN_sortSend = 2

        # -- PSU
        elif (self.CAN_sortSend == 2):
            self.CAN_dataSend.id = self.ID_RTC
            self.CAN_dataSend.byte0 = self.ID_PSU
            self.CAN_dataSend.byte1 = self.POWER_control.charge
            self.CAN_dataSend.byte2 = 0
            self.CAN_dataSend.byte3 = 0
            self.CAN_dataSend.byte4 = 0
            self.CAN_dataSend.byte5 = 0
            self.CAN_dataSend.byte6 = 0
            self.CAN_dataSend.byte7 = 0
            self.CAN_sortSend = 3

        # -- OC
        elif (self.CAN_sortSend == 3):
            self.CAN_dataSend.id = self.ID_RTC
            self.CAN_dataSend.byte0 = self.ID_OC
            self.CAN_dataSend.byte1 = self.lift_control.control.data
            self.CAN_dataSend.byte2 = self.lift_control.reset.data
            self.CAN_dataSend.byte3 = 0
            self.CAN_dataSend.byte4 = 0
            self.CAN_dataSend.byte5 = 0
            self.CAN_dataSend.byte6 = 0
            self.CAN_dataSend.byte7 = 0
            self.CAN_sortSend = 0

    def analysisFrame_receivedCAN(self):
        # -- HC
        if self.CAN_dataReceived.idSend == self.ID_HC:
            self.HCU_status.status_recCAN = self.CAN_dataReceived.byte0
            self.HCU_status.area1_zone1 = (int(self.CAN_dataReceived.byte1))&1
            self.HCU_status.area1_zone2 = (int(self.CAN_dataReceived.byte1)>>1)&1
            self.HCU_status.area1_zone3 = (int(self.CAN_dataReceived.byte1)>>2)&1
            self.HCU_status.area1_zone4 = (int(self.CAN_dataReceived.byte1)>>3)&1
            self.HCU_status.area2_zone1 = (int(self.CAN_dataReceived.byte2))&1
            self.HCU_status.area2_zone2 = (int(self.CAN_dataReceived.byte2)>>3)&1
            self.HCU_status.area2_zone3 = (int(self.CAN_dataReceived.byte2)>>3)&1
            self.HCU_status.area2_zone4 = (int(self.CAN_dataReceived.byte2)>>3)&1
            self.HCU_status.status_input1 = (int(self.CAN_dataReceived.byte3))&1
            self.HCU_status.status_input2 = (int(self.CAN_dataReceived.byte3)>>1)&1
            self.HCU_status.status_input3 = (int(self.CAN_dataReceived.byte3)>>2)&1
            self.HCU_status.status_input4 = (int(self.CAN_dataReceived.byte3)>>3)&1
            self.HCU_status.status_input5 = (int(self.CAN_dataReceived.byte3)>>4)&1
            self.HCU_status.status_input6 = (int(self.CAN_dataReceived.byte3)>>5)&1
            self.pub_statusHCU.publish(self.HCU_status)

            self.hc_info.status = self.HCU_status.status_recCAN
            self.hc_info.zone_sick_ahead = self.syntheticFrame_Sick(int(self.CAN_dataReceived.byte1))
            # self.hc_info.zone_sick_behind = self.syntheticFrame_Sick(int(self.CAN_dataReceived.byte2))
            self.hc_info.zone_sick_behind = 0


            self.pub_statusHC.publish(self.hc_info)
            # print ("--- HC")

        # -- Main
        elif self.CAN_dataReceived.idSend == self.ID_MAIN:
            self.MCU_status.status_recCAN = self.CAN_dataReceived.byte0
            self.MCU_status.status_safetyRelay = self.CAN_dataReceived.byte1
            self.MCU_status.status_EMG1 = (int(self.CAN_dataReceived.byte2))&1
            self.MCU_status.status_EMG2 = (int(self.CAN_dataReceived.byte2)>>1)&1
            self.MCU_status.status_EMG3 = (int(self.CAN_dataReceived.byte2)>>2)&1
            self.MCU_status.status_CS1 = (int(self.CAN_dataReceived.byte3))&1
            self.MCU_status.status_CS2 = (int(self.CAN_dataReceived.byte3)>>1)&1
            self.MCU_status.status_btnReset = self.CAN_dataReceived.byte4

            self.MCU_status.status_input1 = (int(self.CAN_dataReceived.byte5))&1
            self.MCU_status.status_input2 = (int(self.CAN_dataReceived.byte5)>>1)&1
            self.MCU_status.status_input3 = (int(self.CAN_dataReceived.byte5)>>2)&1
            self.MCU_status.status_input4 = (int(self.CAN_dataReceived.byte5)>>3)&1

            self.pub_statusMCU.publish(self.MCU_status)

            self.POWER_info.EMC_status = self.MCU_status.status_safetyRelay
            self.POWER_info.stsButton_reset = self.MCU_status.status_btnReset

            self.hc_info.vacham = self.MCU_status.status_CS1|self.MCU_status.status_CS2
            # print ("--- --- Main")

        # -- PSU
        elif (self.CAN_dataReceived.idSend == self.ID_PSU):
            self.PSU_status.stsRev_CAN = self.CAN_dataReceived.byte0
            self.PSU_status.voltages_analog = int((self.CAN_dataReceived.byte2 << 8) | self.CAN_dataReceived.byte1)
            self.PSU_status.voltages =  self.PSU_status.voltages_analog/10.
            # self.PSU_status.charge_analog = (self.CAN_dataReceived.byte4 << 8 | self.CAN_dataReceived.byte3) - 10.0
            self.PSU_status.charge_analog = (self.CAN_dataReceived.byte4 << 8 | self.CAN_dataReceived.byte3)
            self.PSU_status.charge_current = self.PSU_status.charge_analog
            self.PSU_status.stsButton_power = self.CAN_dataReceived.byte5
            self.PSU_status.status_input1 = self.CAN_dataReceived.byte6
            self.PSU_status.temperture = self.CAN_dataReceived.byte7
            #self.PSU_status.charge_analog = 0
            print(self.PSU_status.voltages_analog)

            print(self.PSU_status.charge_analog)
            self.pub_statusPSU.publish(self.PSU_status)
            # print ("--- --- --- PSU")
            # rospy.loginfo(f"voltages_analog value: {self.PSU_status.voltages_analog}")
            #rospy.loginfo(f"voltages_analog type: {type(self.PSU_status.voltages_analog)}")

            self.POWER_info.voltages = self.PSU_status.voltages
            self.POWER_info.voltages_analog = self.PSU_status.voltages_analog
            self.POWER_info.charge_current = self.PSU_status.charge_current
            self.POWER_info.charge_analog = self.PSU_status.charge_analog
            self.POWER_info.stsButton_power = self.PSU_status.stsButton_power
            self.POWER_info.CAN_status = self.PSU_status.stsRev_CAN

            self.pub_statusPOWER.publish(self.POWER_info)


        # -- OC
        elif (self.CAN_dataReceived.idSend == self.ID_OC):
            stsOC = int(self.CAN_dataReceived.byte1)

            if self.CAN_dataReceived.byte4 == 0:
                stsOC = -2

            self.OC_status.status.data = stsOC

            self.OC_status.sensorLift.data = (int(self.CAN_dataReceived.byte2))&1
            self.OC_status.sensorUp.data = (int(self.CAN_dataReceived.byte2)>>3)&1
            self.OC_status.sensorDown.data = (int(self.CAN_dataReceived.byte2)>>2)&1

            self.pub_statusOC.publish(self.OC_status)

    def try_run(self):
        val = 100
        print ("Bit0", self.getBit_fromInt8(val, 0))
        print ("Bit1", self.getBit_fromInt8(val, 1))
        print ("Bit2", self.getBit_fromInt8(val, 2))
        print ("Bit3", self.getBit_fromInt8(val, 3))
        print ("Bit4", self.getBit_fromInt8(val, 4))
        print ("Bit5", self.getBit_fromInt8(val, 5))
        print ("Bit6", self.getBit_fromInt8(val, 6))
        print ("Bit7", self.getBit_fromInt8(val, 7))

    def string_arry(self):
        arr_str = "0123456"
        print ("OUT0: ", arr_str[2:4])
        print ("OUT1: ", arr_str[0:2])
        print ("OUT2: ", arr_str[:2])
        print ("OUT3: ", arr_str[:-3])
        print ("OUT4: ", arr_str[-2:0])
        print ("OUT5: ", arr_str[2:0])

    def run(self):
        while not rospy.is_shutdown():
            # -- SEND CAN
            delta_time = (time.time() - self.CAN_saveTimeSend)%60
            if (delta_time > 1/self.CAN_frequenceSend):
                self.CAN_saveTimeSend = time.time()
                self.syntheticFrame_sendCAN()
                self.pub_sendCAN.publish(self.CAN_dataSend)

            self.rate.sleep()

def main():
    print('Program starting')
    program = CAN_ROS()
    program.run()
    print('Programer stopped')

if __name__ == '__main__':
    main()



