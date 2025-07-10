import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, qos_profile_sensor_data

from message_pkg.msg import *
from std_msgs.msg import Int16, Int8

class CAN_ROS(LifecycleNode):
	def __init__(self):
		super().__init__('CAN_ROS')
		self.get_logger().warn("ROS 2 Node CAN_ROS Initialized!")
		self.killnode = 0

		self.declare_parameters(
			namespace='',
			parameters=[
				('ID_RTC',  		1),
				('ID_MC',  			2),
				('ID_HC',  			3),
				('ID_MAIN',  		4),
				('ID_COVEYOR',   	5)
			]
		)

		# -- Get param
		self.ID_RTC = self.get_parameter('ID_RTC').value
		self.ID_MC = self.get_parameter('ID_MC').value
		self.ID_HC = self.get_parameter('ID_HC').value
		self.ID_MAIN = self.get_parameter('ID_MAIN').value
		self.ID_COVEYOR = self.get_parameter('ID_COVEYOR').value

		# -- Publisher 
		qos_profile_pub = QoSProfile(
			reliability=QoSReliabilityPolicy.RELIABLE,   # Đảm bảo tin nhắn được nhận đầy đủ
			durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Giữ lại tin nhắn cuối cùng như latched topic trong ROS1
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=10
		)
		# -
		self.pub_statusHC = self.create_publisher(HcInfo, '/hc_info', 10)
		self.HC_status = HcInfo()
		# -
		self.pub_statusHC2 = self.create_publisher(HcInfo2, '/hc_info2', 10)
		self.HC_status2 = HcInfo2()
		# -
		self.pub_statusMC = self.create_publisher(McInfo, '/mc_info', 10)
		self.MC_status = McInfo()
		# -
		self.pub_statusPower = self.create_publisher(PowerInfo, '/power_info', 10)
		self.Power_status = PowerInfo()
		# -
		self.pub_statusConveyor = self.create_publisher(ConveyorStatus, '/conveyor_status', 10)
		self.status_conveyor = ConveyorStatus()
		# -
		self.pub_sendCAN = self.create_publisher(Cansend, '/can_send', 10)
		self.CAN_dataSend = Cansend()
		self.CAN_frequenceSend = 20. # - Hz
		self.CAN_saveTimeSend = time.time()
		self.CAN_sortSend = 1

		# -- Subcriber
		# -
		# - QOS
		qos_profile_sub = QoSProfile(
			reliability=QoSReliabilityPolicy.RELIABLE,   # Đảm bảo tin nhắn được nhận đầy đủ
			durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Giữ lại tin nhắn cuối cùng như latched topic trong ROS1
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=10
		)
		
		self.sub_canReceived = self.create_subscription(
			Canreceived,
			'can_received',
			self.callback_dataCAN,
			qos_profile_sensor_data)
		self.sub_canReceived
		self.sdata_CANReceived = Canreceived()

		# -
		self.sub_powerRequest = self.create_subscription(
			PowerRequest,
			'power_request',
			self.callback_powerRequest,
			10)
		self.sub_powerRequest
		self.sdata_powerRequest = PowerRequest()

		# -
		self.sub_mcRequest = self.create_subscription(
			McRequest,
			'mc_request',
			self.callback_mcRequest,
			10)
		self.sub_mcRequest
		self.sdata_mcRequest = McRequest()

		# -
		self.sub_hcRequest = self.create_subscription(
			HcRequest,
			'hc_request',
			self.callback_hcRequest,
			10)
		self.sub_hcRequest
		self.sdata_hcRequest = HcRequest()

		# -
		self.sub_conveyorControl = self.create_subscription(
			ConveyorControl,
			'conveyor_control',
			self.callback_conveyorControl,
			10)
		self.sub_conveyorControl
		self.sdata_conveyorControl = ConveyorControl()

		# -- Loop
		self.rate = 20.0
		self.timer_period = 1.0/self.rate
		self.timer = self.create_timer(self.timer_period, self.run)

		self.savehc = time.time()

	def on_shutdown(self, state):
		self.killnode = 1
		self.get_logger().warn("Shutting down! Exiting program...")
		return TransitionCallbackReturn.SUCCESS

	# -- Callback function
	def callback_powerRequest(self, data):
		self.sdata_powerRequest = data

	def callback_mcRequest(self, data):
		self.sdata_mcRequest = data

	def callback_hcRequest(self, data):
		self.sdata_hcRequest = data

	def callback_conveyorControl(self, data):
		self.sdata_conveyorControl = data

	def callback_dataCAN(self, data):
		self.sdata_CANReceived = data
		self.analysisFrame_receivedCAN()

	def getBit_fromInt8(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(8):
			bit_out = value_now%2
			value_now = value_now/2
			if (i == pos):
				return int(bit_out)

			if (value_now < 1):
				return 0		
		return 0

	def convert_4byte_int(self, byte0, byte1, byte2, byte3):
		int_out = 0
		int_out = byte0 + byte1*256 + byte2*256*256 + byte3*256*256*256
		if int_out > pow(2, 32)/2.:
			int_out = int_out - pow(2, 32)
		return int_out

	# -- 
	def analysisFrame_receivedCAN(self):
		# -- HC
		if self.sdata_CANReceived.idsend == self.ID_HC:
			self.HC_status2.status_rec_can = self.sdata_CANReceived.byte0
			# print(bin(self.data_receivedCAN.byte1))
			self.HC_status2.area1_zone1 = True if self.getBit_fromInt8(self.sdata_CANReceived.byte1, 0) == 0 else False
			self.HC_status2.area1_zone2 = True if self.getBit_fromInt8(self.sdata_CANReceived.byte1, 1) == 0 else False
			self.HC_status2.area1_zone3 = True if self.getBit_fromInt8(self.sdata_CANReceived.byte1, 2) == 0 else False
			self.HC_status2.area1_zone4 = False
			self.HC_status2.area2_zone1 = True if self.getBit_fromInt8(self.sdata_CANReceived.byte2, 0) == 0 else False
			self.HC_status2.area2_zone2 = True if self.getBit_fromInt8(self.sdata_CANReceived.byte2, 1) == 0 else False
			self.HC_status2.area2_zone3 = True if self.getBit_fromInt8(self.sdata_CANReceived.byte2, 2) == 0 else False
			self.HC_status2.area2_zone4 = False
			self.HC_status2.collision_sensor 	= self.sdata_CANReceived.byte3
			self.pub_statusHC2.publish(self.HC_status2)

			# -- HC info
			self.HC_status.status = self.HC_status2.status_rec_can
			if self.HC_status2.area1_zone1:
				self.HC_status.zone_sick_ahead = 1
			elif self.HC_status2.area1_zone2:
				self.HC_status.zone_sick_ahead = 2
			elif self.HC_status2.area1_zone3:
				self.HC_status.zone_sick_ahead = 3
			else:
				self.HC_status.zone_sick_ahead = 0

			if self.HC_status2.area2_zone1:
				self.HC_status.zone_sick_behind = 1
			elif self.HC_status2.area2_zone2:
				self.HC_status.zone_sick_behind = 2
			elif self.HC_status2.area2_zone3:
				self.HC_status.zone_sick_behind = 3
			else:
				self.HC_status.zone_sick_behind = 0

			self.HC_status.vacham = self.HC_status2.collision_sensor
			self.pub_statusHC.publish(self.HC_status)

		if self.sdata_CANReceived.idsend == self.ID_MC:
			self.MC_status.alamp1 		= self.sdata_CANReceived.byte0
			self.MC_status.alamp2  		= self.sdata_CANReceived.byte1
			self.MC_status.dir1 		= self.sdata_CANReceived.byte2
			self.MC_status.dir2 		= self.sdata_CANReceived.byte3
			self.MC_status.speed1 		= self.sdata_CANReceived.byte4
			self.MC_status.speed2 		= self.sdata_CANReceived.byte5
			self.MC_status.mode_operate = self.sdata_CANReceived.byte6
			self.MC_status.is_stop   	= self.sdata_CANReceived.byte7&1
			self.MC_status.sensor_1bit  = 1 if (self.sdata_CANReceived.byte7>>1)&1 == 0 else 0
			# print(self.MC_status)
			self.pub_statusMC.publish(self.MC_status)

		# -- Main
		elif (self.sdata_CANReceived.idsend == self.ID_MAIN):
			self.Power_status.voltages = self.convert_4byte_int(self.sdata_CANReceived.byte0, self.sdata_CANReceived.byte1, 0, 0) / 10.0
			self.Power_status.voltages_analog = 0.0

			self.Power_status.charge_current = float(self.convert_4byte_int(self.sdata_CANReceived.byte2, self.sdata_CANReceived.byte3, 0, 0))
			self.Power_status.charge_analog = 0.0

			self.Power_status.status_btn_reset = int(self.sdata_CANReceived.byte4) 
			self.Power_status.status_btn_power = int(self.sdata_CANReceived.byte5)
			self.Power_status.emg_status = int(self.sdata_CANReceived.byte6)
			
			self.Power_status.status_rec_can = self.sdata_CANReceived.byte7

			self.pub_statusPower.publish(self.Power_status)

		# -- Conveyor
		elif (self.sdata_CANReceived.idsend == self.ID_COVEYOR):
			sensorAhead = 1
			sensorBehind = 2

			self.status_conveyor.mode 			  		= self.data_receivedCAN.byte0
			self.status_conveyor.status 			  	= self.data_receivedCAN.byte1
			self.status_conveyor.sensor_limit_ahead  	= 0 if (self.data_receivedCAN.byte4>>sensorAhead)&1 == 1 else 1
			self.status_conveyor.sensor_limit_behind 	= 0 if (self.data_receivedCAN.byte4>>sensorBehind)&1 == 1 else 1
			self.status_conveyor.sensor_check_rack   	= 0

			self.pub_statusConveyor.publish(self.status_conveyor)

	def syntheticFrame_sendCAN(self):
		# -- HC
		# if (self.CAN_sortSend == 0):
		# 	self.CAN_dataSend.id = self.ID_RTC
		# 	self.CAN_dataSend.byte0 = self.ID_HC
		# 	self.CAN_dataSend.byte1 = self.sdata_hcRequest.rgb1
		# 	self.CAN_dataSend.byte2 = self.sdata_hcRequest.rgb2
		# 	self.CAN_dataSend.byte3 = self.sdata_fieldRequest1.data
		# 	self.CAN_dataSend.byte4 = self.sdata_fieldRequest2.data
		# 	self.CAN_dataSend.byte5 = 0
		# 	self.CAN_dataSend.byte6 = 0
		# 	self.CAN_dataSend.byte7 = 0
		# 	self.CAN_sortSend = 1

		# -- MC
		if (self.CAN_sortSend == 1):
			self.CAN_dataSend.id = self.ID_RTC
			self.CAN_dataSend.byte0 = self.ID_MC
			self.CAN_dataSend.byte1 = self.sdata_mcRequest.dir1
			self.CAN_dataSend.byte2 = self.sdata_mcRequest.dir2
			self.CAN_dataSend.byte3 = self.sdata_mcRequest.speed1
			self.CAN_dataSend.byte4 = self.sdata_mcRequest.speed2
			self.CAN_dataSend.byte5 = self.sdata_mcRequest.mode_operate
			self.CAN_dataSend.byte6 = self.sdata_mcRequest.reset
			self.CAN_dataSend.byte7 = 0
			self.CAN_sortSend = 2

		# -- MC
		elif (self.CAN_sortSend == 2):
			self.CAN_dataSend.id = self.ID_RTC
			self.CAN_dataSend.byte0 = self.ID_MC
			self.CAN_dataSend.byte1 = self.sdata_mcRequest.dir1
			self.CAN_dataSend.byte2 = self.sdata_mcRequest.dir2
			self.CAN_dataSend.byte3 = self.sdata_mcRequest.speed1
			self.CAN_dataSend.byte4 = self.sdata_mcRequest.speed2
			self.CAN_dataSend.byte5 = self.sdata_mcRequest.mode_operate
			self.CAN_dataSend.byte6 = self.sdata_mcRequest.reset
			self.CAN_dataSend.byte7 = 0
			self.CAN_sortSend = 3

		# -- MC
		elif (self.CAN_sortSend == 3):
			self.CAN_dataSend.id = self.ID_RTC
			self.CAN_dataSend.byte0 = self.ID_MC
			self.CAN_dataSend.byte1 = self.sdata_mcRequest.dir1
			self.CAN_dataSend.byte2 = self.sdata_mcRequest.dir2
			self.CAN_dataSend.byte3 = self.sdata_mcRequest.speed1
			self.CAN_dataSend.byte4 = self.sdata_mcRequest.speed2
			self.CAN_dataSend.byte5 = self.sdata_mcRequest.mode_operate
			self.CAN_dataSend.byte6 = self.sdata_mcRequest.reset
			self.CAN_dataSend.byte7 = 0
			self.CAN_sortSend = 4

		# -- Main
		elif (self.CAN_sortSend == 4):
			self.CAN_dataSend.id = self.ID_RTC
			self.CAN_dataSend.byte0 = self.ID_MAIN
			self.CAN_dataSend.byte1 = self.sdata_powerRequest.sound_on
			self.CAN_dataSend.byte2 = self.sdata_powerRequest.sound_type
			self.CAN_dataSend.byte3 = self.sdata_powerRequest.charge
			self.CAN_dataSend.byte4 = self.sdata_powerRequest.emg_reset
			self.CAN_dataSend.byte5 = self.sdata_powerRequest.emg_write
			self.CAN_dataSend.byte6 = self.sdata_powerRequest.led_type
			self.CAN_dataSend.byte7 = 0
			self.CAN_sortSend = 5

		# -- MC
		elif (self.CAN_sortSend == 5):
			self.CAN_dataSend.id = self.ID_RTC
			self.CAN_dataSend.byte0 = self.ID_MC
			self.CAN_dataSend.byte1 = self.sdata_mcRequest.dir1
			self.CAN_dataSend.byte2 = self.sdata_mcRequest.dir2
			self.CAN_dataSend.byte3 = self.sdata_mcRequest.speed1
			self.CAN_dataSend.byte4 = self.sdata_mcRequest.speed2
			self.CAN_dataSend.byte5 = self.sdata_mcRequest.mode_operate
			self.CAN_dataSend.byte6 = self.sdata_mcRequest.reset
			self.CAN_dataSend.byte7 = 0
			self.CAN_sortSend = 6

		# -- MC
		elif (self.CAN_sortSend == 6):
			self.CAN_dataSend.id = self.ID_RTC
			self.CAN_dataSend.byte0 = self.ID_MC
			self.CAN_dataSend.byte1 = self.sdata_mcRequest.dir1
			self.CAN_dataSend.byte2 = self.sdata_mcRequest.dir2
			self.CAN_dataSend.byte3 = self.sdata_mcRequest.speed1
			self.CAN_dataSend.byte4 = self.sdata_mcRequest.speed2
			self.CAN_dataSend.byte5 = self.sdata_mcRequest.mode_operate
			self.CAN_dataSend.byte6 = self.sdata_mcRequest.reset
			self.CAN_dataSend.byte7 = 0
			self.CAN_sortSend = 7

		# -- MC
		elif (self.CAN_sortSend == 7):
			self.CAN_dataSend.id = self.ID_RTC
			self.CAN_dataSend.byte0 = self.ID_MC
			self.CAN_dataSend.byte1 = self.sdata_mcRequest.dir1
			self.CAN_dataSend.byte2 = self.sdata_mcRequest.dir2
			self.CAN_dataSend.byte3 = self.sdata_mcRequest.speed1
			self.CAN_dataSend.byte4 = self.sdata_mcRequest.speed2
			self.CAN_dataSend.byte5 = self.sdata_mcRequest.mode_operate
			self.CAN_dataSend.byte6 = self.sdata_mcRequest.reset
			self.CAN_dataSend.byte7 = 0
			self.CAN_sortSend = 8

		# -- Conveyor
		elif (self.CAN_sortSend == 8):
			self.CAN_dataSend.id = self.ID_RTC
			self.CAN_dataSend.byte0 = self.ID_COVEYOR
			self.CAN_dataSend.byte1 = self.sdata_conveyorControl.no1_mission
			self.CAN_dataSend.byte2 = self.sdata_conveyorControl.no1_speed
			self.CAN_dataSend.byte3 = 0
			self.CAN_dataSend.byte4 = 0
			self.CAN_dataSend.byte5 = 0
			self.CAN_dataSend.byte6 = 0
			self.CAN_dataSend.byte7 = 0
			self.CAN_sortSend = 1

		# -- MC
		# elif (self.CAN_sortSend == 9):
		# 	self.CAN_dataSend.id = self.ID_RTC
		# 	self.CAN_dataSend.byte0 = self.ID_MC
		# 	self.CAN_dataSend.byte1 = self.sdata_mcRequest.dir1
		# 	self.CAN_dataSend.byte2 = self.sdata_mcRequest.dir2
		# 	self.CAN_dataSend.byte3 = self.sdata_mcRequest.speed1
		# 	self.CAN_dataSend.byte4 = self.sdata_mcRequest.speed2
		# 	self.CAN_dataSend.byte5 = self.sdata_mcRequest.mode_operate
		# 	self.CAN_dataSend.byte6 = self.sdata_mcRequest.reset
		# 	self.CAN_dataSend.byte7 = 0
		# 	self.CAN_sortSend = 10

		# # -- MC
		# elif (self.CAN_sortSend == 10):
		# 	self.CAN_dataSend.id = self.ID_RTC
		# 	self.CAN_dataSend.byte0 = self.ID_MC
		# 	self.CAN_dataSend.byte1 = self.sdata_mcRequest.dir1
		# 	self.CAN_dataSend.byte2 = self.sdata_mcRequest.dir2
		# 	self.CAN_dataSend.byte3 = self.sdata_mcRequest.speed1
		# 	self.CAN_dataSend.byte4 = self.sdata_mcRequest.speed2
		# 	self.CAN_dataSend.byte5 = self.sdata_mcRequest.mode_operate
		# 	self.CAN_dataSend.byte6 = self.sdata_mcRequest.reset
		# 	self.CAN_dataSend.byte7 = 0
		# 	self.CAN_sortSend = 11

		# # -- MC
		# elif (self.CAN_sortSend == 11):
		# 	self.CAN_dataSend.id = self.ID_RTC
		# 	self.CAN_dataSend.byte0 = self.ID_MC
		# 	self.CAN_dataSend.byte1 = self.sdata_mcRequest.dir1
		# 	self.CAN_dataSend.byte2 = self.sdata_mcRequest.dir2
		# 	self.CAN_dataSend.byte3 = self.sdata_mcRequest.speed1
		# 	self.CAN_dataSend.byte4 = self.sdata_mcRequest.speed2
		# 	self.CAN_dataSend.byte5 = self.sdata_mcRequest.mode_operate
		# 	self.CAN_dataSend.byte6 = self.sdata_mcRequest.reset
		# 	self.CAN_dataSend.byte7 = 0
		# 	self.CAN_sortSend = 1


	def run(self):
		self.syntheticFrame_sendCAN()
		self.pub_sendCAN.publish(self.CAN_dataSend)

		# -- KILL NODE -- 
		if self.killnode:
			sys.exit(0)


def main():
	# -- Khoi tao ROS
	rclpy.init()
	node = CAN_ROS()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()
		print('Program stopped')

if __name__ == '__main__':
	main()
