#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dev: Archie Phung
Date Modify: 2/4/2025
"""

import rospy
from std_msgs.msg import *
import requests

class R2000_Scanconfig():
	def __init__(self):
		print("ROS Initial: R2000 config filter for Scan!")
		rospy.init_node('keyboard_command', anonymous = False, disable_signals=True) # False

		self.rate = rospy.Rate(10)
		# -
		self.pub_keyboard_command = rospy.Publisher("/Keyboard_cmd", String, queue_size = 20)	# Dieu khien Toyo.
		self.Keyboard_command = String()

		# -- CONST --
		self.TIME_SAVEDATA = 2
		# -- VAR -- 
		self.input = ""
		self.ct_sendcmd = rospy.get_time()
		self.step = 0
		self.is_exit = 0

		self.lidar_ip = rospy.get_param("~lidar_ip", '169.254.32.205')

	def shutdown(self):
		self.is_exit = 1

	def run(self):
		try:
			if self.is_exit == 0:
				while not rospy.is_shutdown():
					if self.input == "":
						self.input = input("Please enter command: ")
					
					else:
						if self.input == "t":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_type=remission"

						if self.input == "reset":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_type=none"

						elif self.input == "w2":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_width=2"
						
						elif self.input == "w4":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_width=4"

						elif self.input == "w8":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_width=8"

						elif self.input == "w16":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_width=16"

						elif self.input == "errtole":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_error_handling=tolerant"

						elif self.input == "errs":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_error_handling=strict"

						elif self.input == "reflow":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_remission_threshold=reflector_low"

						elif self.input == "refstd":
							command_url = f"http://{self.lidar_ip}/cmd/set_parameter?filter_remission_threshold=reflector_std"

						try:
							# Gửi yêu cầu GET đến LiDAR
							response = requests.get(command_url)

							# In phản hồi từ LiDAR
							print("Phản hồi từ LiDAR:", response.json())  # Nếu phản hồi là JSON

						except requests.exceptions.RequestException as e:
							print(f"Lỗi kết nối: {e}")
							return

						self.input = ''

					self.rate.sleep()
		except KeyboardInterrupt:
			rospy.on_shutdown(self.shutdown)
			self.is_exit = 1
			print('!!FINISH!!')

def main():
	print('Starting main program')
	program = R2000_Scanconfig()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()
    