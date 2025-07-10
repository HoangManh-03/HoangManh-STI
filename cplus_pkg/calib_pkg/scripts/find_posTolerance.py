#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math 

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_pkg.msg import *
from sti_msgs.msg import *

class Find_poseTolerance():
	def __init__(self):
		print("ROS Initial: Find_poseTolerance!")
		rospy.init_node('Find_poseTolerance', anonymous = False, disable_signals=True) # False

		self.rate = rospy.Rate(10)

		# -- Keyboard command 
		rospy.Subscriber("/Keyboard_cmd", Keyboard_command, self.callback_Keyboard) # 
		self.keyboard_cmd = Keyboard_command()	

		# -- robot_pos -- 
		rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback_getPose)
		self.robot_pose = PoseStamped()

		# -- CONST --
		self.LENGHTOFARR = 100
		self.DONE_JOB = 0
		self.FIND_TOLPOS_JOB = 1
		self.FIND_POSDIR1_JOB = 2       # Đưa ra tọa độ theo hướng của AGV
		self.FIND_POSDIR2_JOB = 3       # Đưa ra sai số theo hướng của AGV

		# -- VAR -- 
		self.input = ""
		self.index = 0
		self.PosXarr = []
		self.PosYarr = []
		self.PosZarr = []
		self.sys_job = 0

		self.robotPose_max = [0,0,0]
		self.robotPose_min = [0,0,0]

		self.robotPose_mean1 = [0,0,0]
		self.robotPose_mean2 = [0,0,0]

		self.euler = 0.0

		self.tolX12 = 0
		self.tolY12 = 0

		self.toleranceX = 0
		self.toleranceY = 0
		self.toleranceZ = 0
		self.is_exit = 0

		self.flag_tolpos = 0
		self.flag_posdir1 = 0
		self.flag_posdir2 = 0
		self.flag_reset = 0	
		self.flag_restart = 0

		self.completed_reset = 0

		self.enb_log = 0
		self.step_log = 0
		self.ct_log = rospy.get_time()

	def log_mess(self, mess):
		if self.enb_log == 1:
			if self.step_log == 0:
				self.ct_log = rospy.get_time()
				self.step_log = 1
				rospy.loginfo(mess)
			else:
				if rospy.get_time() - self.ct_log >= self.TIME_LOG:
					rospy.loginfo(mess)
					self.ct_log = rospy.get_time()
		else:
			self.step_log = 0

	def callback_Keyboard(self, data):
		self.keyboard_cmd = data
		if self.keyboard_cmd.value == "":
			self.flag_reset = 0	

		elif self.keyboard_cmd.value == "tolpos":
			self.flag_tolpos = 1

		elif self.keyboard_cmd.value == "posdir1":
			self.flag_posdir1 = 1
		
		elif self.keyboard_cmd.value == "posdir2":
			self.flag_posdir2 = 1

		elif self.keyboard_cmd.value == "reset":
			self.flag_reset = 1						

		elif self.keyboard_cmd.value == "restart":
			self.flag_restart = 1

	def callback_getPose(self, dat):
		self.robot_pose = dat
		# doi quaternion -> rad    
		quaternion1 = (dat.pose.orientation.x, dat.pose.orientation.y,\
					dat.pose.orientation.z, dat.pose.orientation.w)
		self.euler = tf.transformations.euler_from_quaternion(quaternion1)

	"""
		>> Return position tolerance of robot
	"""
	def get_tolpos(self):
		if self.index < self.LENGHTOFARR:
			self.PosXarr.append(self.robot_pose.pose.position.x)
			self.PosYarr.append(self.robot_pose.pose.position.y)
			self.PosZarr.append(self.euler[2])
			self.index = self.index + 1
			print("JOB Find tolerance of pose is still running" , self.index)
		else:
			self.robotPose_max = [max(self.PosXarr), max(self.PosYarr), max(self.PosZarr)]
			self.robotPose_min = [min(self.PosXarr), min(self.PosYarr), min(self.PosZarr)]

			self.toleranceX = self.robotPose_max[0] - self.robotPose_min[0]
			self.toleranceY = self.robotPose_max[1] - self.robotPose_min[1]
			self.toleranceZ = self.robotPose_max[2] - self.robotPose_min[2]

			rospy.loginfo("robot pose tolerance, x = %f, y = %f, z = %f", self.toleranceX, self.toleranceY, self.toleranceZ)

			self.index = 0
			self.PosXarr = []
			self.PosYarr = []
			self.Poszarr = []
			self.flag_tolpos = 0	

	"""
		>> Return mean position of robot in 1st direction
	"""
	def get_posdir1(self):
		if self.index < self.LENGHTOFARR:
			self.PosXarr.append(self.robot_pose.pose.position.x)
			self.PosYarr.append(self.robot_pose.pose.position.y)
			self.PosZarr.append(self.euler[2])
			self.index = self.index + 1
			print("JOB find pos robot is still running" , self.index)
		else:
			self.robotPose_mean1[0] = sum(self.PosXarr)/len(self.PosXarr)
			self.robotPose_mean1[1] = sum(self.PosYarr)/len(self.PosYarr)
			self.robotPose_mean1[2] = sum(self.PosZarr)/len(self.PosZarr)
			rospy.loginfo("robot pose 1 in %f degree, x = %f, y = %f", self.robotPose_mean1[2]*180/math.pi, self.robotPose_mean1[0], self.robotPose_mean1[1])
			self.flag_posdir1 = 0
			self.index = 0
			self.PosXarr = []
			self.PosYarr = []
			self.Poszarr = []
	"""
		>> Return mean position of robot in 2nd direction
		>> Return tolerance pos of robot between 1st and 2nd direction.
	"""
	def get_posdir2(self):
		if self.index < self.LENGHTOFARR:
			self.PosXarr.append(self.robot_pose.pose.position.x)
			self.PosYarr.append(self.robot_pose.pose.position.y)
			self.PosZarr.append(self.euler[2])
			self.index = self.index + 1
			print("JOB find pos robot is still running" , self.index)
		else:
			self.index = 0
			self.robotPose_mean2[0] = sum(self.PosXarr)/len(self.PosXarr)
			self.robotPose_mean2[1] = sum(self.PosYarr)/len(self.PosYarr)
			self.robotPose_mean2[2] = sum(self.PosZarr)/len(self.PosZarr)
			rospy.loginfo("robot pose 2 in %f degree, x = %f, y = %f", self.robotPose_mean2[2]*180/math.pi, self.robotPose_mean2[0], self.robotPose_mean2[1])

			self.tolX12 = self.robotPose_mean2[0] - self.robotPose_mean1[0]
			self.tolY12 = self.robotPose_mean2[1] - self.robotPose_mean1[1]
			rospy.loginfo("robot pos tolerance 12   , x = %f cm, y = %f cm", self.tolX12*100, self.tolY12*100)
			self.flag_posdir2 = 0

			self.index = 0
			self.PosXarr = []
			self.PosYarr = []
			self.Poszarr = []
			
	def reset(self):
		self.completed_reset = 1

		self.flag_tolpos = 0
		self.flag_posdir1 = 0
		self.flag_posdir2 = 0
		self.flag_reset = 0	
		self.step_log = 0
		self.enb_log = 0

		self.index = 0
		self.PosXarr = []
		self.PosYarr = []
		self.Poszarr = []
		print("Reset hệ thống!")

	def restart(self):
		self.completed_reset = 0
		self.flag_restart = 0
		print("Restart hệ thống!")		

	def shutdown(self):
		self.is_exit = 1

	def run(self):
		try:
			if self.is_exit == 0:
				while not rospy.is_shutdown():
					if self.flag_restart == 1:
						self.restart()

					if self.flag_tolpos == 1:
						self.get_tolpos()
					
					if self.flag_posdir1 == 1:
						self.get_posdir1()

					if self.flag_posdir2 == 1:
						self.get_posdir2()

					if self.flag_reset == 1 and self.completed_reset == 0:
						self.reset()

					self.rate.sleep()
		except KeyboardInterrupt:
			rospy.on_shutdown(self.shutdown)
			self.is_exit = 1
			print('!!FINISH!!')

def main():
	print('Starting main program')
	program = Find_poseTolerance()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()
    