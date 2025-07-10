#!/usr/bin/env python3
import rospy
import sys
import time
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# from sti_msgs.msg import Move_query, Move_respond
# from sti_msgs.msg import Tag_node, Zone_safety, Tag_info

from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees, acos, fabs
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Twist, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int16


class rickshaw_navigation():
	def __init__(self):
		print("ROS Initial: navigation!")
		rospy.init_node('rickshaw_navigation', anonymous = False) # False

		self.control_frequence = 15
		self.rate = rospy.Rate(self.control_frequence)
		# --
		rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback_robotPoseNAV)
		self.robotPoseNAV = PoseStamped()
		self.is_robotPoseNAV = 0


	def callback_robotPoseNAV(self, data):
		self.robotPoseNAV = data
		self.is_robotPoseNAV = 1

	def convert_relative_coordinates(self, X_cv, Y_cv, pose_end):
		angle = -self.quaternion_to_euler(pose_end.orientation)
		_X_cv = (X_cv - pose_end.position.x)*cos(angle) - (Y_cv - pose_end.position.y)*sin(angle)
		_Y_cv = (X_cv - pose_end.position.x)*sin(angle) + (Y_cv - pose_end.position.y)*cos(angle)
		return _X_cv, _Y_cv


	def calculateAngle_twoPoint(self, point_1, point_2): # point_1 | point_2 | geometry_msgs/Point | out | radian
		delta_x = point_2.x - point_1.x
		delta_y = point_2.y - point_1.y
		if (delta_x == 0):
			if (delta_y >= 0):
				return pi/2.
			else:
				return pi*(-1/2.)
		else:
			return atan2(delta_y, delta_x)

	def angleLine_AB(self, pointA, pointB): # -- Angle Line Point A to Point B. | Point()
		a = pointA.y - pointB.y
		b = pointB.x - pointA.x
		ang = 0.0
		if b == 0:
			if a < 0:
				ang = pi/2.0
			elif a > 0:
				ang = -pi/2.0
		elif a == 0:
			if -b < 0:
				ang = 0.0
			elif -b > 0:
				ang = pi
		else:
			ang = acos(b/sqrt(b*b + a*a))
			if -a/b > 0:
				if fabs(ang) > pi/2:
					ang = -ang
				else:
					ang = ang
			else:
				if fabs(ang) > pi/2:
					ang = ang
				else:
					ang = -ang
		return ang

	def limmit_corner(self, angle_in):
		angle_out = 0.0
		angle_out = angle_in
		if (abs(angle_in) >= pi):
			if (angle_in >= 0):
				angle_out = (pi*2 - abs(angle_in))*(-1)
			else:
				angle_out = pi*2 - abs(angle_in)		
		return angle_out

	def limitAngle(self, angle_in): # - rad
		qua_in = self.euler_to_quaternion(angle_in)
		angle_out = self.quaternion_to_euler(qua_in)
		return angle_out

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)
		
	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def euler_to_quaternion(self, euler):
		quat_out = Quaternion()
		quat = quaternion_from_euler(0, 0, euler)
		quat_out.x = quat[0]
		quat_out.y = quat[1]
		quat_out.z = quat[2]
		quat_out.w = quat[3]
		return quat_out
		
	def differrence_from_targetSpecial(self, pose_first, pose_end):
		pose_out = Pose()
		angle_first = self.quaternion_to_euler(pose_first.orientation)
		# print ("angle_first: ", degrees(angle_first) )
		angle_end = self.quaternion_to_euler(pose_end.orientation)
		# print ("angle_end: ", degrees(angle_end) )
		angle_end_to_first = self.calculateAngle_twoPoint(pose_end.position, pose_first.position)
		# print ("angle_end_to_first: ", degrees(angle_end_to_first) )
		# --
		deltaDis = self.calculate_distance(pose_first.position, pose_end.position)
		# print ("deltaDis: ", deltaDis)
		# -- 
		deltaAngle_end_with_etf = angle_end_to_first - angle_end #- angle_end_to_first
		# print ("deltaAngle_end_with_etf1: ", degrees(deltaAngle_end_with_etf) )
		deltaAngle_end_with_etf = self.limmit_corner(deltaAngle_end_with_etf)
		# print ("deltaAngle_end_with_etf: ", degrees(deltaAngle_end_with_etf) )
		
		# --
		deltaAngle_end_with_first = angle_first - angle_end
		deltaAngle_end_with_first = self.limmit_corner(deltaAngle_end_with_first)
		# print ("deltaAngle_end_with_first: ", degrees(deltaAngle_end_with_first) )
		# --
		pose_out.position.x = deltaDis*cos(deltaAngle_end_with_etf)
		pose_out.position.y = deltaDis*sin(deltaAngle_end_with_etf)

		print ("x: ", pose_out.position.x)
		print ("y: ", pose_out.position.y)
		print ("deltaAngle_end_with_first: ", degrees(deltaAngle_end_with_first) )

		theta_x = pose_out.position.x
		theta_y = pose_out.position.y

		# return theta_x, theta_y, theta_g


	def fakePose_robotFollowGoal(self, robotPose, goalPose): # -- goal -> main.
		distancePoint = self.calculate_distance(robotPose.position, goalPose.position)
		# --
		angle_goalToRobot = self.angleLine_AB(goalPose.position, robotPose.position)
		# angle_robotToGoal = angleLine_AB(robotPose.position, goalPose.position)
		# --
		angleGoal = self.quaternion_to_euler(goalPose.orientation)
		angleRobot = self.quaternion_to_euler(robotPose.orientation)
		# -- 
		# print ("angleRobot: ", degrees(angleRobot))
		# print ("angleGoal: ", degrees(angleGoal))
		# print ("angle_goalToRobot: ", degrees(angle_goalToRobot))
		# print ("angle_robotToGoal: ", degrees(angle_robotToGoal))
		# -- 
		angleNew_goalToRobot = angle_goalToRobot - angleGoal
		angleNew_goalToRobot = self.limitAngle(angleNew_goalToRobot)
		# -- 
		# print ("angleNew_goalToRobot: ", degrees(angleNew_goalToRobot) )
		# -- 
		angleRobot_new = angleRobot - angleGoal
		angleRobot_new = self.limitAngle(angleRobot_new)
		# -- 
		poseRobot_new = Pose()
		poseRobot_new.position.x = cos(angleNew_goalToRobot)*distancePoint
		poseRobot_new.position.y = sin(angleNew_goalToRobot)*distancePoint
		poseRobot_new.orientation = self.euler_to_quaternion(angleRobot_new)

		poseGoal_new = Pose()
		poseGoal_new.orientation = self.euler_to_quaternion(0)

		# print ("poseRobot_new X: ", poseRobot_new.position.x)
		# print ("poseRobot_new Y: ", poseRobot_new.position.y)
		# print ("poseRobot_new R: ", degrees(angleRobot_new) )

		print ("x: ", poseRobot_new.position.x)
		print ("y: ", poseRobot_new.position.y)
		print ("deltaAngle_end_with_first: ", degrees(angleRobot_new) )


	def my_fakePose_robotFollowGoal(self, robotPose, goalPose): # -- goal -> main.-
		angleGoal = self.quaternion_to_euler(goalPose.orientation)
		angleRobot = self.quaternion_to_euler(robotPose.orientation)
		# -- 
		# print ("angleRobot: ", degrees(angleRobot))
		# print ("angleGoal: ", degrees(angleGoal))
		# print ("angle_goalToRobot: ", degrees(angle_goalToRobot))
		# print ("angle_robotToGoal: ", degrees(angle_robotToGoal))
		# -- 
		# -- 
		# print ("angleNew_goalToRobot: ", degrees(angleNew_goalToRobot) )
		# -- 
		# angleRobot = 1.58
		# angleGoal = 1.57

		angleRobot_new = angleRobot - angleGoal
		angleRobot_new = self.limitAngle(angleRobot_new)
		# -- 
		poseRobot_new = Pose()
		poseRobot_new.position.x, poseRobot_new.position.y = self.convert_relative_coordinates(robotPose.position.x, robotPose.position.y, goalPose)
		poseRobot_new.orientation = self.euler_to_quaternion(angleRobot_new)

		poseGoal_new = Pose()
		poseGoal_new.orientation = self.euler_to_quaternion(0)

		# print ("poseRobot_new X: ", poseRobot_new.position.x)
		# print ("poseRobot_new Y: ", poseRobot_new.position.y)
		# print ("poseRobot_new R: ", degrees(angleRobot_new) )

		print ("x: ", poseRobot_new.position.x)
		print ("y: ", poseRobot_new.position.y)
		print ("deltaAngle_end_with_first: ", degrees(angleRobot_new) )

	def run(self):
		poseTarget = Pose()
		poseTarget.position.x = 3.596
		poseTarget.position.y = -3.931
		poseTarget.orientation = self.euler_to_quaternion(radians(90))

		poseRobot= Pose()
		poseRobot.position.x = 3.596
		poseRobot.position.y = -3.931
		poseRobot.orientation = self.euler_to_quaternion(radians(90))
		while not rospy.is_shutdown():
			if self.is_robotPoseNAV == 1:
				self.my_fakePose_robotFollowGoal(self.robotPoseNAV.pose, poseTarget)
				print("------------------------------")
				self.fakePose_robotFollowGoal(self.robotPoseNAV.pose, poseTarget)
				print("------------------------------")

			self.rate.sleep()

	def test(self):
		poseTarget = Pose()
		poseTarget.position.x = 4.739
		poseTarget.position.y = -175.183
		poseTarget.orientation = self.euler_to_quaternion(radians(90))

		poseRobot = Pose()
		poseRobot.position.x = 4.78
		poseRobot.position.y = -176.864
		poseRobot.orientation = self.euler_to_quaternion(radians(90))
		self.fakePose_robotFollowGoal(poseRobot, poseTarget)

def main():
	print('Starting main program')
	program = rickshaw_navigation()
	program.test()
	# print('Exiting main program')
	# program.stop_run()

if __name__ == '__main__':
    main()
