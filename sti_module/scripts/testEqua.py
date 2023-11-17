import roslib
import copy
import sys
import signal
import rospy
from geometry_msgs.msg import PoseStamped, Pose

pose1 = Pose()
pose1.position.x = 1
pose1.position.y = 0

pose2 = copy.deepcopy(pose1)
pose2.position.x = 10

pose1.position.y = 100

print(pose1)
print(pose2)

