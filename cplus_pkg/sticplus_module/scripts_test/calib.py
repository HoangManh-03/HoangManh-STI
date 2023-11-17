# import numpy as np
# import matplotlib.pyplot as plt
# from scipy import interpolate
# x = np.array([0.008, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8])
# y = np.array([0.11, 0.22, 0.3, 0.4, 0.55, 0.65, 0.7, 0.75, 0.8, 0.9, 1., 1.2, 1.25, 1.3, 1.35, 1.4])
# f = interpolate.interp1d(x, y)

# xnew = 0.18
# ynew = f(xnew)   # use interpolation function returned by `interp1d`
# # plt.plot(x, y, 'o', xnew, ynew, '-')
# # plt.show()

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi as PI

class SUB():
    def __init__(self):
        rospy.init_node('test', anonymous=False)
        print("initial node!")
        self.pubMarker = rospy.Publisher('/visualization_markerAGV', Marker, queue_size=20)
        self.pubMarker2 = rospy.Publisher('/visualization_markerpoint', Marker, queue_size=20)
        # rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback)
        self.rate = rospy.Rate(15)

    def callback(self, data):
        marker = Marker()
        marker.header.frame_id = "frame_map_nav350"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = "file:///home/hoang/Desktop/Car.dae"
        marker.pose.position.x = data.pose.position.x
        marker.pose.position.y = data.pose.position.y
        marker.pose.position.z = -1.

        (roll, pitch, yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        yaw = yaw - 3.*PI/2.
        q = quaternion_from_euler(roll, pitch, yaw)

        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        # marker.color.r = 1.0
        # marker.color.g = 0.
        # marker.color.b = 0.
        marker.color.a = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        self.pubMarker.publish(marker)

    def run(self):
        while not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "frame_map_nav350"
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'pointfollow'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = 9.989
            marker.pose.position.y = -1.501
            marker.pose.position.z = -1.
            marker.pose.orientation.w = 1.
            marker.color.r = 1.0
            marker.color.g = 0.
            marker.color.b = 0.
            marker.color.a = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            self.pubMarker2.publish(marker)

            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = SUB()
    class_1.run()
    # class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()