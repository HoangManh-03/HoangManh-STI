#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from math import atan2, pi, sqrt, atan, cos, sin, tan
from sti_msgs.msg import ListPointRequestMove
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

# - 0.3: 0.2 0.09874588511470928


class StanleyController:
    def __init__(self):
        rospy.init_node('stanley_controller', anonymous=True)

        # Khởi tạo các thông số
        self.k_e = 8.0  # Hệ số điều chỉnh CTE
        self.linear_velocity = 0.5  # Tốc độ tuyến tính (m/s)

        # Khởi tạo publisher và subscriber
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pubMarker = rospy.Publisher('/visualization_markerPoint', Marker, queue_size=10)
        self.pubMarker_robot = rospy.Publisher('/visualization_markerRobot', Marker, queue_size=10)

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.pose_callback)
        self.poseRbMa = Pose()
        self.theta_rb_ht = 0.0

        rospy.Subscriber('/list_pointRequestMove', ListPointRequestMove, self.path_callback)

        rospy.on_shutdown(self.fnShutDown)

        # Biến lưu trữ thông tin
        self.is_pose = False
        self.path_points = []

        self.max_velocity = 1.1  # Maximum linear velocity (m/s)
        self.min_velocity = 0.05
        self.max_angular_velocity = pi / 3  # Maximum angular velocity (rad/s)
        self.cmd_vel = 0.0

        self.lookAheadReferenceByVel = [[self.min_velocity,     0.12],
                                        # [0.2,                   0.3],
                                        [0.3,                   0.7],
                                        [0.8,                   0.7],
                                        [self.max_velocity,     1.0]]
        
    def pose_callback(self, data):
        # Nhận thông tin từ Odometry và lấy vị trí và góc hiện tại của xe
        # self.is_pose_robot = True
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb_ht = euler[2]
        self.is_pose = True

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'pointrobot'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.poseRbMa.position.x
        marker.pose.position.y = self.poseRbMa.position.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = self.poseRbMa.orientation.z
        marker.pose.orientation.w = self.poseRbMa.orientation.w

        marker.color.r = 0.0
        marker.color.g = 0.
        marker.color.b = 1.
        marker.color.a = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        self.pubMarker.publish(marker)

    def path_callback(self, msg):
        # Cập nhật các điểm quỹ đạo từ ROS topic
        self.path_points = [(point.pose.position.x, point.pose.position.y, point.pose.orientation.z, point.pose.orientation.w, point.velocity) for point in msg.infoPoint]

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        for i in range(2):
            self.cmd_pub.publish(Twist())

    def pubMakerPointFollow(self, x, y, r_z, r_w):
        # Create rviz marker message
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'pointfollow'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = r_z
        marker.pose.orientation.w = r_w

        marker.color.r = 1.0
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        self.pubMarker.publish(marker)

    def euler_from_quaternion(self, quaternion):
        # Chuyển đổi quaternion sang góc Euler (yaw)
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return atan2(siny_cosp, cosy_cosp)
    
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def findLookAheadByVel(self, curr_velocity):
        a = 0.
        b = 0.
        if curr_velocity >= self.max_velocity:
            curr_velocity = self.max_velocity
        elif curr_velocity <= self.min_velocity:
            curr_velocity = self.min_velocity

        for i in range(len(self.lookAheadReferenceByVel) - 1):
            if curr_velocity >= self.lookAheadReferenceByVel[i][0] and curr_velocity <= self.lookAheadReferenceByVel[i+1][0]:
                a = (self.lookAheadReferenceByVel[i][1] - self.lookAheadReferenceByVel[i+1][1])/(self.lookAheadReferenceByVel[i][0] - self.lookAheadReferenceByVel[i+1][0])
                b = self.lookAheadReferenceByVel[i][1] - a*self.lookAheadReferenceByVel[i][0]

        return a*curr_velocity + b
    
    def calculate_control(self):
        if not self.path_points:
            return

        # - Tim diem gan quy dao nhat
        min_dis = float('inf')
        index_near = -1
        for index, p in enumerate(self.path_points):
            d = self.fnCalcDistPoints(self.poseRbMa.position.x, p[0], self.poseRbMa.position.y, p[1])
            if min_dis >= d:
                min_dis = d
                index_near = index

        self.cmd_vel = self.path_points[index_near][4]

        # lookAhead = self.findLookAheadByVel(self.cmd_vel)
        # print(self.cmd_vel, lookAhead)

        # index_select = -1
        # for i in range(index_near + 1, len(self.path_points)*2 - 1, 1):
        #     index_select = i if i < len(self.path_points) else i - len(self.path_points)

        #     d = self.fnCalcDistPoints(self.path_points[index_near][0], self.path_points[index_select][0],self.path_points[index_near][1], self.path_points[index_select][1])
        #     if d >= lookAhead:
        #         break

        point_select = self.path_points[index_near]
        self.pubMakerPointFollow(point_select[0], point_select[1], point_select[2], point_select[3])

        # # Tính toán Heading Error (HE)
        theta_point = euler_from_quaternion((0., 0., point_select[2], point_select[3]))[2]
        # print(theta_point,  self.theta_rb_ht)
        theta_e = theta_point - self.theta_rb_ht
        theta_e = (theta_e + pi) % (2 * pi) - pi  # Giới hạn trong khoảng -pi đến pi

        # - Tinh toan sai so khoang cach
        # dx = self.poseRbMa.position.x - point_select[0]
        # dy = self.poseRbMa.position.y - point_select[1]
        # ss_dis = dy*cos(theta_point) - dx*sin(theta_point)

        # x = (self.poseRbMa.position.x - point_select[0])*cos(theta_point) + (self.poseRbMa.position.y - point_select[1])*sin(theta_point)
        y = -(self.poseRbMa.position.x - point_select[0])*sin(theta_point) + (self.poseRbMa.position.y - point_select[1])*cos(theta_point)
        ss_dis = y

        self.k_e = 0.6
        L = 0.4 #0.532 L cành nhỏ đáp ứng góc càng nhanh
        
        # # Tính toán góc lái theo công thức Stanley
        # delta = theta_e + atan2(self.k_e * ss_dis * (-1) + theta_e, point_select[4])
        delta = theta_e + atan2(self.k_e * ss_dis * (-1), point_select[4])
        print("delta: %s, ss_dis: %s" %(delta, ss_dis))

        w = (point_select[4]/L)*tan(delta)
        # w = point_select[4]*delta

        # w = self.k_e * ss_dis * (-1) +  point_select[4] * sin(theta_e)

        # # Tạo lệnh điều khiển
        cmd = Twist()
        cmd.linear.x = point_select[4]  # Tốc độ tuyến tính
        # cmd.linear.x = self.linear_velocity # Tốc độ tuyến tính
        cmd.angular.z = w  # Tốc độ góc (điều khiển góc lái)

        print(cmd.linear.x, cmd.angular.z)
        
        # # Gửi lệnh lên ROS
        self.cmd_pub.publish(cmd)

    def run(self):
        rate = rospy.Rate(30)  # Tần suất điều khiển 10Hz
        while not rospy.is_shutdown():
            if self.is_pose:
                self.calculate_control()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = StanleyController()
        controller.run()

    except rospy.ROSInterruptException:
        pass
