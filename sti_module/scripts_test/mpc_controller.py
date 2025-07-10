#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose
from sti_msgs.msg import ListPointRequestMove
from tf.transformations import euler_from_quaternion
from scipy.optimize import minimize
from math import cos, sin, atan2, pi, sqrt
from visualization_msgs.msg import Marker

class MPCController:
    def __init__(self):
        rospy.init_node('mpc_controller', anonymous=True)

        # Parameters
        self.horizon = 5  # Number of prediction steps
        self.dt = 0.14  # Time step for prediction (s)
        self.rho = 1.0  # Weight for control effort in cost function
        self.coefficient_dis = 0.011
        self.coefficient_angle = 0.0008

        self.max_velocity = 1.1  # Maximum linear velocity (m/s)
        self.min_velocity = 0.05
        self.max_angular_velocity = pi / 3  # Maximum angular velocity (rad/s)
        self.cmd_vel = None

        # Subscribers and Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pubMarker = rospy.Publisher('/visualization_markerPoint', Marker, queue_size=10)

        self.pubMarker_robot = rospy.Publisher('/visualization_markerRobot', Marker, queue_size=10)

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.pose_callback)
        rospy.Subscriber('/list_pointRequestMove', ListPointRequestMove, self.path_callback)

        rospy.on_shutdown(self.fnShutDown)

        # State variables
        self.current_pose = Pose()
        self.current_theta = 0.0
        self.path_points = []
        self.is_pose_received = False

        self.lookAheadReferenceByVel = [[self.min_velocity,     0.12],
                                        [0.2,                   0.3],
                                        [0.8,                   1.5],
                                        [self.max_velocity,     1.5]]

    def pose_callback(self, data):
        self.current_pose = data.pose
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        self.current_theta = euler[2]  # Yaw angle
        self.is_pose_received = True

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'pointrobot'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.current_pose.position.x
        marker.pose.position.y = self.current_pose.position.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = self.current_pose.orientation.z
        marker.pose.orientation.w = self.current_pose.orientation.w

        marker.color.r = 0.0
        marker.color.g = 0.
        marker.color.b = 1.
        marker.color.a = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        self.pubMarker.publish(marker)

    def path_callback(self, msg):
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
    
    def calculate_distance(self, x1, x2, y1, y2):
        return sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
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
            

    def mpc_cost(self, u, x, target_point):
        """
        Cost function for MPC optimization.
        """
        cost = 0
        x_pred = x.copy()

        for k in range(self.horizon):            
            v = u[k * 2]
            omega = u[k * 2 + 1]

            # Predict the next state
            x_pred[0] += v * cos(x_pred[2]) * self.dt
            x_pred[1] += v * sin(x_pred[2]) * self.dt
            x_pred[2] += omega * self.dt
                
            # # Tính toán Heading Error (HE)
            theta_point = euler_from_quaternion((0., 0., target_point[2], target_point[3]))[2]
            # print(theta_point,  self.theta_rb_ht)
            theta_e = theta_point - x_pred[2]
            theta_e = (theta_e + pi) % (2 * pi) - pi  # Giới hạn trong khoảng -pi đến pi
        
            cost += self.coefficient_dis*((x_pred[0] - target_point[0])**2 + (x_pred[1] - target_point[1])**2) + self.coefficient_angle*theta_e**2
            
            # Penalize control effort
            # cost += self.rho * (v**2 + omega**2)
        
        # print("cost: ",  cost)
        return cost

    def optimize_control(self, x, target_point):
        """
        Optimize the control inputs using MPC.
        """
        u0 = np.zeros(self.horizon * 2)  # Initial guess for [v1, omega1, ..., vN, omegaN]
        # vel_max = target_point[4]
        vel_max = self.cmd_vel

        bounds = [
            (0.0, vel_max),
            # (-1*vel_max, vel_max),
            (-self.max_angular_velocity, self.max_angular_velocity)
        ] * self.horizon

        # -- lay point target
        result = minimize(
            self.mpc_cost, u0, args=(x, target_point),
            bounds=bounds, method='SLSQP' # BFGS | SLSQP
        )

        if not result.success:
            print(result.fun, result.success)
        
        return result.x[0], result.x[1]  # Return the first [v, omega]

    def calculate_control(self):
        if not self.path_points or not self.is_pose_received:
            return

        # Current state [x, y, theta]
        x = [
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_theta
        ]

        # - Tim diem gan quy dao nhat
        min_dis = float('inf')
        index_near = -1
        for index, p in enumerate(self.path_points):
            d = self.calculate_distance(x[0], p[0], x[1], p[1])
            if min_dis >= d:
                min_dis = d
                index_near = index

        # -- tim look ahead
        # if self.cmd_vel == None:
        self.cmd_vel = self.path_points[index_near][4]

        lookAhead = self.findLookAheadByVel(self.cmd_vel)
        print(self.cmd_vel, lookAhead)

        index_select = -1
        for i in range(index_near + 1, len(self.path_points)*2 - 1, 1):
            index_select = i if i < len(self.path_points) else i - len(self.path_points)

            d = self.calculate_distance(self.path_points[index_near][0], self.path_points[index_select][0],self.path_points[index_near][1], self.path_points[index_select][1])
            if d >= lookAhead:
                break

        target_point = self.path_points[index_select]
        self.pubMakerPointFollow(target_point[0], target_point[1], target_point[2], target_point[3])

        # Apply MPC to get optimal control inputs
        v, omega = self.optimize_control(x, target_point)

        print(v, omega)
        
        # Create and publish the Twist command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    def run(self):
        rate = rospy.Rate(30)  # Control loop frequency (Hz)
        while not rospy.is_shutdown():
            self.calculate_control()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = MPCController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
