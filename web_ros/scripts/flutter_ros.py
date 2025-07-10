import websockets
import asyncio
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from sti_msgs.msg import *
from message_pkg import *

# Tỉ lệ điều chỉnh cho vận tốc
LINEAR_SCALE = 0.5
ANGULAR_SCALE = 0.5

# class ROSPub:
#     def __init__(self):
#         rospy.init_node("monitor", anonymous=True)

#         self.x = 0
#         self.y = 0
#         self.rate_pubVel = 30
#         self.time_tr = rospy.get_time()

#         self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
#         self.pub_main = rospy.Publisher("/POWER_request",POWER_request ,queue_size=10)
        
#         rospy.Subscriber("/web_info", Float32MultiArray, self.joyCallback)
#         rospy.Subscriber("/POWER_info", POWER_info, self.callback_PowerInfo)
#         rospy.Subscriber("NN_infoRespond", ,self.callback_NN_Info_Respond)

#         self.joy = Float32MultiArray().data
#         self.info_power = POWER_info()
#         self.infoRespond = NN_infoRespond()
        
#     def joyCallback(self, msg):
#         self.joy = msg.data 

#     def callback_PowerInfo(self, msg):
#         self.info_power = msg

#     def callback_NNInfoRespond(self,msg):
#         self.infoRespond = msg

#     def callback_NAV_pose(self,msg):
#         self.NAV_pose = msg
    
#     def callback_HC_info(self, msg):
#         self.HC_info = msg


#     def pub_cmdVel(self, twist, rate):
#         if rospy.get_time() - self.time_tr > float(1 / rate):
#             self.time_tr = rospy.get_time()
#             self.pub.publish(twist)

#     def run(self):
#         rate = rospy.Rate(self.rate_pubVel)

#         while not rospy.is_shutdown():
#             if len(self.joy) >= 5:  # Đảm bảo rằng self.joy có ít nhất 5 phần tử
#                 x = self.joy[2]  # Truy cập đến dữ liệu trong self.joy
#                 y = self.joy[3]
#                 w = self.joy[4]

#                 vel = Twist()
#                 vel.linear.x = LINEAR_SCALE * (-y / w)
#                 vel.angular.z = ANGULAR_SCALE * (-x / w)

#                 if abs(vel.linear.x) > 0.0001 or abs(vel.angular.z) > 0.0001:
#                     self.pub_cmdVel(vel, self.rate_pubVel)
#                 else:
#                     vel_0 = Twist()
#                     self.pub_cmdVel(vel_0, self.rate_pubVel)
#             else:
#                 rospy.logwarn("self.joy không đủ phần tử: {}".format(len(self.joy)))

#             rate.sleep()

# async def server(websocket, path):
#     print("Client connected")
#     ros_pub = ROSPub()

#     async for message in websocket:
#         print(f"Received: {message}")

#         # Gửi dữ liệu từ ROS qua WebSocket
#         rospy.sleep(0.1)  # Để đảm bảo dữ liệu ROS ổn định
#         if ros_pub.joy:
#             await websocket.send(str(ros_pub.joy))

#     print("Client disconnected")

# # Chạy server trên port 8080
# start_server = websockets.serve(server, "192.168.1.55", 8080, ping_interval=None)

# asyncio.get_event_loop().run_until_complete(start_server)
# asyncio.get_event_loop().run_forever()

class ROSPub:
    def __init__(self):
        rospy.init_node("websocket_server", anonymous=True)

        self.joy = Float32MultiArray().data
        self.twist = None
        self.odom = None
        self.battery = None

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.twist_callback)
        rospy.Subscriber("/robotPose_nav", Odometry, self.odom_callback)
        rospy.Subscriber("/battery", Float32, self.battery_callback)

    def joy_callback(self, msg):
        self.joy = {"axes": msg.axes, "buttons": msg.buttons}

    def twist_callback(self, msg):
        self.twist = {"linear": msg.linear.x, "angular": msg.angular.z}

    def odom_callback(self, msg):
        self.odom = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "theta": msg.pose.pose.orientation.z,
        }

    def battery_callback(self, msg):
        self.battery = msg.data

async def server(websocket, path):
    print("Client connected")
    ros_pub = ROSPub()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)  # Giữ ổn định ROS

        data = {
            "joy": ros_pub.joy,
            "twist": ros_pub.twist,
            "odom": ros_pub.odom,
            "battery": ros_pub.battery,
        }

        # Chỉ gửi nếu có dữ liệu hợp lệ
        filtered_data = {k: v for k, v in data.items() if v is not None}
        if filtered_data:
            await websocket.send(json.dumps(filtered_data))

async def main():
    async with websockets.serve(server, "0.0.0.0", 8765):
        await asyncio.Future()  # Chạy mãi mãi

if __name__ == "__main__":
    asyncio.run(main())
