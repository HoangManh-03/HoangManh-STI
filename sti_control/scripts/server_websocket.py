# DucAnhLe

import websockets
import asyncio
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import json
import signal
import sys

# Tỉ lệ điều chỉnh cho vận tốc
LINEAR_SCALE = 0.5
ANGULAR_SCALE = 0.5

class WebSocketServer:
    def __init__(self, host="127.0.0.1", port=8080):
        rospy.init_node("websocket_joystick", anonymous=True)
        self.host = host
        self.port = port
        self.time_tr = rospy.get_time()
        self.rate_pubVel = 30
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0

    async def handle_connection(self, websocket, path):
        print("Client connected")

        try:
            async for message in websocket:

                try:
                    json_str = message[1:-1]
                    print(json_str)
                    key_value_pairs = json_str.split(", ")
                    data = {}
                    for pair in key_value_pairs:
                        key, value = pair.split(": ")
                        data[key] = float(value)

                    self.x = data['dx']
                    self.y = data['dy']

                    print(f"x: {self.x}, y: {self.y}")

                except json.JSONDecodeError:
                    self.x = -99999
                    self.y = -99999
                    print("Invalid JSON data received")

                await websocket.send(f"Echo: {message}")
        except websockets.ConnectionClosed:
            print("Client disconnected")


            ###########
    async def run_ros_logic(self):
        rate = rospy.Rate(self.rate_pubVel)
        while not rospy.is_shutdown():

            if self.x > -99998:

                a = -self.y/50
                b = -self.x/50

                vel = Twist()
                vel.linear.x = LINEAR_SCALE * a
                vel.angular.z = ANGULAR_SCALE * b

                if abs(vel.linear.x) > 0.0001 or abs(vel.angular.z) > 0.0001:
                    self.pub.publish(vel)
                else:
                    self.pub.publish(Twist())

            await asyncio.sleep(1 / self.rate_pubVel)


    # async def start_server(self):
    #     start_server = websockets.serve(self.handle_connection, self.host, self.port, ping_interval=None)
    #     await asyncio.gather(
    #         start_server,
    #         self.run_ros_logic()
    #     )
    async def start_server(self):
        start_server = await websockets.serve(self.handle_connection, self.host, self.port, ping_interval=None)
        await asyncio.gather(
            asyncio.create_task(self.run_ros_logic()),
            start_server.wait_closed()
        )


    def stop_server(self):
        if self.server:
            print("Shutting down WebSocket server...")
            self.server.close()  # Dừng server WebSocket
            print("WebSocket server stopped.")

    def run(self):
        def signal_handler(sig, frame):
            print("\nCtrl+C detected, shutting down...")
            self.stop_server()
            sys.exit(0)  # Thoát chương trình an toàn

        # Đăng ký bắt tín hiệu SIGINT (Ctrl+C)
        signal.signal(signal.SIGINT, signal_handler)

        try:
            asyncio.run(self.start_server())
        except rospy.ROSInterruptException:
            pass


# Sử dụng lớp WebSocketServer
if __name__ == "__main__":
    server = WebSocketServer(host="192.168.1.55", port=8080)
    server.run()
