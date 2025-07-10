#!/usr/bin/env python3

import asyncio
import signal
import rospy
import websockets
import rospy
import json
from geometry_msgs.msg import Twist, PoseStamped,TwistWithCovarianceStamped
from sti_msgs.msg import *
from message_pkg.msg import *
from std_msgs.msg import String, Int16, Bool


class WebSocketServer:

    def __init__(self):
        rospy.init_node("websocket_server", anonymous=True)

        # Tạo WebSocket server
        # self.clients = set()
        self.server = None

        # Lắng nghe topic cmd_vel
        self.publisher = rospy.Publisher("websocket_data", String, queue_size=10)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.pub_cancelMission = rospy.Publisher("/cancelMission_control", Int16, queue_size = 4)
        self.pub_POWER_request = rospy.Publisher("/POWER_request", POWER_request, queue_size = 10)
        self.publish_lifting = rospy.Publisher("/lift_control", Lift_control, queue_size = 10)
        self.pub_lift = Lift_control()
        self.pub_appbutton = rospy.Publisher("app_button", App_button, queue_size = 10)
        self.app_button = App_button()

        self.check_sound = 0;

        rospy.Subscriber("POWER_request", POWER_request, self.callback_POWER_request)
        self.request_power = POWER_request()
        # phanh
        rospy.Subscriber("/enable_brake", Bool, self.callback_brakeControl) 
        self.status_brake = Bool()
        # -- Safety Zone
        rospy.Subscriber("/safety_zone", Zone_lidar_2head, self.callback_safetyZone) 
        self.safety_zone = Zone_lidar_2head()
                # -- HC
        rospy.Subscriber("/HC_info", HC_info, self.callback_HC) 
        self.HC_info = HC_info()
        # -- Main
        rospy.Subscriber("/POWER_info", POWER_info, self.callback_Main) 
        self.main_info = POWER_info()

        # -- OC board
        rospy.Subscriber("/lift_status", Lift_status, self.callback_OC_board) # lay thong tin trang thai mach dieu khien ban nang.
        self.OC_status = Lift_status()

        # -- Status Port
        rospy.Subscriber("/status_port", Status_port, self.callback_statusPort) 
        self.status_port = Status_port()

        # -- NUC info
        rospy.Subscriber("/nuc_info", Nuc_info, self.callback_nucInfo)
        self.nuc_info = Nuc_info()

        # -- Pose robot
        rospy.Subscriber("/robot_pose", PoseStamped, self.callback_robotPose) 
        self.robotPose = PoseStamped()

        # -- info AGV
        rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_callback) 
        self.NN_infoRespond = NN_infoRespond()

        # -----------------------------------------------------------
        # rospy.Subscriber("/cancelMission_status", Int16, self.callBack_cancelMission)
        # self.cancelMission_status = Int16()

        rospy.Subscriber("/raw_vel", TwistWithCovarianceStamped, self.callback_Rawvel) # 
        self.data_rawvel = TwistWithCovarianceStamped()
        self.VEL_MAX = 0.8

    def callback_HC(self,data):
        self.HC_info = data
    def callback_Main(self, data):
        self.main_info = data 
    def callback_OC_board(self, data):
        self.OC_status = data 
    def callback_statusPort(self,data):
        self.status_port = data
    def callback_nucInfo(self, data):
        self.nuc_info = data 
    def callback_robotPose(self, data):
        self.robotPose = data
    def infoAGV_callback(self, data):
        self.NN_infoRespond = data

    def callback_brakeControl(self, data):
        self.status_brake = data
    def callback_safetyZone(self, data):
        self.safety_zone = data 
    def callback_Rawvel(self, data):
        self.data_rawvel = data

    def callback_POWER_request(self, data):
        self.request_power = data

    # async def handle_client(self, websocket, path):
    #     rospy.loginfo("🔗 Client đã kết nối!")

    #     try:

    #         # hello_msg = json.dumps({"message": "hello_world"})
    #         # await websocket.send(hello_msg)
    #         # rospy.loginfo(f"📤 Đã gửi: {hello_msg}")
    #         while not rospy.is_shutdown():
    #             # Gửi dữ liệu định kỳ
    #             hello_msg = json.dumps({"message": "hello_world"})
    #             await websocket.send(hello_msg)
    #             rospy.loginfo(f"📤 Đã gửi: {hello_msg}")


    #         async for message in websocket:
    #             rospy.loginfo(f"📩 Nhận tin nhắn: {message}")

    #             try:
    #                 # Giải mã JSON từ client gửi về
    #                 data = json.loads(message)
    #             except json.JSONDecodeError:
    #                 rospy.logerr("❌ Dữ liệu JSON không hợp lệ!")
    #                 continue  # Bỏ qua dữ liệu lỗi và chờ tin nhắn tiếp theo

    #             cmd_control = int(data.get("cmd_control", 0))
    #             cmd_velocity = float(data.get("cmd_velocity", 0.0))

    #             self.control_vel(cmd_control, cmd_velocity)


        # except websockets.exceptions.ConnectionClosed as e:
        #     rospy.logwarn("⚠️ Mất kết nối với client")
        # finally:
        #     self.clients.remove(websocket)
        #     rospy.loginfo("❌ Client đã ngắt kết nối")


    def convert_errorAll(self, val):
        switcher={
            0:'AGV Hoạt Động Bình Thường',
            2:'Đã Khởi Tạo Lại Quy Trình Hiện Tại',
            3:'Nút Xóa Lỗi Đã Được Nhấn',
            311:'Mất kết nối với Mạch STI-RTC',         
            361:'Mất Kết Nối Với Mạch STI-CPD', # 
            362:'Mất Kết Nối Với Mạch STI-CPD2', #
            351:'Mất Kết Nối Với Mạch STI-HC', # 
            352:'Không Giao Tiếp CAN Với Mạch STI-HC', # 

            341:'Mất Kết Nối Với Mạch STI-OC', #
            342:'Mất Cổng USB của USB của Mạch STI-OC', # 
            343:'Không Giao Tiếp CAN Với Mạch STI-OC', # 
            344:'Không Giao Tiếp Với Mạch STI-OC1', # 
            345:'Không Giao Tiếp Với Mạch STI-OC2', # 
            346:'Không Giao Tiếp Với Mạch STI-OC3', # 

            323:'Mạng CAN Không Gửi Được', # 
            321:'Mất Kết Nối Với Mạch STI-Main', # 
            322:'Mất Cổng USB của USB của Mạch STI-Main', # 
            250:'Mất Kết Nối Với Driver', # 
            251:'Mất Kết Nối Với Driver1', # 
            252:'Lỗi Động Cơ Số 1', # 
            261:'Mất Kết Nối Với Driver2', # 
            262:'Lỗi Động Cơ Số 2', # 
            231:'Mất Kết Nối Với Cảm Biến Góc', # 
            232:'Mất Cổng USB của Cảm Biến IMU', # 
            241:'Lỗi Định Vị: Matching', # 
            242:'Lỗi Định Vị: Localization', # 
            243:'Lỗi Định Vị: Matching',
            221:'Mất Kết Nối Với Cảm Biến NAV350', # 
            181:'LoadCell-Ket Noi', # 
            182:'LoadCell-Dau noi', # 
            183:'LoadCell-USB', # 
            184:'Quá Tải 700kg', # 
            222:'Mất Tọa Độ Định vị', # 
            223:'Parking: Không Phát Hiện mã Tag', #
            224:'Parking: Mất Dữ Liệu Odom Lidar', #
            225:'FANUC xảy ra lỗi', #
            141:'Lỗi Không Chạm Được Cảm Biến Bàn Nâng', # 
            121:'Trạng Thái Dừng Khẩn - EMG', # 
            122:'AGV Bị Chạm Blsock', #
            272:'Không Phát Hiện Được Đủ Gương', #
            281:'Mất TF Parking', #
            282:'Mất Gói Điều Hướng Di Chuyển', #

            440:'Parking: Không phát hiện được Tag', #
            441:'AGV Đã Di Chuyển Hết Điểm', #
            442:'AGV Đang Dừng Để Nhường Đường Cho AGV Khác', # 
            460:'FANUC: Đang Chưa Được Khởi Động', # 
            461:'FANUC: Đang Tạm Dừng Chương Trình', # 
            411:'Vướng Vật Cản - Di Chuyển Giữa Các Điểm', #
            412:'Vướng Vật Cản - Di Chuyển Vào Vị Trí Tag', # 
            414:'Parking: ID Tag Không Khớp', # 
            413:'Không Giao Tiếp Được Với Máy TAIFUN Qua Toyo', # 
            431:'AGV Không Giao Tiếp Với Phần Mềm Traffic', #
            451:'Điện Áp Của AGV Đang Rất Thấp', # 
            452:'AMR Không Sạc Được Pin', # 
            453:'Chưa Thực Hiện Thao Tác Định Vị', # 
            471:'Không Có Kệ Tại Vị Trí', # 
            454:'Đang Khởi Tạo Lại Vị Trí của AGV',
        }
        return switcher.get(val, 'UNK')

    async def handle_client(self, websocket, path):
        rospy.loginfo("🔗 Client đã kết nối!")

        async def send_loop():
            while not rospy.is_shutdown():
                self.converted_errors = [self.convert_errorAll(val) for val in self.NN_infoRespond.listError]
                # print(self.NN_infoRespond.listError)
                data = {
                    "x": 12345.0,                   # float
                    "y": 12345.0,                   # float
                    "angle": 90.0,           # float
                    "safety_front": self.HC_info.zone_sick_ahead,   # bool/int
                    "safety_behind": self.HC_info.zone_sick_behind, # bool/int
                    "bardershock": self.HC_info.vacham,
                    "voltage": self.main_info.voltages,      # float
                    "charge_current": self.main_info.charge_current,
                    "stsButton_reset": int(self.main_info.stsButton_reset),
                    "stsButton_power": int(self.main_info.stsButton_power),
                    "EMC_status": int(self.main_info.EMC_status),
                    "CAN_status": int(self.main_info.CAN_status),
                    "nuc_mac": self.nuc_info.nuc_mac,
                    "nuc_name": self.nuc_info.nuc_name,
                    "nuc_ipWifi":self.nuc_info.nuc_ipWifi,
                    "cpu_usage": self.nuc_info.cpu_usage,
                    "cpu_temp": self.nuc_info.cpu_temp,
                    "ram_usage": self.nuc_info.ram_usage,
                    "ram_total": self.nuc_info.ram_total,
                    "ram_percent": self.nuc_info.ram_percent,
                    "ping_server": self.nuc_info.ping_server,
                    "wifi_quality":self.nuc_info.wifi_quality,
                    "error": self.converted_errors

                }
                # print(self.converted_errors)

                msg = json.dumps(data, ensure_ascii=False)

                await websocket.send(msg)
                rospy.loginfo(f"📤 Đã gửi: {msg}")
                await asyncio.sleep(1)  # gửi mỗi giây

        async def recv_loop():
            async for message in websocket:
                # rospy.loginfo(f"📩 Nhận tin nhắn: {message}")
                try:
                    # Giải mã JSON từ client gửi về
                    data = json.loads(message)
                except json.JSONDecodeError:
                    rospy.logerr("❌ Dữ liệu JSON không hợp lệ!")
                    continue  # Bỏ qua dữ liệu lỗi và chờ tin nhắn tiếp theo

                # cmd_control = int(data.get("cmd_control", 0))
                # cmd_velocity = float(data.get("cmd_velocity", 0.0))
                # sound = int(data.get("cmd_sound",0))
                # lift = int(data.get("cmd_lifting", 0))

                # self.control_vel(cmd_control, cmd_velocity)
                # if sound == 1 and self.request_power.sound_on == 1:
                #     self.request_power.sound_on = False
                # elif sound == 1 and self.request_power.sound_on == 0:
                #     self.request_power.sound_on = True
                # self.pub_POWER_request.publish(self.request_power)

                # if lift == 1:
                #     self.pub_lift.data = 1
                # elif lift == 2:
                #     self.pub_lift.data = 2
                # elif lift == 0:
                #     self.pub_lift.data = 0
                # self.publish_lifting.publish(self.pub_lift)

                cmd_control = int(data.get("cmd_control", 0))
                cmd_velocity = float(data.get("cmd_velocity", 0.0))
                sound = int(data.get("cmd_sound",0))
                lift = int(data.get("cmd_lifting", 0))
                print(cmd_control)
                self.app_button.vs_speed = 80
                if cmd_control == 1:
                    self.app_button.bt_forwards = True
                    self.app_button.bt_backwards = False
                    self.app_button.bt_rotation_left = False
                    self.app_button.bt_rotation_right = False
                    self.app_button.bt_stop = False
                elif cmd_control == 2:
                    self.app_button.bt_forwards = False
                    self.app_button.bt_backwards = True
                    self.app_button.bt_rotation_left = False
                    self.app_button.bt_rotation_right = False
                    self.app_button.bt_stop = False
                elif cmd_control == 3:
                    self.app_button.bt_forwards = False
                    self.app_button.bt_backwards = False
                    self.app_button.bt_rotation_left = True
                    self.app_button.bt_rotation_right = False
                    self.app_button.bt_stop = False
                elif cmd_control == 4:
                    self.app_button.bt_forwards = False
                    self.app_button.bt_backwards = False
                    self.app_button.bt_rotation_left = False
                    self.app_button.bt_rotation_right = True
                    self.app_button.bt_stop = False
                elif cmd_control == 0:
                    self.app_button.bt_forwards = False
                    self.app_button.bt_backwards = False
                    self.app_button.bt_rotation_left = False
                    self.app_button.bt_rotation_right = False
                    self.app_button.bt_stop = True


                if sound == 1:
                    self.app_button.bt_spk_off = True
                    self.app_button.bt_spk_on = False
                if sound == 0:
                    self.app_button.bt_spk_off = False
                    self.app_button.bt_spk_on = True

                # self.app_button.vs_speed = 50
                if lift == 1:
                    self.app_button.bt_lift = 2
                elif lift == 2:
                    self.app_button.bt_lift = 1
                elif lift == 0:
                    self.app_button.bt_lift = 0

                self.pub_appbutton.publish(self.app_button)



        try:
            send_task = asyncio.create_task(send_loop())
            recv_task = asyncio.create_task(recv_loop())

            await asyncio.gather(send_task, recv_task)
            
        except asyncio.CancelledError:
            rospy.logwarn("🛑 Bị hủy bởi asyncio loop")
        except websockets.exceptions.ConnectionClosed:
            rospy.logwarn("⚠️ Client đã ngắt kết nối")




    def control_vel(self,dir,velocity):
        twist = Twist()
        if dir == 0:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif dir == 1:  #tien
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        elif dir == 3:  #xoay trai
            twist.linear.x = 0
            twist.angular.z = 0.3
        elif dir == 2:  #lui
            twist.linear.x = -0.2
            twist.angular.z = 0.0
        elif dir == 4:  #xoay phai
            twist.linear.x = 0 
            twist.angular.z = -0.3
        self.pub_vel.publish(twist)

    async def run(self):
        rospy.loginfo("🚀 WebSocket server đang chạy trên ws://0.0.0.0:8000")

        stop_event = asyncio.Event()

        def shutdown():
            rospy.loginfo("Đang đóng WebSocket server...")
            stop_event.set()

        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT, shutdown)
        loop.add_signal_handler(signal.SIGTERM, shutdown)

        async with websockets.serve(self.handle_client, "0.0.0.0", 8000):
            await stop_event.wait()

if __name__ == "__main__":
    # rospy.init_node("websocket_server")
    server = WebSocketServer()
    asyncio.run(server.run())

