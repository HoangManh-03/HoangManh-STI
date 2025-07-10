#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Phùng Quý Dương
Company: STI Viet Nam
date: 20/3/2025

"""
from app_ros.app_interface_new import * 

class Program(Node):
	def __init__(self):
		super().__init__('app_ros')
		# self.threadID = threadID
		# self.shutdown_flag = threading.Event()
		self.thread = None
		# --
		# self.name_card = rospy.get_param("name_card", "wlo2")
		self.name_card = "wlo2"

		# self.address_traffic = rospy.get_param("address_traffic", "172.21.15.224")
		# self.address_traffic = "192.168.2.1"
		self.pre_timePing = time.time()
		# --
		# rospy.init_node('app_ros', anonymous = False)
		# self.rate = rospy.Rate(40)

		self.app = QApplication(sys.argv)
		self.welcomeScreen = WelcomeScreen()
		screen = self.app.primaryScreen()

		size = screen.size()
		print('Size: %d x %d' % (size.width(), size.height()))

		self.widget = QtWidgets.QStackedWidget()
		self.widget.addWidget(self.welcomeScreen)
		self.widget.setFixedHeight(450)
		self.widget.setFixedWidth(800)
		# --
		self.valueLable = valueLable()
		self.statusColor = statusColor()
		# -- 
		self.is_exist = 1
		# self.widget.setWindowFlag(Qt.FramelessWindowHint)
		# -----------------------------------------------------------

		############################## SUBSCRIBE TOPIC #########################
		# -- Driver1
		self.sub_driver1_respond = self.create_subscription(
		    DriverRespond,
		    'driver1_respond',
		    self.callback_driver1,
		    10)
		self.driver1_respond = DriverRespond()
		self.sub_driver1_respond


		# -- Driver2
		self.sub_driver2_respond = self.create_subscription(
		    DriverRespond,
		    'driver2_respond',
		    self.callback_driver2,
		    10)

		self.driver2_respond = DriverRespond()
		self.sub_driver2_respond

		# -- HC
		self.sub_hc_info = self.create_subscription(
		    HcInfo,
		    'hc_info',
		    self.callback_HC,
		    10)

		self.HC_info = HcInfo()
		self.sub_hc_info

		# -- Main
		self.sub_main_info = self.create_subscription(
		    PowerInfo,
		    'power_info',
		    self.callback_Main,
		    10)

		self.main_info = PowerInfo()
		self.sub_main_info

		# -- OC board
		self.sub_oc_info = self.create_subscription(
		    LiftStatus,
		    'lift_status',
		    self.callback_OC_board,
		    10)

		self.OC_status = LiftStatus()
		self.sub_oc_info

		# -- Status Port
		self.sub_status_port = self.create_subscription(
		    StatusPort,
		    'status_port',
		    self.callback_statusPort,
		    10)

		self.status_port = StatusPort()
		self.sub_status_port

		# -- NUC info
		self.sub_nuc_info = self.create_subscription(
		    NucInfo,
		    'nuc_info',
		    self.callback_nucInfo,
		    10)

		self.nuc_info = NucInfo()
		self.sub_nuc_info

		# ------------------------------
		# -- data safety NAV
		# rospy.Subscriber("/safety_NAV", Int8, self.callback_safetyNAV) 
		# self.safety_NAV = Int8()

		# -- Pose robot
		# rospy.Subscriber("/robot_pose", Pose, self.callback_robotPose) 
		# self.robotPose = Pose()

		# -- Traffic cmd
		self.sub_traffic_cmd = self.create_subscription(
		    NNcmdRequest,
		    'NN_cmdRequest',
		    self.NN_cmdRequest_callback,
		    10)

		self.NN_cmdRequest = NNcmdRequest()
		self.sub_traffic_cmd

		# -- Traffic info
		self.sub_traffic_info = self.create_subscription(
		    NNinfoRequest,
		    'NN_infoRequest',
		    self.callback_NN_infoRequest,
		    10)

		self.NN_infoRequest = NNinfoRequest()
		self.sub_traffic_info

		# -- info AGV
		self.sub_agv_info = self.create_subscription(
		    NNinfoRespond,
		    'NN_infoRespond',
		    self.infoAGV_callback,
		    10)

		self.NN_infoRespond = NNinfoRespond()
		self.sub_agv_info

		# -- Launch
		self.sub_launch_info = self.create_subscription(
		    StatusLaunch,
		    'status_launch',
		    self.callback_statusLaunch,
		    10)

		self.status_launch = StatusLaunch()
		self.sub_launch_info

		# -- cancel mission 
		self.sub_cancel_mission = self.create_subscription(
		    Int16,
		    'cancelMission_status',
		    self.callBack_cancelMission,
		    10)

		self.cancelMission_status = Int16()
		self.sub_cancel_mission

		#################################### PUBLISH TOPIC ################################3
		# -- cancel mission
		self.pub_cancelMission = self.create_publisher(Int16, 'cancelMission_control', 4)
		self.cancelMission_control = Int16()

		# -- app button
		self.pub_button = self.create_publisher(AppButtonAgvmag, 'app_button', 4)
		self.app_button = AppButtonAgvmag()

		timer_period = 0.05  # 20 Hz
		self.timer = self.create_timer(timer_period, self.timer_callback)

		# --------------------
		self.name_agv = ""
		self.ip_agv = ""
		# --
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_byhand = 1
		self.modeRun_auto = 2

		self.signal_front = []
		self.signal_behind = []

		self.signal_front_final = []
		self.signal_behind_final = []


	def callback_driver1(self, data):
		self.driver1_respond = data

	def callback_driver2(self, data):
		self.driver2_respond = data

	def callback_HC(self, data):
		self.HC_info = data

	def callback_Main(self, data):
		self.main_info = data

	def callback_OC_board(self, data):
		self.lift_status = data

	def callback_nucInfo(self, data):
		self.nuc_info = data

	def callback_statusPort(self, data):
		self.status_port = data

	def callBack_cancelMission(self, data):
		self.cancelMission_status = data

	def NN_cmdRequest_callback(self, data):
		self.NN_cmdRequest = data	

	def callback_NN_infoRequest(self, data):
		self.NN_infoRequest = data

	def infoAGV_callback(self, data):
		self.NN_infoRespond = data	

	def callback_statusLaunch(self, data):
		self.status_launch = data

	def callBack_cancelMission(self, data):
		self.cancelMission_status = data

	# def ping_traffic(self, address):
	# 	try:
	# 		ping = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
	# 		# print(ping)
	# 		vitri = str(ping).find("time")
	# 		time_ping = str(ping)[(vitri+5):(vitri+9)]
	# 		# print (time_ping)
	# 		return str(float(time_ping))
	# 	except Exception:
	# 		return '-1'

	# def get_ipAuto(self, name_card): # name_card : str()
	# 	try:
	# 		address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
	# 		print ("address: ", address)
	# 		return address
	# 	except Exception:
	# 		return "-1"

	def run_screen(self):
		self.widget.show()
		try:
			# print ("run 1")
			sys.exit(self.app.exec_())
			# print ("run 2")
		except:
			pass
			# print("Exiting 1")
		self.is_exist = 0

	def kill_app(self):
		self.welcomeScreen.out()
		self.is_exist = 0

	# --
	# def get_MAC(self, name_card): # name_card : str()
	# 	try:
	# 		MAC = ''
	# 		output = os.popen("ip addr show {}".format(name_card) ).read()
	# 		pos1 = str(output).find('link/ether ') # tuyet doi ko sua linh tinh.
	# 		pos2 = str(output).find(' brd')   # tuyet doi ko sua linh tinh.

	# 		if (pos1 >= 0 and pos2 > 0):
	# 			MAC = str(output)[pos1+11:pos2]

	# 		print ("MAC: ", MAC)
	# 		return MAC
	# 	except Exception:
	# 		return "-1"
	# --
	# def get_qualityWifi(self, name_card): # int
	# 	try:
	# 		quality_data = '0'
	# 		output = os.popen("iwconfig {}".format(name_card)).read()
	# 		pos_quality = str(output).find('Link Quality=')
	# 		# -
	# 		if pos_quality >= 0:
	# 			quality_data = str(output)[pos_quality+13:pos_quality+15]
	# 		# print ("quality_data: ", int(quality_data) )
	# 		# -
	# 		return int(quality_data)
	# 	except Exception:
	# 		return 0
	# ---
	# def get_hostname(self):
	# 	try:
	# 		output = os.popen("hostname").read()
	# 		# print ("output: ", output)
	# 		leng = len(output)
	# 		hostname = str(output)[0:leng-1]
	# 		print ("hostname: ", hostname)
	# 		return hostname
	# 	except Exception:
	# 		return "-1"

	# def python_euler_from_quaternion(self, x, y, z, w):
	# 	"""
	# 	Convert a quaternion into euler angles (roll, pitch, yaw)
	# 	roll is rotation around x in radians (counterclockwise)
	# 	pitch is rotation around y in radians (counterclockwise)
	# 	yaw is rotation around z in radians (counterclockwise)
	# 	"""
	# 	t0 = +2.0 * (w * x + y * z)
	# 	t1 = +1.0 - 2.0 * (x * x + y * y)
	# 	roll_x = atan2(t0, t1)
	
	# 	t2 = +2.0 * (w * y - z * x)
	# 	t2 = +1.0 if t2 > +1.0 else t2
	# 	t2 = -1.0 if t2 < -1.0 else t2
	# 	pitch_y = asin(t2)
	
	# 	t3 = +2.0 * (w * z + x * y)
	# 	t4 = +1.0 - 2.0 * (y * y + z * z)
	# 	yaw_z = atan2(t3, t4)
	
	# 	return roll_x, pitch_y, yaw_z # in radians
			
	# def euler_to_quaternion(self, euler):
	# 	quat = Quaternion()
	# 	odom_quat = quaternion_from_euler(0, 0, euler)
	# 	quat.x = odom_quat[0]
	# 	quat.y = odom_quat[1]
	# 	quat.z = odom_quat[2]
	# 	quat.w = odom_quat[3]
	# 	return quat

	# def quaternion_to_euler(self, qua):
	# 	# quat = (qua.x, qua.y, qua.z, qua.w )
	# 	a, b, euler = self.python_euler_from_quaternion(qua.x, qua.y, qua.z, qua.w)
	# 	return euler

	# def limitAngle(self, angle_in): # - rad
	# 	qua_in = self.euler_to_quaternion(angle_in)
	# 	angle_out = self.quaternion_to_euler(qua_in)
	# 	return angle_out

	def convert_errorAll(self, val):
		switcher={
			0:'AGV Hoạt Động Bình Thường',
			311:'Mất kết nối với Mạch STI-RTC',			
			351:'Mất Kết Nối Với Mạch STI-HC', # 
			352:'Không Giao Tiếp CAN Với Mạch STI-HC', # 

			341:'Mất Kết Nối Với Mạch STI-OC', #
			342:'Không Giao Tiếp CAN Với Mạch STI-OC', # 

			347:'AGV Đang Nằm Ngoài Line Từ',

			322:'Không Giao Tiếp CAN Với Mạch STI-MAIN', # 
			321:'Mất Kết Nối Với Mạch STI-Main', # 

			# 250:'Mất Kết Nối Với Driver', # 
			251:'Mất Kết Nối Với Driver1', # 
			252:'Lỗi Động Cơ Số 1', # 
			261:'Mất Kết Nối Với Driver2', # 
			262:'Lỗi Động Cơ Số 2', # 
			# 231:'Mất Kết Nối Với Cảm Biến Góc', # 
			# 232:'Mất Cổng USB của Cảm Biến IMU', # 
			# 241:'Lỗi Định Vị: Matching', # 
			# 242:'Lỗi Định Vị: Localization', # 
			# 221:'Mất Kết Nối Với Cảm Biến NAV350', # 
			# 181:'LoadCell-Ket Noi', # 
			# 182:'LoadCell-Dau noi', # 
			# 183:'LoadCell-USB', # 
			# 184:'Quá Tải 700kg', # 
			# 222:'Mất Tọa Độ Định vị', # 
			# 223:'Parking: Không Phát Hiện mã Tag', #
			# 224:'Parking: Mất Dữ Liệu Odom Lidar', #
			# 225:'FANUC xảy ra lỗi', #
			141:'Bàn Nâng Không Di Chuyển Hết Hành Trình', # 

			121:'Trạng Thái Dừng Khẩn - EMG', # 
			122:'AGV Bị Chạm Blsock', #
			282:'Mất Gói Điều Hướng Di Chuyển', #
			283:'Mất Kết Nối Với Dàn Dò Trước',
			284:'Mất Kết Nối Với Dàn Dò Sau',
			285:'Mất Kết Nối Với Bộ Đọc RFID',


			# 440:'Parking: Không phát hiện được Tag', #
			# 441:'AGV Đã Di Chuyển Hết Điểm', #
			442:'AGV Đang Dừng Chờ', # 
			443:'AGV Đang Dừng Chờ Do Có Tín Hiệu Báo cháy',
			# 460:'FANUC: Đang Chưa Được Khởi Động', # 
			# 461:'FANUC: Đang Tạm Dừng Chương Trình', # 
			411:'Vướng Vật Cản - Di Chuyển Giữa Các Điểm', #
			# 412:'Vướng Vật Cản - Di Chuyển Vào Vị Trí Tag', # 
			# 414:'Parking: ID Tag Không Khớp', # 
			# 413:'Không Giao Tiếp Được Với Máy TAIFUN Qua Toyo', # 
			431:'AGV Không Giao Tiếp Với Phần Mềm Traffic', #
			451:'Điện Áp Của AGV Đang Rất Thấp', # 
			#452:'AMR Không Sạc Được Pin', # 
			#453:'Chưa Thực Hiện Thao Tác Định Vị', # 
			471:'Không Phát Hiện Kệ Hàng trên AGV' # 
		}
		return switcher.get(val, 'UNK')

	def show_job(self, val):
		job_now = ''
		switcher={
			0:'...', # 
			1:'Kiem Tra Tu Dong', # kiem tra trang thai ban nang sau khi Sang che do tu dong
			2:'Nhiem Vu Truoc', # thuc hien nhiem vu truoc
			3:'Kiem Tra Ke', # kiem tra ke 
			4:'Di Ra', # 
			5:'Di Chuyen', #
			6:'Di Vao Ke', # 
			7:'Nhiem Vu Sau', # 
			8:'Doi Lenh Moi', # 
			10:'Đợi Xác Nhận Từ Người Thao Tác',
			20:'Bang Tay' # 
		}
		return switcher.get(val, job_now)

	def show_misson(self, val):
		job_now = ''
		switcher={
			0:'...', # 
			1:'Kiem Tra Tu Dong', # kiem tra trang thai ban nang sau khi Sang che do tu dong
			2:'Nhiem Vu Truoc', # thuc hien nhiem vu truoc
			3:'Kiem Tra Ke', # kiem tra ke 
			4:'Di Ra', # 
			5:'Di Chuyen', #
			6:'Di Vao Ke', # 
			7:'Nhiem Vu Sau', # 
			10:'Sac\nPin', # 
			20:'Bang Tay' # 
		}
		return switcher.get(val, job_now)

	def find_element(self, value_find, list_in):
		lenght = len(list_in)
		for i in range(lenght):
			if (value_find == list_in[i]):
				return 1
		return 0

	def controlColor(self):
		# --
		self.statusColor.lbc_button_clearError = self.main_info.status_btn_reset
		self.statusColor.lbc_button_power = self.main_info.status_btn_power
		self.statusColor.lbc_emg = self.main_info.emg_status

		# -- 
		self.statusColor.lbc_badersock = self.HC_info.vacham
		# -- Port
		self.statusColor.lbc_port_rtcBoard = self.status_port.rtc
		self.statusColor.lbc_port_Magline  = self.status_port.magline
		self.statusColor.lbc_port_RFID     = self.status_port.rfid
		self.statusColor.lbc_port_Driver   = self.status_port.driverall

		# --
		# self.statusColor.lbc_toyoRead_1 = self.toyo_status.bit1
		# self.statusColor.lbc_toyoRead_2 = self.toyo_status.bit2
		# self.statusColor.lbc_toyoRead_3 = self.toyo_status.bit3

		# --
		self.statusColor.lbc_safety_ahead  = self.HC_info.zone_sick_ahead
		self.statusColor.lbc_safety_behind = self.HC_info.zone_sick_behind
		# self.statusColor.lbc_safety_left   = self.safety_zone.zone_left
		# self.statusColor.lbc_safety_right  = self.safety_zone.zone_right
		# self.statusColor.lbc_safety_circle = self.safety_zone.zone_circle_beside
		# --
		self.statusColor.lbc_limit_up = self.OC_status.sensor_up
		self.statusColor.lbc_limit_down = self.OC_status.sensor_down
		self.statusColor.lbc_detect_lifter = self.OC_status.sensor_lift
		
		# if self.signal_front_final != self.signal_front:
		# for i in range(16):
		# 	if self.find_element(i, self.signal_magline_front.signal_magline) == 1:
		# 		self.signal_front.append(1)
		# 	else:
		# 		self.signal_front.append(0)
		
		# print(self.signal_front)
		# self.signal_front_final = self.signal_front
		# self.signal_front = []

		# self.statusColor.lbc_magline_t1 = self.signal_front_final[0]
		# self.statusColor.lbc_magline_t2 = self.signal_front_final[1]
		# self.statusColor.lbc_magline_t3 = self.signal_front_final[2]
		# self.statusColor.lbc_magline_t4 = self.signal_front_final[3]
		# self.statusColor.lbc_magline_t5 = self.signal_front_final[4]
		# self.statusColor.lbc_magline_t6 = self.signal_front_final[5]
		# self.statusColor.lbc_magline_t7 = self.signal_front_final[6]
		# self.statusColor.lbc_magline_t8 = self.signal_front_final[7]
		# self.statusColor.lbc_magline_t9 = self.signal_front_final[8]
		# self.statusColor.lbc_magline_t10 = self.signal_front_final[9]
		# self.statusColor.lbc_magline_t11 = self.signal_front_final[10]
		# self.statusColor.lbc_magline_t12 = self.signal_front_final[11]
		# self.statusColor.lbc_magline_t13 = self.signal_front_final[12]
		# self.statusColor.lbc_magline_t14 = self.signal_front_final[13]
		# self.statusColor.lbc_magline_t15 = self.signal_front_final[14]
		# self.statusColor.lbc_magline_t16 = self.signal_front_final[15]

		# if self.signal_behind_final != self.signal_behind:
		# for i in range(16):
		# 	if self.find_element(i, self.signal_magline_behind.signal_magline) == 1:
		# 		self.signal_behind.append(1)
		# 	else:
		# 		self.signal_behind.append(0)
		
		# print(self.signal_behind)
		# self.signal_behind_final = self.signal_behind
		# self.signal_behind = []

		# self.statusColor.lbc_magline_s1 = self.signal_behind_final[0]
		# self.statusColor.lbc_magline_s2 = self.signal_behind_final[1]
		# self.statusColor.lbc_magline_s3 = self.signal_behind_final[2]
		# self.statusColor.lbc_magline_s4 = self.signal_behind_final[3]
		# self.statusColor.lbc_magline_s5 = self.signal_behind_final[4]
		# self.statusColor.lbc_magline_s6 = self.signal_behind_final[5]
		# self.statusColor.lbc_magline_s7 = self.signal_behind_final[6]
		# self.statusColor.lbc_magline_s8 = self.signal_behind_final[7]
		# self.statusColor.lbc_magline_s9 = self.signal_behind_final[8]
		# self.statusColor.lbc_magline_s10 = self.signal_behind_final[9]
		# self.statusColor.lbc_magline_s11 = self.signal_behind_final[10]
		# self.statusColor.lbc_magline_s12 = self.signal_behind_final[11]
		# self.statusColor.lbc_magline_s13 = self.signal_behind_final[12]
		# self.statusColor.lbc_magline_s14 = self.signal_behind_final[13]
		# self.statusColor.lbc_magline_s15 = self.signal_behind_final[14]
		# self.statusColor.lbc_magline_s16 = self.signal_behind_final[15]

	def controlAll(self):
		# -- Mode show
		if (self.NN_infoRespond.mode == 0):   # - launch
			self.valueLable.modeRuning = self.modeRun_launch

		elif (self.NN_infoRespond.mode == 1): # -- md_by_hand
			self.valueLable.modeRuning = self.modeRun_byhand

		elif (self.NN_infoRespond.mode == 2): # -- md_auto
			self.valueLable.modeRuning = self.modeRun_auto

		# -- Battery
		if (self.main_info.charge_current > 0.1):
			self.statusColor.lbc_battery = 4
		else:
			if (self.main_info.voltages < 23.5):
				self.statusColor.lbc_battery = 3
			elif (self.main_info.voltages >= 23.5 and self.main_info.voltages < 24.5):
				self.statusColor.lbc_battery = 2
			else:
				self.statusColor.lbc_battery = 1
		
		bat = 55
		bat = round(self.main_info.voltages, 1)
		if (bat > 55):
			bat = 55
		self.valueLable.lbv_battery = str(bat) + " V"

		# -- status AGV
		self.statusColor.cb_status = self.NN_infoRespond.status
		# --
		lg_err = len(self.NN_infoRespond.list_error)
		self.valueLable.listError = []
		if (lg_err == 0):
			self.valueLable.listError.append( self.convert_errorAll(0) )
		else:
			for i in range(lg_err):
				self.valueLable.listError.append( self.convert_errorAll(self.NN_infoRespond.list_error[i]) )
		
		self.valueLable.lbv_name_agv = self.NN_infoRequest.name_agv
		
		# -- Ping
		# deltaTime_ping = (time.time() - self.pre_timePing)%60
		# if (deltaTime_ping > 2.0):
		# 	self.pre_timePing = time.time()
		# 	self.valueLable.lbv_pingServer = self.ping_traffic(self.address_traffic)
		# 	# -
		# 	self.valueLable.lbv_qualityWifi = self.get_qualityWifi(self.name_card)

		# -- 
		self.valueLable.lbv_rfid_lastcode = str(self.NN_infoRespond.rfid_last_code)
		self.valueLable.lbv_rfid_code = str(self.NN_infoRespond.rfid_code)
		self.valueLable.lbv_direction = str(self.NN_infoRespond.direction)
		
		# --
		self.valueLable.lbv_route_target = str(self.NN_cmdRequest.target_id) + "\n" + str(self.NN_cmdRequest.target_dir) + "\n" + str(self.NN_cmdRequest.moving_dir) + "\n" + str(self.NN_cmdRequest.tag) + "\n" + str(self.NN_cmdRequest.offset)
		# -
		if len(self.NN_cmdRequest.list_id) >= 5:
			# self.valueLable.lbv_route_point0 = str(self.server_cmdRequest.list_id[0]) + "\n" + str(self.server_cmdRequest.list_x[0]) + "\n" + str(self.server_cmdRequest.list_y[0]) + "\n" + str(self.server_cmdRequest.list_speed[0]) + "\n" + str(self.server_cmdRequest.list_directionTravel[0]) + "\n" + str(self.server_cmdRequest.list_angleLine[0])
			# self.valueLable.lbv_route_point1 = str(self.server_cmdRequest.list_id[1]) + "\n" + str(self.server_cmdRequest.list_x[1]) + "\n" + str(self.server_cmdRequest.list_y[1]) + "\n" + str(self.server_cmdRequest.list_speed[1]) + "\n" + str(self.server_cmdRequest.list_directionTravel[1]) + "\n" + str(self.server_cmdRequest.list_angleLine[1])
			# self.valueLable.lbv_route_point2 = str(self.server_cmdRequest.list_id[2]) + "\n" + str(self.server_cmdRequest.list_x[2]) + "\n" + str(self.server_cmdRequest.list_y[2]) + "\n" + str(self.server_cmdRequest.list_speed[2]) + "\n" + str(self.server_cmdRequest.list_directionTravel[2]) + "\n" + str(self.server_cmdRequest.list_angleLine[2])
			# self.valueLable.lbv_route_point3 = str(self.server_cmdRequest.list_id[3]) + "\n" + str(self.server_cmdRequest.list_x[3]) + "\n" + str(self.server_cmdRequest.list_y[3]) + "\n" + str(self.server_cmdRequest.list_speed[3]) + "\n" + str(self.server_cmdRequest.list_directionTravel[3]) + "\n" + str(self.server_cmdRequest.list_angleLine[3])
			# self.valueLable.lbv_route_point4 = str(self.server_cmdRequest.list_id[4]) + "\n" + str(self.server_cmdRequest.list_x[4]) + "\n" + str(self.server_cmdRequest.list_y[4]) + "\n" + str(self.server_cmdRequest.list_speed[4]) + "\n" + str(self.server_cmdRequest.list_directionTravel[4]) + "\n" + str(self.server_cmdRequest.list_angleLine[4])

			self.valueLable.lbv_route_point0 = str(self.NN_cmdRequest.list_id[0]) + "\n" + str(self.NN_cmdRequest.list_code[0]) + "\n" + str(self.NN_cmdRequest.list_dir[0]) + "\n" + str(self.NN_cmdRequest.list_speed[0])
			self.valueLable.lbv_route_point1 = str(self.NN_cmdRequest.list_id[1]) + "\n" + str(self.NN_cmdRequest.list_code[1]) + "\n" + str(self.NN_cmdRequest.list_dir[1]) + "\n" + str(self.NN_cmdRequest.list_speed[1])
			self.valueLable.lbv_route_point2 = str(self.NN_cmdRequest.list_id[2]) + "\n" + str(self.NN_cmdRequest.list_code[2]) + "\n" + str(self.NN_cmdRequest.list_dir[2]) + "\n" + str(self.NN_cmdRequest.list_speed[2])
			self.valueLable.lbv_route_point3 = str(self.NN_cmdRequest.list_id[3]) + "\n" + str(self.NN_cmdRequest.list_code[3]) + "\n" + str(self.NN_cmdRequest.list_dir[3]) + "\n" + str(self.NN_cmdRequest.list_speed[3])
			self.valueLable.lbv_route_point4 = str(self.NN_cmdRequest.list_id[4]) + "\n" + str(self.NN_cmdRequest.list_code[4]) + "\n" + str(self.NN_cmdRequest.list_dir[4]) + "\n" + str(self.NN_cmdRequest.list_speed[4])
		
		# self.valueLable.lbv_route_job1 = str(self.NN_cmdRequest.before_mission)
		# self.valueLable.lbv_route_job2 = str(self.NN_cmdRequest.after_mission)

		if self.NN_cmdRequest.before_mission == 1 or self.NN_cmdRequest.before_mission == 65:
			self.valueLable.lbv_route_job1 = 'Nâng'
		elif self.NN_cmdRequest.before_mission == 2 or self.NN_cmdRequest.before_mission == 66:
			self.valueLable.lbv_route_job1 = 'Hạ'
		else:
			self.valueLable.lbv_route_job1 = 'UNK'
		
		if self.NN_cmdRequest.after_mission == 1 or self.NN_cmdRequest.after_mission == 65:
			self.valueLable.lbv_route_job2 = 'Nâng'
		elif self.NN_cmdRequest.after_mission == 3:
			self.valueLable.lbv_route_job2 = 'Nâng\nXN'
		elif self.NN_cmdRequest.after_mission == 2 or self.NN_cmdRequest.after_mission == 66:
			self.valueLable.lbv_route_job2 = 'Hạ'
		elif self.NN_cmdRequest.after_mission == 10:
			self.valueLable.lbv_route_job2 = 'Thay\npin'
		else:
			self.valueLable.lbv_route_job2 = 'UNK'


		self.valueLable.lbv_route_message = self.NN_cmdRequest.command
		self.valueLable.lbv_jobRuning = self.show_job(self.NN_infoRespond.process)

		self.valueLable.lbv_process = self.NN_infoRespond.process
		# -- 
		# self.valueLable.lbv_goalFollow_id = str(self.navigation_respond.id_goalFollow)
		# -- Launch
		self.valueLable.percentLaunch = self.status_launch.persent
		self.valueLable.lbv_launhing = self.status_launch.notification
		self.valueLable.lbv_numberLaunch = self.status_launch.position

		# - Nuc info
		self.valueLable.lbv_nuc_mac = self.nuc_info.nuc_mac
		self.valueLable.lbv_nuc_name = self.nuc_info.nuc_name
		self.valueLable.lbv_nuc_ip = self.nuc_info.nuc_ipwifi

		self.valueLable.lbv_cpu_usage = str(self.nuc_info.cpu_usage) + ' %'
		self.valueLable.lbv_cpu_temp = str(self.nuc_info.cpu_temp) + ' ' + str(chr(176)) + 'C'

		self.valueLable.lbv_ram = str(self.nuc_info.ram_usage) + '/' + str(self.nuc_info.ram_total)

		self.valueLable.lbv_ping = str(self.nuc_info.ping_server)

		self.valueLable.lbv_wifi_quality = str(self.nuc_info.wifi_quality) + ' %'

		if self.nuc_info.wifi_quality == 0 or self.nuc_info.wifi_signal == 0:
			self.valueLable.lbv_wifi_signal = 'Mất kết nối'
		else:
			if self.nuc_info.wifi_signal < -70:
				self.valueLable.lbv_wifi_signal = 'Kém'
			elif -70 <= self.nuc_info.wifi_signal < -60:
				self.valueLable.lbv_wifi_signal = 'Trung Bình'
			elif -60 <= self.nuc_info.wifi_signal:
				self.valueLable.lbv_wifi_signal = 'Tốt'

		self.valueLable.lbv_ap_mac = self.nuc_info.ap_mac
		
		self.valueLable.lbv_runtime = self.nuc_info.uptime
		
		# Driver
		self.valueLable.lbv_notification_driver1 = self.driver1_respond.message_error
		self.valueLable.lbv_notification_driver2 = self.driver2_respond.message_error

		self.valueLable.lbv_velLeft = str(self.driver1_respond.speed)
		self.valueLable.lbv_velRight = str(self.driver2_respond.speed)
	
		# --
		# self.valueLable.setPose_tagID = str(self.setpose_status.find_tag)

		# self.valueLable.lbv_tagID = str(self.parking_status.find_tagID)
		# self.valueLable.lbv_tagDistance = str(round(self.parking_status.find_tagDistance, 3))

	def readButton(self):
		# -- Toyo

		# -- 
		self.app_button.bt_cancel_mission = self.welcomeScreen.statusButton.bt_cancelMission
		self.app_button.bt_pass_auto	 = self.welcomeScreen.statusButton.bt_passAuto
		self.app_button.bt_pass_hand 	 = self.welcomeScreen.statusButton.bt_passHand
		self.app_button.bt_setting 		 = self.welcomeScreen.statusButton.bt_setting
		self.app_button.bt_clear_error 	 = self.welcomeScreen.statusButton.bt_clearError
		# --
		self.app_button.bt_movehand = self.welcomeScreen.statusButton.bt_moveHand
		# --
		self.app_button.bt_charger	= self.welcomeScreen.statusButton.bt_charger
		self.app_button.bt_speaker  = self.welcomeScreen.statusButton.bt_speaker
		self.app_button.bt_lifter	= self.welcomeScreen.statusButton.bt_lifter
		self.app_button.bt_disable_brake	= self.welcomeScreen.statusButton.bt_disableBrake
		# -- 
		self.app_button.ck_remote = self.welcomeScreen.statusButton.ck_remote
		self.app_button.ck_magline = self.welcomeScreen.statusButton.ck_magline
		self.app_button.bt_reset_framework = self.welcomeScreen.statusButton.bt_resetFrameWork
		# -
		self.app_button.vs_speed = self.welcomeScreen.statusButton.vs_speed

		self.app_button.bt_confirm = self.welcomeScreen.statusButton.bt_confirm

		# -
		self.app_button.soundtype = self.welcomeScreen.statusButton.soundtype
		self.app_button.ledtype = self.welcomeScreen.statusButton.ledtype

		# -
		self.app_button.bt_dir = self.welcomeScreen.statusButton.bt_dir
		
	def timer_callback(self):
		# -- 
		self.controlAll()
		self.controlColor()
		# --
		self.readButton()
		self.pub_button.publish(self.app_button)

		# if (self.cancelMission_status.data == 1):
		# 	self.welcomeScreen.status_button.bt_cancelMission = 0
		# 	self.cancelMission_control.data = 0

		# self.app_button = self.welcomeScreen.statusButton

		# if (self.welcomeScreen.status_button.bt_cancelMission == 1):
		# 	self.cancelMission_control.data = 1

		# self.pub_cancelMission.publish(self.cancelMission_control)

		# ----------------------
		self.welcomeScreen.valueLable = self.valueLable
		self.welcomeScreen.statusColor = self.statusColor
		# -- 
		# self.welcomeScreen.robotPoseNow = self.robotPose


def rclpy_spin(node):
	rclpy.spin(node)

def signal_handler(sig, frame):
    rclpy.shutdown()  # Tắt ROS2 khi nhận tín hiệu
    QApplication.quit()
    print("Shutting down....Turn off app")

def main(args=None):
	rclpy.init(args=args)
	print('Programmer started!')
	node = Program()
	signal.signal(signal.SIGINT, signal_handler)

	try:
		x = threading.Thread(target=rclpy_spin, args=(node,), daemon=True)
		logging.info("Main    : before running thread")
		x.start()

		node.run_screen()

	except KeyboardInterrupt:
		node.get_logger().info('Keyboard interrupt received. Exiting...')

	finally:
		# Destroy the node explicitly
		# (optional - otherwise it will be done automatically
		# when the garbage collector destroys the node object)
		node.kill_app()
		node.destroy_node()
		rclpy.shutdown()
		print('Programmer stopped!!')

if __name__ == '__main__':
	main()
