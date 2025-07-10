#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author : Phùng Quý Dương - Archie Phùng
# Date: 23-05-2024
# Fix: Le Duc Anh - 11-09-2024
"""
    > Get CPU Info
	> Get Wifi Info
	> Get time run of NUC after boot
	> Auto connect wifi if it get lost
	> Send number of losting wifi to Debug file
"""

import os
import re
import psutil
import netifaces
import subprocess
import rospy
import time
from std_msgs.msg import Int16
from sti_msgs.msg import *
import sys
import struct
import time
from decimal import *
import math

class NUC_Info:
    def __init__(self):
        
        ######################################################################################
        #Nuc and wifi
        
        print("ROS Initial!")
        rospy.init_node('nuc_info', anonymous= True) # , NoRosout = True
        self.rate = rospy.Rate(10)

        self.pub_NUC = rospy.Publisher('/nuc_info', Nuc_info, queue_size= 10)            # 1 topic tương ứng với 1 kiểu dữ liệu.
        self.NUC_info = Nuc_info()

        self.pub_wifi = rospy.Publisher('/wifi_respond', Wifi_info, queue_size = 50)
        self.wifi_info = Wifi_info()
        
        rospy.Subscriber('/Wifi_request', Wifi_request, self.wifiReset_callback)
        self.resetWifi = Wifi_request()
        
        ######################################################################################
        #Check Physical port (drive, lidar, rtc)
        
        self.portDriverAll = rospy.get_param('port_driverAll', '')
        self.portRTC = rospy.get_param('port_RTC', '')
        self.portLidar_1 = rospy.get_param('port_Lidar_1', '')
        self.portLidar_2 = rospy.get_param('port_Lidar_2', '')
        
        self.pub_check_port = rospy.Publisher('/port_status_v2', Port_status_v2, queue_size = 50)
        self.statusPort = Port_status_v2()
        
        
        
        ######################################################################################
        #Parameters
        
        
        self.name_card = rospy.get_param("name_card", "wlp4s0")
        self.name_card_lan = rospy.get_param("name_eth", "eno1")
        self.address_traffic = rospy.get_param("address_traffic", "172.21.15.224")
        self.SSID = rospy.get_param("SSID_wifi","")
        self.PASS = rospy.get_param("PASS_wifi","")
        # self.SSID = "STI_VietNam_No8"
        # self.PASS = "66668888"
        self.ip_address = rospy.get_param("ip_address", "")
        self.iplan = rospy.get_param("port_Lidar_1", "")
        
        self.portMagline_front = rospy.get_param("portMagline_front", "")
        self.portMagline_behind = rospy.get_param("portMagline_behind", "")
        self.portRFID_1 = rospy.get_param("portRFID_1", "")
        self.portRFID_2 = rospy.get_param("portRFID_2", "")
        self.portLed = rospy.get_param("portLed", "")
        self.portSpeaker = rospy.get_param("portSpeaker", "")
        self.portIMU = rospy.get_param("portIMU", "")
        self.portCamera = rospy.get_param("portCamera", "")




        self.runOnce = 0
        self.start_time = time.time()
        self.step = 1
        self.time_checkWifi = time.time()
        self.time_getApmac = time.time()
        self.time_getPing = time.time()
        self.uptime = 0
        
        # self.SSID = 'STI_VietNam_No8'
        # self.PASS = '66668888'
        
        self.process = 1
        self.MAC_APNOW = ''
        self.MAC_APSELECT = ''
        self.singal_now = 0
        self.roaming_intensity = 30
        self.saveTime_reconnectWifi = 0
        self.gateway = '192.168.1.1'
        self.dns = "8.8.8.8"
        self.check_change_wifi = 1
        self.config_iplan_oke = 1
        self.signal_now = 0
        self.rate_now = 0
            
        # self.name_card_lan = "eno1"       #LAGV
        # self.name_card_lan = "enp0s31f6"    #Test
        
        self.MAC_lan = ''
        self.data_wifi = CustomDataWifi()
        self.list_data_wifi = []
        self.time_read = 0.5
    
    def wifiReset_callback(self, data):
        self.resetWifi = data
        
    def get_hostname(self):
        try:
            output = os.popen("hostname").read()
            # print ("output: ", output)
            leng = len(output)
            hostname = str(output)[0:leng-1]
            # print ("hostname: ", hostname)
            return hostname
        except Exception:
            return "-1"

    def get_MAC(self, name_card): # name_card : str()
        try:
            MAC = ''
            output = os.popen("ip addr show {}".format(name_card) ).read()
            pos1 = str(output).find('link/ether ') # tuyet doi ko sua linh tinh.
            pos2 = str(output).find(' brd')   # tuyet doi ko sua linh tinh.

            if (pos1 >= 0 and pos2 > 0):
                MAC = str(output)[pos1+11:pos2]

            # print ("MAC: ", MAC)
            return MAC
        except Exception:
            return "-1"

    def get_ipAuto(self, name_card): # name_card : str()
        try:
            address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
            # print ("address: ", address)
            return address
        except Exception:
            return "-1"

    def get_ethernet_ip(self,interface):
        try:
            addrs = netifaces.ifaddresses(interface)
            ip_info = addrs[netifaces.AF_INET][0]
            ip_address = ip_info['addr']
            return str(ip_address)
        except (KeyError, IndexError):
            return "-1"
                  
    def get_cpu_usage(self, time_read):
        try:
            cpu_usage = psutil.cpu_percent(time_read)
            return cpu_usage
        except Exception:
            return 0
        
    def get_cpu_temp(self):
        try:
            temperature = psutil.sensors_temperatures().get('coretemp')[0].current
            return temperature
        except Exception:
            return 0

    def get_ram_usage(self):
        """
        Obtains the absolute number of RAM bytes currently in use by the system.
        :returns: System RAM usage in bytes.
        :rtype: int
        """
        try:
            ram_usage = int(psutil.virtual_memory().total - psutil.virtual_memory().available)
            return ram_usage
        except Exception:
            return 0
        
    def get_ram_total(self):
        """
        Obtains the total amount of RAM in bytes available to the system.
        :returns: Total system RAM in bytes.
        :rtype: int
        """
        try:
            ram_total = int(psutil.virtual_memory().total)
            return ram_total
        except Exception:
            return 0
        

    def get_ram_usage_pct(self):
        """
        Obtains the system's current RAM usage.
        :returns: System RAM usage as a percentage.
        :rtype: float
        """
        try:
            ram_percent = psutil.virtual_memory().percent
            return ram_percent
        except Exception:
            return 0

    def get_qualityWifi(self, name_card): # int
        try:
            pos_quality = '0'
            pos_signal = '0'
            output = os.popen("iwconfig {}".format(name_card)).read()
            # print(output)
            pos_quality = str(output).find('Link Quality=')
            pos_signal = str(output).find('Signal level=')
            # pos_bitrate = str(output).find('Bit Rate=')
            # pos_txpower = str(output).find('Tx-Power=')
            # -
            if pos_quality >= 0:
                quality_data = str(output)[pos_quality+13:pos_quality+15]
            # print ("quality_data: ", int(quality_data))
            
            # -
            if pos_signal >= 0:
                signal_data = str(output)[pos_signal+13:pos_signal+16]

            return int(quality_data), int(signal_data)
        except Exception:
            return 0, 0

    def get_AccessPointMAC(self, name_card): # int
        try:
            ap_mac = ''
            ap_mac_info = ''
            output = os.popen("iwconfig {}".format(name_card)).read()
            # print(output)
            ap_mac = str(output).find('Access Point:')
            # -
            ap_mac_info = str(output)[ap_mac+14:ap_mac+31]

            # print ("quality_data: ", int(quality_data))

            return ap_mac_info
        
        except Exception:
            return ''

    def ping_traffic(self, address):
        try:
            ping = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
            # print(ping)
            vitri = str(ping).find("time")
            time_ping = str(ping)[(vitri+5):(vitri+9)]
            # print (time_ping)
            return str(float(time_ping))
        except Exception:
            return '-1'
        
    def convert_intTotime(self, time):
        str_time = ""
        time_hour = int(time/3600)
        time = time - time_hour*3600
        time_minute = int(time/60)
        time_second = time - time_minute*60

        if time_hour == 0:
            if time_minute == 0:
                str_time = str(time_second) + "s"
            else:
                str_time = str(time_minute) + "m" + str(time_second) + "s"
        else:
            str_time = str(time_hour) + "h" + str(time_minute) + "m" + str(time_second) + "s"
            
        return str_time

    #Tim kiếm các mạng hiện có
    # def scan_networks(self):
    #     # Chạy lệnh nmcli để quét các mạng Wi-Fi
    #     try:
    #         result = subprocess.run(['nmcli', '-f', 'SSID,BSSID,SIGNAL', 'device', 'wifi', 'list'], capture_output=True, text=True)
    #     except Exception as e:
    #         print("Error executing scan nmcli command")
    #         return ""

    #     if result.returncode != 0:
    #         print("Error executing nmcli command")
    #         return ""

    #     # Lấy đầu ra của lệnh nmcli
    #     output = result.stdout
    #     return output

    def scan_networks(self):
        # Chạy lệnh nmcli để quét các mạng Wi-Fi và bao gồm thêm thông tin tốc độ nếu có
        try:
            result = subprocess.run(['nmcli', '-f', 'SSID,BSSID,SIGNAL,RATE', 'device', 'wifi', 'list'], capture_output=True, text=True)
        except Exception as e:
            print("Error executing scan nmcli command:", e)
            return ""

        if result.returncode != 0:
            print("Error executing nmcli command")
            return ""

        # Lấy đầu ra của lệnh nmcli
        output = result.stdout
        return output

    # def disable_autoconnect(self, ssid):
    #     try:
    #         # Tắt chế độ tự động kết nối cho mạng Wi-Fi
    #         subprocess.run(['nmcli', 'connection', 'modify', ssid, 'connection.autoconnect', 'no'], check=True)
            
    #         # Kiểm tra lại cấu hình để chắc chắn autoconnect đã được tắt
    #         result = subprocess.run(['nmcli', 'connection', 'show', ssid], capture_output=True, text=True, check=True)
            
    #         # Kiểm tra xem autoconnect đã được tắt hay chưa
    #         if 'connection.autoconnect: no' in result.stdout:
    #             print(f"Autoconnect for {ssid} is disabled successfully.")
    #         else:
    #             print(f"Failed to disable autoconnect for {ssid}.")
        
    #     except subprocess.CalledProcessError as e:
    #         print(f"Error occurred: {e}")
    #     except Exception as e:
    #         print(f"An unexpected error occurred: {e}")
            
        
    def parse_networks(self, output):
        # Phân tích đầu ra văn bản để lấy thông tin về các mạng Wi-Fi
        networks = []
        lines = output.splitlines()

        # Bỏ qua dòng tiêu đề
        if len(lines) > 1:
            for line in lines[1:]:
                # Tách các cột SSID, BSSID, SIGNAL và RATE từ mỗi dòng
                parts = re.split(r'\s{2,}', line.strip())  
                if len(parts) >= 4:  # Số trường có thể thay đổi, tùy thuộc vào thông tin có sẵn
                    ssid = parts[0]
                    bssid = parts[1]
                    signal = parts[2]
                    rate = parts[3] if len(parts) > 3 else 'Unknown'
                    
                    # Xử lý để loại bỏ đơn vị "Mbit/s"
                    rate = re.sub(r' Mbit/s', '', rate).strip()
                    
                    networks.append([ssid, bssid, signal, rate])
        
        return networks

    
    def filter_networks(self, networks, target_ssid):
        # Lọc các mạng theo SSID cụ thể
        filtered_networks = [network for network in networks if network[0] == target_ssid]
        return filtered_networks
    
    def get_wifi_info(self, interface):
        try:
            # Chạy lệnh iwconfig để lấy thông tin kết nối Wi-Fi
            result = subprocess.run(['iwconfig', interface], capture_output=True, text=True)
            result.check_returncode()

        except subprocess.CalledProcessError:
            print("Error executing iwconfig command")
            return None, None
        
        output = result.stdout
        ssid = None
        bssid = None
        
        # Tìm SSID, BSSID và cường độ tín hiệu từ đầu ra
        ssid_match = re.search(r'ESSID:"([^"]+)"', output)
        bssid_match = re.search(r'Access Point: ([\w:]+)', output)
        # signal_match = re.search(r'Signal level=(-?\d+) dBm', output)

        if ssid_match:
            ssid = ssid_match.group(1)
        
        if bssid_match:
            bssid = bssid_match.group(1)
        
        return ssid, bssid

    def connect_to_wifi_start(self):
            
        result = subprocess.run(['nmcli', 'dev', 'wifi', 'connect', self.SSID, 'password', str(self.PASS)], stdout=subprocess.PIPE, text=True, timeout=15)
        # result = subprocess.run(['nmcli', 'dev', 'wifi', 'connect', str(self.SSID), 'password', str(self.PASS)], stdout=subprocess.PIPE, text=True, timeout=10)
        if "successfully activated" in result.stdout:
            print(f"Đã kết nối thành công với MAC: {self.get_current_wifi_mac()}")
            return 1
        else:
            print("Không thể kết nối với mạng Wi-Fi.")
            return 0

    def connect_to_wifi(self, mac_ap):
        # Kết nối với mạng Wi-Fi sử dụng nmcli
        # try:
            
        # result = subprocess.run(['nmcli', 'dev', 'wifi', 'connect', str(self.SSID), 'password', str(self.PASS)], stdout=subprocess.PIPE, text=True, timeout=10)
        result = subprocess.run(['nmcli', 'dev', 'wifi', 'connect', str(self.SSID), 'bssid', mac_ap, 'password', str(self.PASS)], stdout=subprocess.PIPE, text=True, timeout=10)
        if "successfully activated" in result.stdout:
            # Kiểm tra nếu MAC của điểm truy cập hiện tại khớp với mac_ap
            if str(self.get_current_wifi_mac()) == str(mac_ap):
                print(f"Đã kết nối thành công với MAC: {self.get_current_wifi_mac()}")
                return 1
            else:
                print("Kết nối không đúng với MAC yêu cầu.")
                return 0
        else:
            print("Không thể kết nối với mạng Wi-Fi.")
            return 0

    def loop_connect_to_wifi(self, mac_ap):
        result = subprocess.run(['nmcli', 'dev', 'wifi', 'connect', str(self.SSID), 'bssid', mac_ap, 'password', str(self.PASS)], stdout=subprocess.PIPE, text=True)
        if "successfully activated" in result.stdout:
            # Kiểm tra nếu MAC của điểm truy cập hiện tại khớp với mac_ap
            if str(self.get_current_wifi_mac()) == str(mac_ap):
                print(f"Đã kết nối thành công với MAC: {self.get_current_wifi_mac()}")
                return 1
            else:
                print("Kết nối không đúng với MAC yêu cầu.")
                return 0
        else:
            print("Không thể kết nối với mạng Wi-Fi.")
            return 0


    def get_current_wifi_mac(self):
        try:
            # Chạy lệnh iwconfig để lấy thông tin kết nối Wi-Fi
            result = subprocess.run(
                ['iwconfig'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10
            )
            
            # Lấy thông tin từ stdout
            output = result.stdout
            
            # Tìm địa chỉ MAC của điểm truy cập trong đầu ra
            # Địa chỉ MAC thường được hiển thị sau từ "Access Point:"
            mac_address = re.search(r'Access Point: ([0-9A-Fa-f:]{17})', output)
            
            if mac_address:
                mac_address = mac_address.group(1)
                # print(f"Địa chỉ MAC của điểm truy cập Wi-Fi hiện tại là: {mac_address}")
                return mac_address
            else:
                # print("Không tìm thấy địa chỉ MAC của điểm truy cập Wi-Fi.")
                return None

        except subprocess.CalledProcessError as e:
            print(f"Đã xảy ra lỗi khi thực hiện lệnh iwconfig: {e}")
        except Exception as e:
            print(f"Lỗi không xác định: {e}")

        return None
  
        
    def configure_static_ip_wifi(self, mac_address, ssid, subnet_mask="24"):

        # print("ssid: ", ssid)
        if not ssid:
            print(f"Không tìm thấy SSID tương ứng với MAC: {mac_address}")
            return

        try:
            # Cấu hình địa chỉ IP tĩnh, gateway, DNS trong một lệnh duy nhất
            ip_configuration = f'{self.ip_address}/{subnet_mask}'
            subprocess.run(['nmcli', 'connection', 'modify', ssid, 'ipv4.addresses', ip_configuration, 
                            'ipv4.gateway', self.gateway, 'ipv4.dns', self.dns, 'ipv4.method', 'manual', 'ipv6.method', 'ignore'], check=True)

            # Kích hoạt lại kết nối để áp dụng cấu hình mới
            subprocess.run(['nmcli', 'connection', 'up', ssid], check=True)

            print(f"IP tĩnh đã được cấu hình thành công cho mạng {ssid} với MAC {mac_address}.")
            self.config_iplan_oke = 0
            
        except subprocess.CalledProcessError as e:
            print(f"Đã xảy ra lỗi khi cấu hình IP tĩnh: {e}")

    def subnet_to_prefix(self, subnet_mask):
        mask_bits = sum([bin(int(x)).count('1') for x in subnet_mask.split('.')])
        return mask_bits
    
    def config_static_ip_lan(self, mac_address, subnet_mask="24"):
        
        ip_configuration = f'{self.iplan}/{subnet_mask}'
        # ip_configuration = f'{self.iplan}/{self.subnet_to_prefix(subnet_mask)}'
        
        try:
            subprocess.run(['nmcli', 'connection', 'modify', 'Wired connection 1', 'ipv4.addresses', ip_configuration, 
                            'ipv4.method', 'manual', 'ipv6.method', 'ignore'], check=True)
            subprocess.run(['nmcli', 'connection', 'up', 'Wired connection 1'], check=True)
            print(f"IP LAN tĩnh đã được cấu hình thành công cho MAC {mac_address}.")

        except subprocess.CalledProcessError as e:
            print(f"Đã xảy ra lỗi khi cấu hình IP LAN tĩnh: {e}")
        
    
    def convert_level_strength(self, strength):
        if 75 < strength and strength <= 100:
            return 4
        elif 50 < strength and strength < 76:
            return 3
        elif 25 < strength and strength < 51:
            return 2
        else:
            return 1
    
    def usbSerial_check(self, nameport):
        try:
            output = subprocess.check_output("ls {} {} {} {}".format('/dev/','|','grep', nameport ), shell=True)
            locate = str(output).find(nameport)
            if locate != -1:
                return 1
            return 0
        except Exception as e:
            return 0
    
    def ethernet_check(self, address):
        try:
            # print address
            output = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
            # print(output)
            result = str(output).find('time=') 
            # print(result) # yes : >0 
            if result != -1:
                return 1
            return 0
        except Exception as e:
            return 0

    def toggle_wifi(self, enable=True):
        try:
            if enable:
                # Bật Wi-Fi
                subprocess.run(['nmcli', 'r', 'wifi', 'on'], check=True)
                rospy.sleep(3)
            else:
                # Tắt Wi-Fi
                subprocess.run(['nmcli', 'r', 'wifi', 'off'], check=True)
                print("Wi-Fi đã được tắt")
        except subprocess.CalledProcessError as e:
            print(f"Đã xảy ra lỗi: {e}")

    def check_and_toggle_wifi(self):
        try:
            # Kiểm tra trạng thái của Wi-Fi bằng nmcli
            result = subprocess.run(['nmcli', 'radio', 'wifi'], stdout=subprocess.PIPE, text=True)
            
            wifi_status = result.stdout.strip()
            
            # Nếu Wi-Fi đang tắt, bật lại
            if wifi_status == 'disabled':
                print("Wi-Fi đang tắt, bật Wi-Fi...")
                self.toggle_wifi(enable=True)  # Bật Wi-Fi
            else:
                # print("Wi-Fi đã được bật.")
                pass
        
        except subprocess.CalledProcessError as e:
            print(f"Đã xảy ra lỗi khi kiểm tra trạng thái Wi-Fi: {e}")


    def usbCamera_check(self,nameport):
        try:
            output = subprocess.check_output("rs-enumerate-devices -{}".format('s'), shell=True)
            # print(output)
            vitri = str(output).find(nameport[1:])
            # name = output[(vitri):(vitri+len(nameport))]
            # print(result)
            # print(nameport[1:])
            if vitri > 1 : return 1
        except Exception as e:
            # print("no port")
            return 0


    def run(self):
        self.toggle_wifi(enable=True)
        
        while not rospy.is_shutdown():
            # print("Hello huhuhu")
            self.check_and_toggle_wifi() #Kiem tra xem wifi co bi tat thu cong ko

            ###########################################################################################################

            # -- NUC mac
            self.NUC_info.nuc_mac = self.get_MAC(self.name_card)

            # -- NUC name
            self.NUC_info.nuc_name = self.get_hostname()

            # -- NUC IP Wifi
            self.NUC_info.nuc_ipWifi = self.get_ipAuto(self.name_card)

            # -- NUC IP Ethernet
            # self.NUC_info.nuc_ipEthernet = self.get_ethernet_ip(self.name_eth)

            # -- CPU usage
            self.NUC_info.cpu_usage = self.get_cpu_usage(self.time_read)

            # -- CPU temp
            self.NUC_info.cpu_temp = self.get_cpu_temp()

            # -- RAM usage
            self.NUC_info.ram_usage = int(self.get_ram_usage() / 1024 / 1024)
            self.NUC_info.ram_usage = round(self.NUC_info.ram_usage/1000, 1)

            # -- RAM total
            self.NUC_info.ram_total = int(self.get_ram_total() / 1024 / 1024)
            self.NUC_info.ram_total = round(self.NUC_info.ram_total/1000, 1)
            
            # -- RAM percent
            self.NUC_info.ram_percent = self.get_ram_usage_pct()

            # -- Wifi
            self.NUC_info.wifi_quality, self.NUC_info.wifi_signal = self.get_qualityWifi(self.name_card)

            # -- Access point 
            self.NUC_info.ap_mac = self.get_AccessPointMAC(self.name_card)


            # -- Ping server
            self.NUC_info.ping_server = self.ping_traffic(self.address_traffic)

            # -- Time run
            if time.time() - self.start_time >= 1:
                self.start_time = time.time()
                self.uptime = self.uptime + 1
            
            self.NUC_info.uptime = self.convert_intTotime(self.uptime)
            
            ###########################################################################################################
            
            self.MAC_lan = self.get_MAC(self.name_card_lan)
            if self.config_iplan_oke == 1:
                self.config_static_ip_lan(self.MAC_lan)
                self.config_iplan_oke = 0
            
            ###########################################################################################################
            
            #   Lidar front -> nav350/nanoscan3/lms1xx...
            if self.ethernet_check(self.portLidar_1) == 1:
                self.statusPort.lidar_1 = 1
            else:
                self.statusPort.lidar_1 = 0
            #   Lidar behind -> ...
            if self.ethernet_check(self.portLidar_2) == 1:
                self.statusPort.lidar_2 = 1
            else:
                self.statusPort.lidar_2 = 0  
            #    ROS to CAN communicate
            if self.usbSerial_check(self.portRTC) == 1:
                self.statusPort.rtc = 1
            else:
                self.statusPort.rtc = 0
            #   Driver control motor
            if self.usbSerial_check(self.portDriverAll) == 1:
                self.statusPort.driverall = 1
            else:
                self.statusPort.driverall = 0
            #   camera
            
            # if self.usbCamera_check(self.portCamera) == 1:
            #     self.statusPort.camera = 1
            # else:
            #     self.statusPort.camera = 0
            
            # if self.usbSerial_check(self.portIMU) == 1:
            #     self.statusPort.imu = 1
            # else:
            #     self.statusPort.imu = 0
            

            # if self.usbSerial_check(self.portMagline_front) == 1:
            #     self.statusPort.magline_front = 1
            # else:
            #     self.statusPort.magline_front = 0

            # if self.usbSerial_check(self.portMagline_behind) == 1:
            #     self.statusPort.magline_behind = 1
            # else:
            #     self.statusPort.magline_behind = 0
            

            # if self.usbSerial_check(self.portRFID_1) == 1:
            #     self.statusPort.rfid_1 = 1
            # else:
            #     self.statusPort.rfid_1 = 0

            # if self.usbSerial_check(self.portRFID_2) == 1:
            #     self.statusPort.rfid_2 = 1
            # else:
            #     self.statusPort.rfid_2 = 0


            # if self.usbSerial_check(self.portLed) == 1:
            #     self.statusPort.led = 1
            # else:
            #     self.statusPort.led = 0
            
            # if self.usbSerial_check(self.portSpeaker) == 1:
            #     self.statusPort.speaker = 1
            # else:
            #     self.statusPort.speaker = 0

            
            
            
            ###########################################################################################################
            
            if self.process == 1:
                ssid, bssid = self.get_wifi_info(self.name_card)  

                print("MAC:", bssid)
                if ssid != None:

                    print("ssid", ssid)
                    print("self.SSID", self.SSID)
                    if ssid != self.SSID:
                        self.connect_to_wifi_start()
                    else:
                        self.wifi_info.status_wifi = True                                          #wifi dang co 
                        self.MAC_APNOW = bssid
                        self.process = 2
                        if self.check_change_wifi == 1:
                            print("dang doi thanh ip tinhhhhhhhh")
                            self.configure_static_ip_wifi(self.MAC_APNOW, ssid)
                            self.check_change_wifi = 0
                        
                else:
                    self.wifi_info = Wifi_info()
                    self.wifi_info.status_wifi = False                                          #wifi dang bi mat
                    self.pub_wifi.publish(self.wifi_info)
                    
                    all_network = self.scan_networks()                                      #Tìm kiếm tất cả mạng hiện có 
                    
                    if all_network != "":                                                   #Nếu không có mạng nào sẽ kết nối với mạng có tín hiệu mạnh nhất hiện tại
                        networks = self.parse_networks(all_network) 
                        filtered_networks = self.filter_networks(networks, self.SSID)
                        # self.wifi_info.datawifi = filtered_networks
                        signal_level = 0
                        for info_ap in filtered_networks:
                            if int(info_ap[2]) > int(signal_level):
                                self.MAC_APNOW = info_ap[1]
                                signal_level = info_ap[2]
                        self.status_connect_wifi = self.loop_connect_to_wifi(self.MAC_APNOW)     #Hàm kết nối mạng
                        print("status connect wifi", self.status_connect_wifi)
                        self.process = 1                #                                   #Chuyển về vòng lặp 1

            # quét dữ liệu scan và tìm cương độ MAC AP đang connect
            elif self.process == 2:
                self.process = 1
                self.list_data_wifi = []

                if time.time() - self.saveTime_reconnectWifi > 2.:
                    try:
                        all_network = self.scan_networks()
                        # print("all_network", all_network)
                        
                        if all_network != "":
                            networks = self.parse_networks(all_network)
                            # print("networks", networks)
                            # print("parse_networks", networks)
                            filtered_networks = self.filter_networks(networks, self.SSID)
                            # print("filtered_networks", filtered_networks)
                            
                            # self.wifi_info.datawifi = filtered_networks
                            self.singal_now = -1
                            for info_ap in filtered_networks:
                                
                                
                                new_data_wifi = CustomDataWifi()
                                new_data_wifi.ssid = self.SSID
                                new_data_wifi.mac_address = info_ap[1]
                                new_data_wifi.signal_strength = int(info_ap[2])
                                new_data_wifi.level_strength = self.convert_level_strength(int(info_ap[2]))
                                new_data_wifi.rate = int(info_ap[3])

                                self.list_data_wifi.append(new_data_wifi)

                                if self.MAC_APNOW == info_ap[1]:
                                    self.signal_now = int(info_ap[2])
                                    self.rate_now = int(info_ap[3])
                                    self.wifi_info.ap_strength = int(info_ap[2])

                                    break
                            # if self.signal_now != -1 and self.signal_now < self.roaming_intensity:
                            
                            
                            temp = self.signal_now
                            temp_rate = self.rate_now
                            print("temp: ", temp)

                            if self.signal_now < 60:
                                for info_ap in filtered_networks:
                                    if int(info_ap[3]) >= temp_rate and int(info_ap[2]) > self.signal_now + 30 and int(info_ap[2]) > temp and str(info_ap[1]) != str(self.MAC_APNOW):
                                        
                                        print("info_ap[2]", info_ap[2])
                                        print("info_ap[3]", info_ap[3])
                                        temp = int(info_ap[2])
                                        self.MAC_APSELECT = info_ap[1]
                                        print(f"Thay doi AP co MAC: {self.MAC_APSELECT}")
                                        self.process = 3
                                    else:
                                        print("khong co wifi manh hon")
                                        pass
                                    
                            else:
                                pass

                            # if self.singal_now != -1 and self.singal_now <= self.roaming_intensity and self.singal_now < int(filtered_networks[0][2]):
                            #     self.MAC_APSELECT = filtered_networks[0][1]
                            #     print(f"Thay doi AP co MAC: {self.MAC_APSELECT}")
                            #     self.process = 3
                            # print("self.list_data_wifi", self.list_data_wifi)
                            
                        else:
                            self.wifi_info.err = 444
                            
                    except Exception as e:
                        self.wifi_info.err = 333
                        print("Error executing find target wifi command: ", e)

                    self.saveTime_reconnectWifi = time.time()

            # bắt lại BSSID khác nếu cường độ điểm truy cập yếu
            elif self.process == 3:
                status_connect = self.connect_to_wifi(self.MAC_APSELECT)
                print("Da thay doi wifi theo mac manh nhat ^^^^^")
                self.MAC_APNOW = self.MAC_APSELECT  
                rospy.sleep(2)
                self.process = 1
                print("self.MAC_APNOW", self.MAC_APNOW)
                self.configure_static_ip_wifi(self.MAC_APNOW, ssid)
                # self.check_change_wifi = 1
                # if status_connect:
                #     self.process = 1
                #     self.check_change_wifi = 1

            self.wifi_info.datawifi = self.list_data_wifi
            self.wifi_info.bssid = self.MAC_APNOW
            
            self.wifi_info.ap_signal = self.convert_level_strength(self.singal_now)            
            
            self.pub_NUC.publish(self.NUC_info)
            self.pub_check_port.publish(self.statusPort)
            self.pub_wifi.publish(self.wifi_info)
            # self.disable_autoconnect(self.SSID)

            ###########################################################################################################

            self.rate.sleep()

def main():

    print('Program starting')
    try:
        program = NUC_Info()
        program.run()
    except rospy.ROSInterruptException:
        pass
    print('Programer stopped')

if __name__ == '__main__':
    main()


