#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Creation date: 5/9/2024
Detail: tự động chuyển vùng khi cường độ wifi yếu 
Version: 1.0
Company: STI Việt Nam
'''

# get ip
import os                                                                                                                                                           
import re
import subprocess 

import sys
import struct
import time
from decimal import *
import math

class controlWifi():
    def __init__(self):
        # self.name_card = "wlp2s0" # "wlp0s20f3"
        self.name_card = "wlp4s0" # "wlp0s20f3"

        # -- 
        self.SSID = 'STI_VietNam_No8'
        self.PASS = '66668888'

        # self.SSID = 'VNAGV'
        # self.PASS = 'Avery@vnagv2020'

        self.MAC_APNOW = ''
        self.singal_now = 0

        self.MAC_APSELECT = ''

        self.roaming_intensity = 30
          
        self.saveTime_reconnectWifi = 0
        # -- 
        self.process = 1

    def get_wifi_info(self, interface='wlan0'):
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
    
    def scan_networks(self):
        # Chạy lệnh nmcli để quét các mạng Wi-Fi
        try:
            result = subprocess.run(['nmcli', '-f', 'SSID,BSSID,SIGNAL', 'device', 'wifi', 'list'], capture_output=True, text=True)
        except Exception as e:
            print("Error executing scan nmcli command")
            return ""

        if result.returncode != 0:
            print("Error executing nmcli command")
            return ""

        # Lấy đầu ra của lệnh nmcli
        output = result.stdout
        return output

    def parse_networks(self, output):
        # Phân tích đầu ra văn bản để lấy thông tin về các mạng Wi-Fi
        networks = []
        lines = output.splitlines()
        
        # Bỏ qua dòng tiêu đề
        if len(lines) > 1:
            for line in lines[1:]:
                # Tách các cột SSID, BSSID, SIGNAL từ mỗi dòng
                parts = re.split(r'\s{2,}', line.strip())  # Sử dụng biểu thức chính quy để phân tách
                if len(parts) == 3:
                    ssid = parts[0]
                    bssid = parts[1]
                    signal = parts[2]
                    networks.append([ssid, bssid, signal])
        
        return networks

    def filter_networks(self, networks, target_ssid):
        # Lọc các mạng theo SSID cụ thể
        filtered_networks = [network for network in networks if network[0] == target_ssid]
        return filtered_networks

    def connect_to_wifi(self, mac_ap):
        # Kết nối với mạng Wi-Fi sử dụng nmcli
        try:
            result = subprocess.run(['nmcli', 'dev', 'wifi', 'connect', self.SSID, 'password', self.PASS], stdout=subprocess.PIPE, text=True, timeout=10)
            if "successfully activated" in result.stdout:
                print(f"Da ket noi thanh cong den mang wifi {self.SSID}.")
                return 1
            else:
                print(f"Khong the ket noi voi mang wifi {self.SSID}.")

        except Exception as e:
            print("fail to connect wifi")

        return 0

    def is_connected(self, ssid):
        # Lấy thông tin kết nối Wi-Fi hiện tại
        result = subprocess.run(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"], capture_output=True, text=True)

        print(result)
        
        # Kiểm tra xem có SSID nào đang được kết nối không
        for line in result.stdout.splitlines():
            active, current_ssid = line.split(":")
            if active == "yes" and current_ssid == ssid:
                return 1
            
        return 0

    def run(self):	
        # kiểm tra kết nối wifi hiện tại + lấy địa chỉ MAC wifi kết nối
        if self.process == 1:
            ssid, bssid = self.get_wifi_info(self.name_card)
            print(f"ssid: {ssid}, mac: {bssid}")

            if ssid != None:
                self.MAC_APNOW = bssid
                self.process = 2

        # quét dữ liệu scan và tìm cương độ MAC AP đang connect
        elif self.process == 2:
            self.process = 1

            if time.time() - self.saveTime_reconnectWifi > 2.:
                try:
                    all_network = self.scan_networks()
                    
                    if all_network != "":
                        networks = self.parse_networks(all_network)
                        # print("parse_networks", networks)
                        filtered_networks = self.filter_networks(networks, self.SSID)
                        print("filtered_networks", filtered_networks)

                        self.singal_now = -1
                        for info_ap in filtered_networks:
                            signal_level = info_ap[2]
                            if self.MAC_APNOW == info_ap[1]:
                                self.singal_now = int(signal_level)
                                print(f"Cuong do wifi tai MAC {self.MAC_APNOW} la: {self.singal_now}")
                                break

                        if self.singal_now != -1 and self.singal_now <= self.roaming_intensity and self.singal_now < int(filtered_networks[0][2]):
                            self.MAC_APSELECT = filtered_networks[0][1]
                            print(f"Thay doi AP co MAC: {self.MAC_APSELECT}")
                            self.process = 3

                except Exception as e:
                    print("Error executing find target wifi command: ", e)

                self.saveTime_reconnectWifi = time.time()

        # bắt lại BSSID khác nếu cường độ điểm truy cập yếu
        elif self.process == 3:
            status_connect = self.connect_to_wifi(self.MAC_APSELECT)
            if status_connect:
                self.process = 1

        time.sleep(0.1)

def main():

	print('Starting main program')
	class_1 = controlWifi()		
	while True:
		class_1.run()

if __name__ == '__main__':
    main()
    
#
