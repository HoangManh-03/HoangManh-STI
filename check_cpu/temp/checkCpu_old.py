#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author : Phùng Quý Dương - Archie Phùng
# Date: 23-05-2024

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
        print("ROS Initial!")
        rospy.init_node('nuc_info', anonymous= True) # , NoRosout = True
        self.rate = rospy.Rate(10)

        self.pub_NUC = rospy.Publisher('/nuc_info', Nuc_info, queue_size= 10)            # 1 topic tương ứng với 1 kiểu dữ liệu.
        self.NUC_info = Nuc_info()

        self.time_read = 0.5
        self.name_card = rospy.get_param("name_card", "wlo2")
        self.name_card = "wlo2"
        self.name_eth = rospy.get_param("name_eth", "eno1")
        self.address_traffic = rospy.get_param("address_traffic", "172.21.15.224")
        self.runOnce = 0
        self.start_time = time.time()
        self.step = 1
        self.time_checkWifi = time.time()
        self.time_getApmac = time.time()
        self.time_getPing = time.time()
        self.uptime = 0
        
        self.SSID = 'STI_VietNam_No8'
        self.PASS = '66668888'
        self.process = 1
        self.MAC_APNOW = ''
        self.MAC_APSELECT = ''
        self.singal_now = 0
        self.roaming_intensity = 30
        self.saveTime_reconnectWifi = 0

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
            # print ("signal_data: ", int(signal_out)) 

            # -
            # if pos_bitrate >= 0:
            #     bitrate_data = str(output)[pos_bitrate+13:pos_bitrate+16]
            # # print ("bitrate_data: ", int(bitrate_data)) 

            # # -
            # if pos_txpower >= 0:
            #     txpwower_data = str(output)[pos_txpower+13:pos_txpower+16]
            # print ("txpwower_data: ", int(txpwower_data)) 

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
        
    #
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
    
    def roaming(self):
            
        pass
    
    
    def run(self):
        while not rospy.is_shutdown():
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

            self.pub_NUC.publish(self.NUC_info)
            
            
            if self.process == 1:
                ssid, bssid = self.get_wifi_info(self.name_card)

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


            self.rate.sleep()
            
            

def main():
	print('Program starting')

	program = NUC_Info()
	program.run()

	print('Programer stopped')

if __name__ == '__main__':
    main()
