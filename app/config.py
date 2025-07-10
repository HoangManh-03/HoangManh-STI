import netifaces
import uuid
import os
import subprocess
from datetime import datetime
import re

"""
Developer: Le Duc Anh
Company: STI Viet Nam
Date: 13/03/2024
"""


def get_ipAuto(name_card): # name_card : str()
    try:
        address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
        print ("address: ", address)
        return address
    except Exception:
        return "-1"



def get_MAC(name_card): # name_card : str()
    try:
        MAC = ''
        output = os.popen("ip addr show {}".format(name_card) ).read()
        pos1 = str(output).find('link/ether ') # tuyet doi ko sua linh tinh.
        pos2 = str(output).find(' brd')   # tuyet doi ko sua linh tinh.

        if (pos1 >= 0 and pos2 > 0):
            MAC = str(output)[pos1+11:pos2]

        print ("MAC: ", MAC)
        return MAC
    except Exception:
        return "-1"
# --
def get_qualityWifi(name_card): # int
    try:
        quality_data = '0'
        output = os.popen("iwconfig {}".format(name_card)).read()
        pos_quality = str(output).find('Link Quality=')
        # -
        if pos_quality >= 0:
            quality_data = str(output)[pos_quality+13:pos_quality+15]

        
        return int(quality_data)
    except Exception:
        return 0
# ---
def get_hostname():
    try:
        output = os.popen("hostname").read()
        # print ("output: ", output)
        leng = len(output)
        hostname = str(output)[0:leng-1]
        print ("hostname: ", hostname)
        return hostname
    except Exception:
        return "-1"

def ping_traffic(address):
    try:
        ping = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
        # print(ping)
        vitri = str(ping).find("time")
        time_ping = str(ping)[(vitri+5):(vitri+9)]
        # print (time_ping)
        return str(float(time_ping))
    except Exception:
        return '-1'
    
def get_time():
    current_time = datetime.now()
    formatted_time = current_time.strftime("%H:%M")
    return formatted_time

name_card = 'wlp0s20f3'

if __name__ == '__main__':
    
    

    print(get_ipAuto(name_card))
    print(get_MAC(name_card))
    print(get_qualityWifi(name_card))
    print(get_hostname())

