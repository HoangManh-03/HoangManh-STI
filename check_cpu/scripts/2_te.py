import subprocess

def find_ssid_by_mac(mac_address):
    """Tìm SSID của mạng dựa trên địa chỉ MAC"""
    result = subprocess.run(['nmcli', '-t', '-f', 'DEVICE,SSID,BSSID', 'device', 'wifi'], capture_output=True, text=True)
    # print(result)
    for line in result.stdout.splitlines():
        parts = line.split(':')
        if len(parts) >= 3:
            device = parts[0]
            # ssid = ':'.join(parts[1:-1])  # SSID có thể chứa dấu ':', vì vậy ghép lại các phần từ vị trí thứ 2 đến trước cuối
            # bssid = ':'.join([p.replace('\\', '') for p in parts[2:]])
            ssid = ':'.join(parts[1:2]).replace('\\', '')  # Xóa ký tự '\'
            bssid = ':'.join([p.replace('\\', '') for p in parts[2:]])

            print(ssid)
            if bssid.lower() == mac_address.lower():
                return ssid
    return None

def configure_static_ip_by_mac(interface, mac_address, ip_address, gateway, dns, subnet_mask="24"):

    ssid = find_ssid_by_mac(mac_address)
    # print("ssid: ", ssid)
    if not ssid:
        print(f"Không tìm thấy SSID tương ứng với MAC: {mac_address}")
        return

    try:
        # Cấu hình địa chỉ IP tĩnh, gateway, DNS trong một lệnh duy nhất
        ip_configuration = f'{ip_address}/{subnet_mask}'
        subprocess.run(['nmcli', 'connection', 'modify', ssid, 'ipv4.addresses', ip_configuration, 
                        'ipv4.gateway', gateway, 'ipv4.dns', dns, 'ipv4.method', 'manual'], check=True)

        # Kích hoạt lại kết nối để áp dụng cấu hình mới
        subprocess.run(['nmcli', 'connection', 'up', ssid], check=True)

        print(f"IP tĩnh đã được cấu hình thành công cho mạng {ssid} với MAC {mac_address}.")
    
    except subprocess.CalledProcessError as e:
        print(f"Đã xảy ra lỗi khi cấu hình IP tĩnh: {e}")

# Thông tin cần thiết
interface = "wlp4s0"
mac_address = "7A:83:C2:8B:46:FB"  # Địa chỉ MAC của thiết bị
ip_address = "192.168.1.100"
gateway = "192.168.1.1"
dns = "8.8.8.8"
subnet_mask = "24"  # Prefix length for the subnet mask (24 for 255.255.255.0)

# Gọi hàm cấu hình IP tĩnh
configure_static_ip_by_mac(interface, mac_address, ip_address, gateway, dns, subnet_mask)
