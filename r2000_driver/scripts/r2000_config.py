
import requests

# Địa chỉ IP của LiDAR (Thay bằng IP thực tế của bạn)
LIDAR_IP = "169.254.32.205"

# Lệnh cần gửi (ví dụ: Feed Watchdog)
# command_url = f"http://{LIDAR_IP}/cmd/list_iq_parameters"
command_url = f"http://{LIDAR_IP}/cmd/set_parameter?filter_type=none"

try:
    # Gửi yêu cầu GET đến LiDAR
    response = requests.get(command_url)

    # In phản hồi từ LiDAR
    print("Phản hồi từ LiDAR:", response.json())  # Nếu phản hồi là JSON

except requests.exceptions.RequestException as e:
    print(f"Lỗi kết nối: {e}")
