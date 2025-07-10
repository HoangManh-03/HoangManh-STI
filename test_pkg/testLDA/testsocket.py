import rospy
import socket

# Tạo socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Địa chỉ và cổng của server
server_address = ('192.168.1.36', 1997)

# Gửi dữ liệu đến server
message = "Hello, server!"
client_socket.sendto(message.encode('utf-8'), server_address)

# Nhận phản hồi từ server
while True:
    response, _ = client_socket.recvfrom(1024)
    print(f"Phản hồi từ server: {response.decode('utf-8')}")
    # rospy.sleep(1)

# # Đóng kết nối
# client_socket.close()

