import socketio

# Địa chỉ IP của cảm biến LiDAR (thay thế bằng IP thực tế)
sensor_ip = "169.254.32.205"
server_url = f"http://{sensor_ip}"

# Khởi tạo client Socket.IO
sio = socketio.Client()

try:
    # Kết nối đến LiDAR
    sio.connect(server_url)
    print("Kết nối thành công!")

    # # Gửi lệnh thiết lập tham số
    # command = {
    #     "cmd": "set_parameter",
    #     "filter_type": "none"
    # }
    # sio.emit("cmd", command)  # Gửi lệnh đến LiDAR

    # # Lắng nghe phản hồi từ server
    # @sio.on("response")
    # def on_response(data):
    #     print("Phản hồi từ LiDAR:", data)

    # # Đợi một chút để nhận phản hồi
    # sio.wait(seconds=5)

except Exception as e:
    print(f"Lỗi kết nối: {e}")

finally:
    sio.disconnect()
