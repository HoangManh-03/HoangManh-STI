import websocket
import threading
import time

class WebSocketClient:
    def __init__(self, url):
        self.url = url
        self.ws = None
        self.is_connected = False

    def on_open(self, ws):
        """Xử lý sự kiện khi kết nối WebSocket được mở."""
        print("Kết nối WebSocket đã mở.")
        self.is_connected = True

    def on_message(self, ws, message):
        """Xử lý sự kiện khi nhận được tin nhắn."""
        print("Tin nhắn nhận được:", message)

    def on_error(self, ws, error):
        """Xử lý sự kiện khi có lỗi xảy ra."""
        print("Đã xảy ra lỗi:", error)

    def on_close(self, ws, close_status_code, close_msg):
        """Xử lý sự kiện khi kết nối WebSocket bị đóng."""
        print("Kết nối WebSocket đã đóng.")
        self.is_connected = False
        self.reconnect()  # Gọi phương thức để kết nối lại

    def reconnect(self):
        """Cố gắng kết nối lại."""
        while not self.is_connected:
            print("Đang cố gắng kết nối lại...")
            time.sleep(5)  # Đợi 5 giây trước khi cố gắng kết nối lại
            self.run()

    def run(self):
        """Chạy WebSocket và giữ kết nối liên tục."""
        self.ws = websocket.WebSocketApp(
            self.url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        self.ws.run_forever()

    def send_message(self, message):
        """Gửi tin nhắn tới máy chủ nếu kết nối đang hoạt động."""
        if self.is_connected:
            self.ws.send(message)
        else:
            print("Không thể gửi tin nhắn. WebSocket chưa kết nối.")

    def close(self):
        """Đóng kết nối WebSocket."""
        if self.ws:
            self.ws.close()


def main():
    url = "ws://192.168.1.54:8765"  # Thay đổi URL này cho phù hợp với máy chủ WebSocket của bạn
    client = WebSocketClient(url)

    # Tạo một luồng để chạy WebSocket
    ws_thread = threading.Thread(target=client.run)
    ws_thread.start()

    time.sleep(1)  # Đợi một chút để đảm bảo kết nối được thiết lập
	

    # Gửi tin nhắn liên tục
    try:
        while True:
            message = "Xin chào từ WebSocket Client!"
            client.send_message(message)
            time.sleep(0.1)  # Thay đổi thời gian giữa các lần gửi tin nhắn ở đây
    except KeyboardInterrupt:
        print("Đóng kết nối...")
        client.close()
        ws_thread.join()  # Đợi cho luồng WebSocket kết thúc

if __name__ == "__main__":
    main()
