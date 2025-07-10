import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from message_pkg.msg import MagneticLine  # Message có 2 trường: int8 status, float64 valu
import numpy as np
import scipy.linalg

class LQRControllerNode(Node):
    def __init__(self):
        super().__init__('lqr_controller_node')
        self.killnode = 0
        
        # Lấy tham số vận tốc dài v (có thể điều chỉnh)
        self.declare_parameter('v', 0.3)
        self.v = self.get_parameter('v').value

        self.follow_ahead = 1
        
        # Mô hình động học hệ thống:
        # x = [error, error_dot] với error = (msg.valu - 7.5)
        # A = [[0, v],
        #      [0, 0]],   B = [[0], [1]]
        self.A = np.array([[0, self.v],
                           [0, 0]])
        self.B = np.array([[0],
                           [1]])
        # Chọn ma trận trọng số Q và R
        '''
        (đặc biệt q1 cho sai số vị trí), hệ thống sẽ cố gắng quá mức để loại bỏ sai số, dẫn đến phản ứng quá nhanh
        q2 để giảm mức độ “khắt khe” của bộ điều khiển.
        Giá trị R cao hơn sẽ “trừng phạt” tín hiệu điều khiển lớn, giúp giảm hiệu ứng điều khiển quá mạnh và làm cho phản ứng trở nên mượt mà hơn.
        '''
        self.Q = np.array([[0.05, 0],
                           [0, 1.]])
        self.R = np.array([[300.]])
        
        # Giải phương trình Riccati để tìm ma trận P và tính K
        P = scipy.linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ self.B.T @ P

        print(self.K)
        
        self.get_logger().info(f"LQR K matrix: {self.K}")
        
        # Khởi tạo các biến để tính tốc độ sai số (error derivative)
        self.previous_error = None
        self.previous_time = self.get_clock().now().nanoseconds / 1e9
        
        # Publisher để xuất lệnh điều khiển dạng Twist
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscriber nhận dữ liệu từ cảm biến MagneticLine
        self.create_subscription(MagneticLine, '/magneticLine_front', self.sensor_callback, 10)
    
    def sensor_callback(self, msg):
        # Tính sai số: so sánh giá trị cảm biến với 7.5 (giá trị giữa line)
        if msg.value == 20: 
            error = 0
        else:
            error = msg.value - 7.5
        
        # Tính thời gian hiện tại (tính bằng giây)
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.previous_time
        if dt <= 0.0:
            dt = 0.01  # đảm bảo dt > 0
        
        # Nếu chưa có giá trị trước, khởi tạo error_dot = 0
        if self.previous_error is None:
            error_dot = 0.0
        else:
            error_dot = (error - self.previous_error) / dt
        
        # Cập nhật các biến lưu trữ
        self.previous_error = error
        self.previous_time = current_time
        
        # Tạo vector trạng thái x = [error, error_dot]
        x = np.array([[error],
                      [error_dot]])
        # Tính toán tín hiệu điều khiển u = -Kx (là vận tốc góc w)
        u = -self.K @ x
        w = float(u)
        
        # Tạo message Twist: dùng vận tốc dài v cố định và vận tốc góc w vừa tính được
        twist_msg = Twist()
        twist_msg.linear.x = self.v
        twist_msg.angular.z = w
        self.cmd_pub.publish(twist_msg)
        
        self.get_logger().info(f"Sensor valu: {msg.value:.2f} | Error: {error:.2f} | Error_dot: {error_dot:.2f} | w: {w:.2f}")

    # -- 
    def shutdown_hook(self):
        for i in range(2):
            self.cmd_pub.publish(Twist())

        self.get_logger().warning("Shutting down. cmd_vel will be 0")

def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')

if __name__ == '__main__':
    main()
