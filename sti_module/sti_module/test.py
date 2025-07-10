import rclpy
from rclpy.node import Node
from message_pkg.msg import *

class LineFollowerPID:
    def __init__(self, data):
        self.data = data

    def run(self):
        print(self.data)

class DualTimerNode(Node):
    def __init__(self):
        super().__init__('dual_timer_node')

        # -- front magline
        self.sub_maglineFront = self.create_subscription(
            MagneticLine,
            "magneticLine_front",
            self.callBack_maglineFront,
            10)
        self.sub_maglineFront

        self.is_magline_front = False
        self.data_mangline_front = MagneticLine()

        self.test = LineFollowerPID(self.data_mangline_front)

        # Tạo timer 1 chạy mỗi 1 giây
        self.timer1 = self.create_timer(1.5, self.timer_callback_1)
        
        # Tạo timer 2 chạy mỗi 0.5 giây
        self.timer2 = self.create_timer(1.5, self.timer_callback_2)

    def callBack_maglineFront(self, data):
        # self.data_mangline_front = data
        self.is_magline_front = True

    def timer_callback_1(self):
        self.data_mangline_front.value += 1
        pass

    def timer_callback_2(self):
        self.test.run()

def main(args=None):
    rclpy.init(args=args)
    node = DualTimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

