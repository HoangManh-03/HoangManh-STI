from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys
import os
import rospy
from std_msgs.msg import String

class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        try:
            rospy.init_node('listener', anonymous=True)
            self.subscriber = rospy.Subscriber("chatter", String, self.receive_callback)
            print("ROS node initialized successfully")
        except Exception as e:
            print(f"Error initializing ROS: {e}")
            self.publisher = None
        
        try:
            uic.loadUi("receive.ui", self)
        except Exception as e:
            print(f"Error loading UI file: {e}")
            sys.exit(1)
    def receive_callback(self, msg):
        try:
            received_text = msg.data
            rospy.loginfo(f"Received: {received_text}")
            self.firstName.setText(received_text)
        except Exception as e:
            print(f"Error in receive_callback: {e}")

if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        window = MyApp()
        window.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("Interrupted by user")
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Interrupted by user")
        sys.exit(0)