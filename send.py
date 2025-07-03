from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys
import os
import rospy
from std_msgs.msg import String

class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.count = 0
        
        # Initialize ROS node ONCE at startup
        try:
            rospy.init_node('talker', anonymous=True)
            self.publisher = rospy.Publisher('chatter', String, queue_size=10)
            print("ROS node initialized successfully")
        except Exception as e:
            print(f"Error initializing ROS: {e}")
            self.publisher = None
        
        try:
            uic.loadUi("test.ui", self)
        except Exception as e:
            print(f"Error loading UI file: {e}")
            sys.exit(1)

        # Initialize LCD display
        if hasattr(self, 'lcdNumber'):
            self.lcdNumber.display(self.count)

        # Connect signals with error handling
        try:
            self.send.clicked.connect(self.send_text)
            self.exit.clicked.connect(self.close_app)
            self.increase.clicked.connect(self.increase_number)
            self.decrease.clicked.connect(self.decrease_number)
        except AttributeError as e:
            print(f"Error connecting signals: {e}")

    def send_text(self):
        """Send text to ROS topic without blocking the GUI"""
        try:
            # Get text from input
            source_text = self.lineEdit.text()
            
            # Publish to ROS topic (non-blocking)
            if self.publisher and not rospy.is_shutdown():
                msg = String()
                msg.data = source_text
                self.publisher.publish(msg)
                rospy.loginfo(f"Published: {source_text}")
                print(f"Sent to ROS: {source_text}")
            else:
                print("ROS publisher not available or node shutdown")
                
        except Exception as e:
            print(f"Error in send_text: {e}")

    def increase_number(self):
        """Increase counter and optionally publish to ROS"""
        try:
            self.count += 1
            if hasattr(self, 'lcdNumber'):
                self.lcdNumber.display(self.count)
                
        except Exception as e:
            print(f"Error in increase_number: {e}")

    def decrease_number(self):
        """Decrease counter and optionally publish to ROS"""
        try:
            self.count -= 1
            if hasattr(self, 'lcdNumber'):
                self.lcdNumber.display(self.count)
                
        except Exception as e:
            print(f"Error in decrease_number: {e}")

    def close_app(self):
        """Close the application properly"""
        try:
            if not rospy.is_shutdown():
                rospy.signal_shutdown("GUI closing")
                print("ROS node shutdown")
        except Exception as e:
            print(f"Error during shutdown: {e}")
        self.close()

    def closeEvent(self, event):
        """Handle window close event"""
        try:
            if not rospy.is_shutdown():
                rospy.signal_shutdown("GUI closing")
                print("ROS node shutdown")
        except Exception as e:
            print(f"Error during shutdown: {e}")
        event.accept()

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