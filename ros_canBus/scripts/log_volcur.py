import rospy
from std_msgs.msg import Float32, Bool, Int16
from openpyxl import Workbook
from datetime import datetime
from sti_msgs.msg import POWER_info  # Cập nhật đúng kiểu message của bạn

# Tạo file Excel
workbook = Workbook()
sheet = workbook.active
sheet.title = "POWER Logs"
sheet.append(["Thời gian", "Điện áp (V)", "Dòng sạc (A)"])

# Biến để theo dõi thời gian lần cuối ghi log
last_logged_time = None

# Hàm callback xử lý dữ liệu
def power_info_callback(data):
    global last_logged_time
    try:
        # Lấy thời gian hiện tại
        time_now = datetime.now()
        
        # Kiểm tra nếu đã qua 1 giây kể từ lần ghi trước
        if last_logged_time is None or (time_now - last_logged_time).total_seconds() >= 1.:
            # Cập nhật thời gian lần cuối
            last_logged_time = time_now
            
            # Dữ liệu cần ghi
            voltages = data.voltages
            charge_current = data.charge_current/10.
            
            # Ghi dữ liệu vào Excel
            sheet.append([time_now.strftime("%Y-%m-%d %H:%M:%S"), voltages, charge_current])
            workbook.save("log.xlsx")
            rospy.loginfo(f"Đã ghi log: {time_now}, Voltages: {voltages}, Current: {charge_current}")
    except Exception as e:
        rospy.logerr(f"Lỗi khi ghi log: {e}")

# ROS Node
def power_logger():
    rospy.init_node('power_logger', anonymous=True)
    rospy.Subscriber('/POWER_info', POWER_info, power_info_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        power_logger()
    except rospy.ROSInterruptException:
        workbook.save('log.xlsx')
        rospy.loginfo("Đã lưu và đóng file Excel trước khi thoát.")
