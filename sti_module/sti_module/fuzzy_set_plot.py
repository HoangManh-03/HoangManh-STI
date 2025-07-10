import numpy as np
import matplotlib.pyplot as plt
from lib_pidController import *

def plot_pid_parameters():
    # Tạo đối tượng LineFollowerFUZZYPID
    controller = LineFollowerFUZZYPID(pub_vel=None)

    # Cập nhật Fuzzy Logic
    error = 5.0
    velocity = 0.3

    controller.pid.input['error'] = error
    controller.pid.input['velocity'] = velocity
    controller.pid.compute()

    # Lấy giá trị Kp, Ki, Kd từ Fuzzy
    kp_value = controller.pid.output['kp']
    ki_value = 0.0
    kd_value = controller.pid.output['kd']

    print("Kp = %s, Kd = %s" %(kp_value, kd_value))

    # # Lấy dữ liệu PID từ danh sách có sẵn
    # velocities = [item[0] for item in controller.list_pid_vel]
    # kp_values = [item[1] for item in controller.list_pid_vel]
    # ki_values = [item[2] for item in controller.list_pid_vel]
    # kd_values = [item[3] for item in controller.list_pid_vel]

    # # Vẽ đồ thị
    # plt.figure(figsize=(10, 6))
    # plt.plot(velocities, kp_values, label="Kp", marker="o")
    # plt.plot(velocities, ki_values, label="Ki", marker="s")
    # plt.plot(velocities, kd_values, label="Kd", marker="^")

    # # Cấu hình biểu đồ
    # plt.xlabel("Velocity (m/s)")
    # plt.ylabel("PID Parameters")
    # plt.title("PID Parameters vs Velocity")
    # plt.legend()
    # plt.grid()
    # plt.show()

# Gọi hàm vẽ đồ thị
plot_pid_parameters()
