# import numpy as np
# from scipy.optimize import minimize

# # Hàm mục tiêu cần tối ưu (ví dụ: tối thiểu hóa khoảng cách từ điểm (1,2))
# def objective(x):
#     return (x[0] - 1)**2 + (x[1] - 2)**2

# # Ràng buộc 1: x[0] + x[1] >= 2
# def constraint1(x):
#     return x[0] + x[1] - 2

# # Ràng buộc 2: x[0] >= 0.5
# def constraint2(x):
#     return x[0] - 0.5

# # Định nghĩa danh sách các ràng buộc. 
# # Với 'ineq': hàm constraint phải có giá trị >= 0.
# constraints = [
#     {'type': 'ineq', 'fun': constraint1},
#     {'type': 'ineq', 'fun': constraint2}
# ]

# # Giá trị khởi tạo
# x0 = [0, 0]

# # Gọi hàm minimize với phương pháp 'SLSQP' hỗ trợ ràng buộc
# solution = minimize(objective, x0, method='SLSQP', constraints=constraints)

# print("Kết quả tối ưu:", solution.x)
# print("Giá trị hàm mục tiêu:", solution.fun)


import numpy as np
import matplotlib.pyplot as plt

# Định nghĩa một cụm điểm trong không gian 2 chiều
points = np.array([
    [1, 2],
    [2, 3],
    [3, 1],
    [4, 4],
    [2, 5],
    [3, 3]
])

# Tính trung bình cộng (centroid) của các điểm
centroid = np.mean(points, axis=0)

# In ra tọa độ của centroid
print("Centroid:", centroid)

# Vẽ các điểm
plt.scatter(points[:, 0], points[:, 1], color='blue', label='Các điểm trong cụm')

# Vẽ centroid với ký hiệu khác để phân biệt (dấu X màu đỏ)
plt.scatter(centroid[0], centroid[1], color='red', marker='X', s=200, label='Centroid')

# Hiển thị tọa độ của centroid trên đồ thị
plt.annotate(f'({centroid[0]:.2f}, {centroid[1]:.2f})', 
             xy=(centroid[0], centroid[1]), 
             xytext=(centroid[0]+0.2, centroid[1]+0.2),
             arrowprops=dict(facecolor='black', shrink=0.05))

# Thêm tiêu đề và nhãn trục
plt.title('Trực quan cụm điểm và trung bình cộng (Centroid)')
plt.xlabel('Trục X')
plt.ylabel('Trục Y')
plt.legend()
plt.grid(True)
plt.show()
