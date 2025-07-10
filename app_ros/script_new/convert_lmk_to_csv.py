import pandas as pd

# Đọc tệp .lmk
# Giả sử định dạng của tệp .lmk là bảng, bạn có thể cần điều chỉnh phần này
with open('map12.lmk', 'r') as file:
    lines = file.readlines()

# Xử lý dữ liệu, ví dụ, tách các dòng và lưu vào danh sách
data = [line.strip().split() for line in lines]  # Điều chỉnh theo định dạng tệp của bạn

# Tạo DataFrame
df = pd.DataFrame(data)

# Ghi DataFrame vào tệp CSV
df.to_csv('outputmap12.csv', index=False, header=False)  # Thêm header=True nếu bạn muốn thêm tiêu đề

