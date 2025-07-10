#!/usr/bin/env python3

"""
    - Author: Archie
    - Date: 2025-04-10
    - Description: Create fake reference points for testing

    - NOTE:
"""

import matplotlib.pyplot as plt
import random
import numpy as np
import json
import os
from datetime import datetime

# Dữ liệu điểm ban đầu
# original_points = [
#     {"id": 1, "x": -6.748659431231162, "y": -2.2449795173903904},
#     {"id": 2, "x": 2.949223849576202, "y": -2.888093982990618},
#     {"id": 3, "x": 6.442740156356269, "y": -2.7613627865114623},
#     {"id": 4, "x": 6.692198962275153, "y": 3.206363554290005},
#     {"id": 5, "x": 6.78127562886237, "y": 9.197896216516842},
#     {"id": 6, "x": 7.1861808598446215, "y": 21.173954848857974},
#     {"id": 7, "x": 3.9921490316500314, "y": 33.21109891689579},
#     {"id": 8, "x": -4.142163896480668, "y": 33.496567675995294},
#     {"id": 9, "x": -5.760432306596919, "y": 27.564419407286913}
# ]

class reflectorMap():
    def __init__(self, _id = 0, _x = 0., _y = 0.):
        self.id = _id
        self.x = _x
        self.y = _y

# Hàm tạo điểm ngẫu nhiên phân bố đều trong toàn bộ phạm vi
def generate_uniform_random_points(original_points, num_points=100, max_value=150):
    random_points = []
    for point in original_points:
        random_points.append({"id": point.id, "x": point.x, "y": point.y})
    
    for i in range(len(original_points), num_points + len(original_points)):
        # Tạo điểm ngẫu nhiên đều trong khoảng [-max_value, max_value]
        new_x = random.uniform(-max_value, max_value)
        new_y = random.uniform(-max_value, max_value)
        
        random_points.append({"id": i + 1, "x": new_x, "y": new_y})
    
    return random_points

def add_reflectorMap(ls_ref, dir_savemap):
    # -- lấy id gương lớn nhất
    id_refAdd = 0
    ls_new = []
    # for refMap in self.lsReflector_inMap:
    #     if refMap.id > id_refAdd:
    #         id_refAdd = refMap.id
    # -- 
    # -- thêm gương vào list cũ
    for ref in ls_ref:
        id_refAdd += 1
        ls_new.append(reflectorMap(id_refAdd, ref["x"], ref["y"]))

    # -- ghi file
    now = datetime.now().strftime("%Y-%m-%d %H:%M")
    json_out = {"name": 'map_reflector', "time_update": str(now), "info": []}

    for info_ref in ls_new:
        json_infoRef = {"id": info_ref.id, "x": info_ref.x, "y": info_ref.y}
        json_out["info"].append(json_infoRef)

    temp_file_path = dir_savemap + '.tmp' 
    try:
        with open(temp_file_path, 'w', encoding='utf-8') as temp_file:
            json.dump(json_out, temp_file, ensure_ascii=False, indent=4)
            temp_file.flush()
            os.fsync(temp_file.fileno())
        os.rename(temp_file_path, dir_savemap)

    except Exception as e:
        print("Write json file backup error", e)

# -- load data map
dir_mapReflector = '/home/stivietnam/catkin_ws/src/navigation_reflector/data_new/data_v2/fake_data/map_reflector_sti_150_150.json'
lsReflector_inMap = []

try:
    with open(dir_mapReflector, 'r') as file:
        data = json.load(file)
        for ref in data["info"]:
            lsReflector_inMap.append(reflectorMap(ref["id"], ref["x"], ref["y"]))

except Exception as e:
    print("Read json file fail: ", e)

have_map_old = False
if len(lsReflector_inMap) > 2:
    have_map_old = True
else:
    print("No map old")

NUM_POINTS = 50                            # số lượng điểm ngẫu nhiên cần tạo
MAX_VALUE = 150                                    # vị trí tối đa mà gương có thể chạm tới ( giống nhà máy)

dir_savemap = '/home/stivietnam/catkin_ws/src/navigation_reflector/data_new/data_v2/fake_data/map_reflector_sti_200_150.json'

if have_map_old: 
    # Tạo dữ liệu điểm ngẫu nhiên
    random_points = generate_uniform_random_points(lsReflector_inMap, num_points=NUM_POINTS, max_value=MAX_VALUE)

    # Lưu dữ liệu vào file JSON
    add_reflectorMap(random_points, dir_savemap)

    # -- vẽ biểu đồ
    # Chuẩn bị dữ liệu để vẽ
    original_x = [point.x for point in lsReflector_inMap]
    original_y = [point.y for point in lsReflector_inMap]
    random_x = [point["x"] for point in random_points[len(lsReflector_inMap):]]
    random_y = [point["y"] for point in random_points[len(lsReflector_inMap):]]

    # Vẽ biểu đồ
    plt.figure(figsize=(10, 10))

    # Vẽ các điểm ngẫu nhiên (màu xanh)
    plt.scatter(random_x, random_y, color='blue', alpha=0.6, label=f'Điểm ngẫu nhiên ({len(random_x)} điểm)')

    # Vẽ các điểm gốc (màu đỏ, kích thước lớn hơn)
    plt.scatter(original_x, original_y, color='red', s=100, label='Điểm gốc (9 điểm)')

    # Đánh số ID cho các điểm gốc
    for point in lsReflector_inMap:
        plt.text(point.x, point.y, f"  {point.id}", 
                fontsize=10, ha='left', va='center', color='darkred')

    # Cấu hình đồ thị
    plt.title('Phân Bố Điểm Ngẫu Nhiên Trong Hình Chữ Nhật 150×150', fontsize=14)
    plt.xlabel('Giá trị X', fontsize=12)
    plt.ylabel('Giá trị Y', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.legend()
    plt.xlim(-160, 160)
    plt.ylim(-160, 160)

    # Hiển thị đồ thị
    plt.tight_layout()
    plt.show()