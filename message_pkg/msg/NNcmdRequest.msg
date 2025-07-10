int32 process # tương tự lagv
int32 target_id # id target
int32 target_dir # hướng tại điểm cuối : lưu ý agv chỉ ứer dụng 4 hướng 1 2 3 4 tương đương (0, 90, 180, 270)
int32 moving_dir # trafic cập nhật hướng hiẹn tại của AGV
int32 tag # tương tự như tag trên lagv
int64 offset # offset yêu cầu khi tới điểm đích, cần di chuyển lùi vào lấy hàng hay tiến vào lấy hàng (giá trị -2(lùi lấy hangg) và 2(tiến lấy hàng))
uint64[] list_id # list id điểm cần di chuyển
uint64[] list_code # list thẻ rfid cần di chuyển
uint64[] list_dir # list hướng cần di chuyển
uint64[] list_speed # list tốc độ cần di chuyển
uint64[] list_encoder # ko quan tâm
int32 before_mission # nhiệm vụ trc tt lagv
int32 after_mission # nhiệm vụ sau tt lagv
int32 id_command # tt lagv
string command # tt lagv