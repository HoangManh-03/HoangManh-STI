int32 enable # mode di chuyển : 1 - bằng tay tiến theo line từ, 2 - bằng tay lùi theo line từ, 3 - tự động, 4 : di chuyển vào điểm thao tác
int32 target_id # id yêu cầu từ traffic
int32 target_dir # hướng điểm đích
int32 moving_dir # hướng agv 
uint64[] list_id # tt id từ trafic
uint64[] list_code # tt code từ trafic
uint64[] list_dir # tt trafic
uint64[] list_speed # tt trafic
uint64[] list_encoder # tt traffic
int32 mission # tt offset traffic 
int32 speed_move_by_hand # tốc độ khi bám line bằng tay