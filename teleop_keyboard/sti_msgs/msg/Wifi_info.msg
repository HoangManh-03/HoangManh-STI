bool status_wifi
bool status_autoconnect
int16 ap_strength
int16 ap_signal
string bssid
sti_msgs/CustomDataWifi[] datawifi
int16 err
int16 process
int32 nb_toggle_wifi
int32 nb_change_macwifi

# bool status_wifi: trạng thái có đang kết nối không
# int32 ap_strength: Cường độ của Wifi hiện tại  
# string ap_signal: Tín hiệu wifi hiện tại ( Tốt - TB - Yếu - Kém )
# string bssid: Địa chỉ mac của Wifi
# string ssid: Tên wifi
# sti_msgs/CustomData[] datawifi
# int16 err:  1: lỗi kết nối vs wifi
