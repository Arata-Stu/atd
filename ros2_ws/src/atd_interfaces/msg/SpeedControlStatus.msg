# planner 元観測と同じ地図・ライン・セッションに対応する速度制御の観測値。
std_msgs/Header header
string map_id
string map_sha256
string hdmap_sha256
string session_id
uint32 epoch
string line
float64 validity_s
bool valid
string reason
string calibration_run_id
string proposal_sha256
float64 kp
float64 ki
float64 kd
# 速度 [m/s] は has_speeds、計算した CH2 raw と P/I/D 寄与は valid 時に有効。
bool has_speeds
float64 target_speed_mps
float64 estimated_speed_mps
float64 speed_error_mps
float64 feedforward_raw
float64 p_raw
float64 i_raw
float64 d_raw
float64 calculated_raw
bool saturated
# has_propo 時は、速度観測の時刻以前に受信した有効なプロポ手動 CH2 raw。
bool has_propo
uint16 propo_raw
float64 propo_age_s
string propo_reason
# has_joy 時は /teleop/command に観測された正規化スロットル [-1, 1]。負は後退要求、正は前進要求。
bool has_joy
float64 joy_throttle
float64 joy_age_s
bool joy_enable_requested
string joy_reason
# Pico が示す経路 (propo, host, standby, fault)、または unknown。
# host_joy は HOST 経路かつ有効な Joy 入力を観測。基板での採用・ESC 出力は別途確認する。
string manual_source
