# 観測用。header と識別子は参照した PlanningStatus に対応する。
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
string controller
# 有効時の後輪軸姿勢と同一時刻の追従目標。map 座標。
geometry_msgs/Pose pose
geometry_msgs/Point target
float64 curvature_per_m
# 自転車モデルの等価前輪舵角 [rad]。左正。両値とも has_steering 時に有効。
bool has_steering
float64 steering_rad
float64 wheelbase_m
# Pure Pursuit の診断。距離はいずれも後輪軸から目標への直線距離 [m]。
float64 requested_lookahead_m
float64 actual_lookahead_m
bool preview_limited
float64 min_lookahead_m
float64 lookahead_time_s
float64 max_lookahead_m
