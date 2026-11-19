# 有効な場合の stamp は元の後輪軸姿勢の観測時刻、frame_id は map。
std_msgs/Header header
string map_id
string map_sha256
string hdmap_sha256
string session_id
uint32 epoch
# raceline, centerline, custom
string line
float64 validity_s
bool valid
string reason
# 以下は valid == true のときだけ使う。進捗はライン始点からの弧長（閉路は周回内）。
geometry_msgs/Pose pose
float64 progress_m
# ライン進行方向から見て車体が左側で正。
float64 lateral_error_m
float64 target_speed_mps
float64 estimated_speed_mps
# 先頭は後輪軸位置の投影点、後続はライン上の前方軌道。点と速度は同じ長さ。
geometry_msgs/Point[] trajectory
float64[] trajectory_speeds_mps
