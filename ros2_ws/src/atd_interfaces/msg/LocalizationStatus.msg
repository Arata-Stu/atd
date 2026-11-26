# state == "tracking" のときだけ pose, speed_mps, yaw_rate_rps を使用できる。
# /localization/status での tracking の stamp は照合済みステレオ観測時刻、それ以外は状態通知の ROS 時刻。
# /orbslam3/frame_status では全状態でステレオ観測時刻。画像のない状態通知は含まない。
std_msgs/Header header
string map_id
string map_sha256
string hdmap_sha256
string session_id
uint32 epoch
# waiting_hint, localizing, tracking, lost, stale
string state
string reason
# map 座標系における後輪軸中心 base_link の姿勢。
geometry_msgs/Pose pose
# 同じ時刻の VO オドメトリが報告する車体 x 方向速度と z 軸角速度。
float64 speed_mps
float64 yaw_rate_rps
