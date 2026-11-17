# 正規化した運転要求。速度、実舵角、PWM、最終的な走行許可ではない。
# header.stamp は元入力の時刻を保持する。再配信で更新しない。
std_msgs/Header header
string source
# 入力と割り当ての利用可否。下流は別途、鮮度・走行許可を判断する。
bool valid
# [-1, 1]、正は左操舵。
float32 steering
# [-1, 1]、正は前進、負は後退の要求。ブレーキ値ではない。
float32 throttle
# 設定した enable ボタンの保持状態。最終的な走行許可ではない。
bool enable_requested
