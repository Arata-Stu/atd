# CRC・連番を確認した CDC0 の JPB1,S のホスト受信時刻。
# 同じ報告に対する Joy 解除通知では stamp を維持する。未受信・切断時はゼロ。
std_msgs/Header header
bool has_report
string serial
string session_id
uint32 sequence_number
# CH3 パルス幅。受信機の有効値がない場合は 0。
uint16 rx_mode_us
# Pico が報告する selector（0=RC、1=HOST許可）と active_path（0=待機、1=RC、2=HOST、3=異常）。
uint8 selector
uint8 active_path
uint32 fault_bits
# サーボ・ESC の基板報告値 [µs]。電源異常 (fault_bits bit0) の間はパルスを出力しない。
uint16 servo_us
uint16 esc_us
# ローカル Joy ドライバが有効な入力を送信中で、基板でも HOST 経路を確認した。
bool joy_active
string reason
