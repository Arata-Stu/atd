# atd_interfaces

`DriveCommand` は正規化した操舵・駆動要求。`steering` は [-1,1]（正は左）、`throttle` は [-1,1]（正は前進要求、負は後退要求）。速度・実舵角・ブレーキ・PWM ではない。`valid` は入力の利用可否、`enable_requested` は任意の enable ボタンの保持状態であり、いずれも最終的な走行許可ではない。

`header.stamp` は元入力の時刻、`source` は入力元。周期配信時にも入力時刻を更新しない。下流は自身の受信期限と入力時刻・許可状態を検査する。

`OperatorEvent` は `RECORD_START` / `RECORD_STOP` の一回の要求。記録の開始・終了成功を表す通知ではない。

`PicoTelemetry` は Rev6 の監視用 USB CDC1 で受信した全16ch raw（0〜2047）と基板状態。UI は既定で6chを表示し、10／16chへ切り替えられる。PWM相当のµs表示はrawからの参考換算で、message内の値や実出力とは区別する。`header.stamp` は新しいフレームを受理したホスト受信時刻、`instance_id` は接続・基板再起動の世代、`serial` はUSB固有ID。`rc_valid` は受信時点の判定なので、下流でも元stamp・S.BUSの進行・flagsと期限を検査する。配信時刻の更新で古い入力を延命しない。制御指令や最終走行許可を表す型ではない。

`LocalizationStatus` は地図 ID、cuVSLAM 地図と HDMap の SHA256、実行セッション、追跡状態と世代を一緒に配信する。`state == "tracking"` のとき、`header.stamp` は車体姿勢・前後速度 [m/s]・ヨーレート [rad/s] の観測時刻になる。地図の照合や追跡を待っている間は位置・速度を使用しない。軌跡を描く場合はセッション・世代が変わったところで区切る。[自己位置推定の操作](../../../docs/bringup.md#プロポ手動走行中に自己位置を確認する)

メッセージ生成専用 package は `ament_cmake` / `rosidl_generate_interfaces` を明示的に使う。Docker 内で `atd-build --packages-up-to atd_teleop` により生成・依存 package を build する。
