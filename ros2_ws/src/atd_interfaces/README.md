# atd_interfaces

`DriveCommand` は正規化した操舵・駆動要求。`steering` は [-1,1]（正は左）、`throttle` は [-1,1]（正は前進要求、負は後退要求）。速度・実舵角・ブレーキ・PWM ではない。`valid` は入力の利用可否、`enable_requested` は任意の enable ボタンの保持状態であり、いずれも最終的な走行許可ではない。

`header.stamp` は元入力の時刻、`source` は入力元。周期配信時にも入力時刻を更新しない。下流は自身の受信期限と入力時刻・許可状態を検査する。

`OperatorEvent` は `RECORD_START` / `RECORD_STOP` の一回の要求。記録の開始・終了成功を表す通知ではない。

`JoyDriveStatus` は手動走行 node が受けた CDC0 の基板報告と、その node の操作状態を保持する。`has_report` のあるフレームには CH3 の受信値、サーボ・ESC の出力パルス幅 [µs]、Pico の経路・異常・連番が入る。`joy_active` は HOST 経路を確立した DualSense 手動走行の状態。`header.stamp` は直近の基板報告のホスト受信時刻である。

`PicoTelemetry` は Rev6 の監視用 USB CDC1 で受信した全16ch raw（0〜2047）と基板状態。UI は既定で6chを表示し、10／16chへ切り替えられる。PWM相当のµs表示はrawからの参考換算で、message内の値や実出力とは区別する。`header.stamp` は新しいフレームを受理したホスト受信時刻、`instance_id` は接続・基板再起動の世代、`serial` はUSB固有ID。`rc_valid` は受信時点の判定なので、下流でも元stamp・S.BUSの進行・flagsと期限を検査する。配信時刻の更新で古い入力を延命しない。制御指令や最終走行許可を表す型ではない。

`LocalizationStatus` は地図 ID、cuVSLAM 地図と HDMap の SHA256、実行セッション、追跡状態と世代を一緒に配信する。`state == "tracking"` のとき、`header.stamp` は車体姿勢・前後速度 [m/s]・ヨーレート [rad/s] の観測時刻になる。地図の照合や追跡を待っている間は位置・速度を使用しない。軌跡を描く場合はセッション・世代が変わったところで区切る。[自己位置推定の操作](../../../docs/bringup.md#プロポ手動走行中に自己位置を確認する)

メッセージ生成専用 package は `ament_cmake` / `rosidl_generate_interfaces` を明示的に使う。Docker 内で `atd-build --packages-up-to atd_teleop` により生成・依存 package を build する。

`PlanningStatus` は自己位置と同じ時刻の車体姿勢、選択したライン、前方の参照軌道、横ずれ（左正）、目標速度・推定速度を配信する。地図・HDMap の SHA256、追跡セッション・世代の一致を確認し、`valid` と元の観測時刻からの有効期限 `validity_s` を併せて扱う。[planner の仕様](../atd_planning/README.md)

`ControlStatus` は追従計算の観測結果を同じ地図・セッション・元時刻に対応付ける。Pure Pursuit は後輪軸からの先読み距離、目標座標、左正の曲率を配信する。実測ホイールベースがあるサンプルで `has_steering` が立ち、等価前輪舵角 [rad] を参照できる。[controller の仕様](../atd_control/README.md)

`SpeedControlStatus` は同じ planner 元時刻の目標・推定速度、速度 FF＋PID の計算 CH2、プロポの実 CH2、DualSense の正規化スロットルを保持する。`valid` は計算候補、`has_speeds` は速度観測、`has_propo` と `has_joy` は各入力の有効性を表す。`manual_source` には Pico の経路と Joy の観測状態を格納する。`SetSpeedTable` は校正表の選択・保存に使用する。[速度制御の仕様](../atd_control/README.md#速度制御のプレビュー)
