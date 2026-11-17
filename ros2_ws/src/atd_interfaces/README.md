# atd_interfaces

`DriveCommand` は正規化した操舵・駆動要求。`steering` は [-1,1]（正は左）、`throttle` は [-1,1]（正は前進要求、負は後退要求）。速度・実舵角・ブレーキ・PWM ではない。`valid` は入力の利用可否、`enable_requested` は任意の enable ボタンの保持状態であり、いずれも最終的な走行許可ではない。

`header.stamp` は元入力の時刻、`source` は入力元。周期配信時にも入力時刻を更新しない。下流は自身の受信期限と入力時刻・許可状態を検査する。

`OperatorEvent` は `RECORD_START` / `RECORD_STOP` の一回の要求。記録の開始・終了成功を表す通知ではない。

メッセージ生成専用 package は `ament_cmake` / `rosidl_generate_interfaces` を明示的に使う。Docker 内で `/workspaces/atd/scripts/build.sh --packages-up-to atd_teleop` により生成・依存 package を build する。
