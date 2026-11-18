# atd_teleop

Jetson の `atd_joy` が正規化した入力に、操舵・駆動要求と操作ボタンの役割を割り当てる C++20 component / standalone node。package は `atd_teleop`、component は `atd_teleop::TeleopNode`、node / executable は `atd_teleop_node`。物理配置・校正は `atd_joy`、この package は役割設定を担当する。基板への通信、PWM 変換、走行許可の gate、rosbag の実際の記録は実装しない。

## 型と入出力

`atd_interfaces/msg/DriveCommand` の `steering` は [-1,1]（正は左）、`throttle` は [-1,1]（正は前進要求、負は後退要求）。実舵角・速度・ブレーキ値ではない。ESC の後退手順や制動はこの値から推測しない。`valid` は入力と割り当ての利用可否で、`enable_requested` は任意の enable ボタンの保持状態。最終的な走行許可は下流で判断する。

| 方向 | topic | 型 | 内容 |
| --- | --- | --- | --- |
| 購読 | `/joy` | sensor_msgs/msg/Joy | 正規化8軸・17ボタン。順序は atd_joy の論理入力順 |
| 購読 | `/joy/ready` | std_msgs/msg/Bool | 入力層の利用可否。鮮度も検査 |
| 購読 | `/joy/status` | std_msgs/msg/String | YAML の ready / connected / configuring / profile_valid / generation / instance_id |
| 配信 | `/teleop/command` | atd_interfaces/msg/DriveCommand | 既定100Hz、source=joy。利用不可時は valid=false・中立・enable=false |
| 配信 | `/teleop/events` | atd_interfaces/msg/OperatorEvent | RECORD_START / RECORD_STOP の新規押下だけ。一回の要求であり、記録成功通知ではない |
| 配信 | `/teleop/status` | std_msgs/msg/String | 1Hzと状態変化時。設定状態、理由、active profile、入力セッション、要求値 |

topic は namespace 相対。QoS は ROS 側で reliable / volatile / keep-last depth1 を指定する。`header.stamp` は元の Joy 時刻を保持し、周期再配信で更新しない。同一・過去の入力 stamp は鮮度を更新しない。watchdog は ROS 時刻ではなく steady clock を使う。下流の指令期限監視や基板 firmware の通信監視は別途必要。

入力配列は `atd_joy` の正規化契約を検査する。8軸のうち L2 / R2（index 4 / 5）は [0,1]、その他は [-1,1]、17ボタンは0または1のみとする。要素数、有限性、範囲が不正な入力は要求を無効化する。駆動要求の符号は役割設定の軸差分・scale によって作る。

## 設定

既定の役割 profile は `/data/results/runs/input/teleop.yaml`。存在しない・不正・両軸未割り当ての場合は中立要求で待機する。初期候補 [config/roles.yaml](config/roles.yaml) は全て未割り当てで、操作配置を固定しない。

```yaml
version: 1
axes:
  steering: {axis: left_x, subtract_axis: '', scale: -1.0}
  throttle: {axis: r2, subtract_axis: l2, scale: 0.3}
buttons:
  enable: r1
  record_start: cross
  record_stop: circle
```

上記は設定形式の例であり、採用済みのボタン配置ではない。軸は `clamp((axis - subtract_axis) * scale, -1, 1)`。空文字の入力は0として扱い、scale は [-1,1]。軸名は left_x/left_y/right_x/right_y/l2/r2/dpad_x/dpad_y。ボタン名は atd_joy の17論理ボタン名。ボタンの空文字は未割り当て。同じボタンに複数の役割は割り当てられない。

運転要求には `steering` / `throttle` の両方に入力軸が必要。記録専用なら開始・停止ボタンを割り当てて再開できる。この場合はイベントを配信し、運転要求は中立・無効を維持する。`/teleop/status` の `ready` は運転要求、`events_ready` は操作イベントの利用可否を表す。Python の `atd_teleop_tools.profile` と C++ の読み込みが schema を検査する。保存は検証後の atomic rename。

| service | 型 | 内容 |
| --- | --- | --- |
| `/atd_teleop_node/configure` | std_srvs/srv/SetBool | true で要求を中立・無効にし設定モードへ。false は有効な役割設定、新鮮な Joy / ready / status、全軸が±0.1以内、全ボタン解放が必要 |
| `/atd_teleop_node/reload_profile` | std_srvs/srv/Trigger | 設定モード内だけ再読込。不正なら既存の有効設定を維持し、要求を再開しない |

保存、再読込、入力マッピングの再開は別操作。Notebook の UI は Jetson loopback bridge を SSH 経由で操作し、操舵・throttle の連続指令は送らない。Joy の設定モード、切断、期限切れ、入力セッション変更では要求を無効化し、全入力解放まで再び有効にしない。起動・復帰・設定変更時の保持ボタンを新規押下として扱わない。これらは入力利用の条件であり、実車の走行再開・停止を検証したものではない。

## 起動と検証

Jetson のホストから、起動済み Docker 内の自作 package を build する。

```bash
./scripts/build.sh --packages-up-to atd_web_bridge
# ホスト側の入口。Joy / teleop / 監視・設定 bridge を起動。
./scripts/bringup.sh input
```

単体では `ros2 launch atd_teleop teleop.launch.py profile:=/absolute/path/teleop.yaml`。launch の config / profile / namespace / composed は atd_joy と同形式。profile は config の値を上書きし、composed=false で standalone を起動する。

parameter は起動時固定。`input_timeout_s=0.25`（.01〜5秒）、`status_timeout_s=2.5`（1〜30秒）、`publish_rate_hz=100.0`（1〜1000Hz）、`start_configuring=false`。通常起動では有効な保存設定と全入力解放があれば Notebook / UI 接続なしで入力マッピングを開始できる。

`test/test_teleop.cpp` は ROS 非依存で、マッピング、上限、押下エッジ、途絶、セッション変更、設定再開、異常入力を検証する。`test/test_profile.py` は schema、重複・不正値、永続化を検証する。ROS 環境では `colcon test --packages-select atd_teleop`、ホストでは C++20 compiler と Python/PyYAML で実行できる。

ホスト上の単体検証と、ROS build/install・message生成・DDS/service結合・Jetson Bluetooth・基板I/O・実走行は別である。後者は対象環境での検証が必要。
