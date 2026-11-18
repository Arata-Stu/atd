# atd_joy

Jetson の Linux evdev を読む C++ component。DualSense の物理入力を正規化し、Web アプリへ生入力・接続状態を提供する。`kart_joy` の入力専用部分を移植した。車両の操作割り当て、走行モード、ESC・サーボ指令は持たない。ブラウザ PC の Gamepad API は使わない。

package / component は `atd_joy` / `atd_joy::JoyNode`、既定 node は `/atd_joy_node`、単体 executable は `atd_joy_node`。低容量の入力データを扱う component と standalone executable を両方提供する。別の旧 GUI/CUI サーバーは同梱しない。

## 起動とプロファイル

```bash
./scripts/build.sh --packages-up-to atd_web_bridge
# Jetson の ROS 環境で。起動自体では車両制御・録画を開始しない。
./scripts/bringup.sh input
```

初回の profile は `/data/results/runs/input/dualsense.yaml`。存在しなければ入力出力を無効にして待機する。Web の「入力」で設定開始、配置の保存、再読込、全入力解放後の入力出力再開を個別に行う。通常起動では保存済み profile と機器が有効なら入力取得が成立し、PC や Web UI の接続を必要としない。設定作業では明示的に設定モードへ入り、設定開始後に自動で入力出力を再開しない。起動時から設定モードにする場合は start_configuring=true を指定する。Joy の ready は物理入力の利用可否であり、車両の走行許可・停止確認ではない。設定モード中は raw のみ操作値を配信し、正規化 Joy は中立、ready は false。未接続・同期中・必要コード不足も ready は false。

配置 profile は ROS parameter と別の version 1 YAML。`device` は vendor/product/name/unique、`axes` は left_x/left_y/right_x/right_y/l2/r2/dpad_x/dpad_y、`buttons` は cross/circle/triangle/square/l1/r1/l2/r2/create/options/ps/l3/r3/dpad_up/dpad_down/dpad_left/dpad_right。各 axis は evdev code、bipolar/unipolar、negative/center/positive、deadzone。各 button は source(button/axis)、code、direction、threshold。code=-1 は無効。DualSense 初期候補は Sony `054c:0ce6`。スティック・十字キーは [-1,1]、トリガーは [0,1]。この物理配置を throttle 等の車両動作に対応付ける実装は含まない。

未接続時は再探索し、複数一致デバイスは任意選択しない。unique で識別する。切断・同期不良時は中立に戻す。設定保存は atomic rename、profile の不正再読込時は以前の有効設定を保持する。

## 入出力

topic は namespace 相対。入力 topic はなく `/dev/input/event*` を読む。ROS 標準 `/rosout`・`/parameter_events` は表から省略する。QoS は volatile / keep-last、raw は best effort depth1、その他は reliable depth1。

| 出力 topic | 型 | 周期・内容 |
| --- | --- | --- |
| `/joy` | sensor_msgs/msg/Joy | 既定100Hz、正規化8軸・17ボタン。設定中は中立 |
| `/joy/raw` | sensor_msgs/msg/Joy | 既定100Hz、ABS codeをindexとする64軸、KEY codeをindexとする768ボタン。軸はkernel min/maxから[-1,1] |
| `/joy/connected` | std_msgs/msg/Bool | 既定100Hz、接続・同期・profileと入力codeの成立 |
| `/joy/ready` | std_msgs/msg/Bool | 既定100Hz、connectedかつ設定モード外。走行許可そのものではない |
| `/joy/status` | std_msgs/msg/String | 1Hzと状態変化時。YAML、device・generation・configuring・profile_valid・利用可能code |
| `/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | 1Hz、状態診断 |

raw の `header.frame_id=atd_joy_raw_<generation>`。機器変更前の raw と新しい status を混ぜない。
`joy/status` の `instance_id` は node 起動ごとの識別子。下流は `generation` と組み合わせ、node の再起動も入力セッションの変更として扱う。

| 提供 service | 型 | 内容 |
| --- | --- | --- |
| `/atd_joy_node/configure` | std_srvs/srv/SetBool | trueで設定モード。falseは機器・profile有効・同期完了・割り当て後の全軸が絶対値0.1以下・全ボタン解放時だけ入力出力再開 |
| `/atd_joy_node/reload_profile` | std_srvs/srv/Trigger | 設定モード内だけ再読込。自動再開しない |

## parameter と launch

全 parameter は [config/joy.yaml](config/joy.yaml)。独自 parameter は起動時固定/read_only。

| parameter | node既定 | 内容 |
| --- | --- | --- |
| profile_path | 空 | 物理配置YAML。空・不正なら待機 |
| input_dir | /dev/input | evdev探索先 |
| start_configuring | false | trueなら設定モードで起動。falseの通常起動はPC/UI不要 |
| publish_rate_hz | 100.0 | 1〜1000Hz |
| scan_period_ms | 1000 | 100ms以上、未接続時の探索間隔 |
| use_sim_time | false | ROS標準時刻 |

`joy.launch.py` の引数は config（share/config/joy.yaml）、profile（空でconfig維持）、namespace（空）、composed（true）。composed の container は joy_container。`input.launch.py` は profile を永続保存先に上書きして bridge と一致させる。

## 検証

`test/test_mapping.cpp` は ROS 非依存の正規化・中立・境界処理、`test/test_profile.py` は profile 保存・検出を検証する。`colcon test --packages-select atd_joy` で登録テストを実行する。ホストでの単体検証と、ROS build/install・Jetson Bluetooth/evdev・実車検証は別であり、後者は未実施。
