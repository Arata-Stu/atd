# atd_web_bridge

Jetson 内だけで DDS に参加し、位置・速度・Jetson 接続 Joy のキャッシュを loopback HTTP へ提供する。Notebook の共通 Web backend が SSH（鍵・known_hosts厳格）で Jetson の標準 Python/urllib を起動してアクセスする。Notebook は車載 DDS に参加せず、Jetson でブラウザを起動しない。HTTP は Python 標準ライブラリ、ROS service 仲介は rclpy。これは低頻度の監視・設定用であり、画像の大容量経路やリアルタイム制御を担わないため Python node を用いる。

node は `/atd_web_bridge`、executable は `web_bridge`。CMake / ament_cmake_auto / ament_cmake_python で install する。

```bash
./scripts/python.sh ros build --packages-up-to atd_web_bridge
# Jetson: ROS と選択したworkspaceの install/setup.bash を source 後
./scripts/input.sh
# Joyが別途起動済みなら bridge だけ
./scripts/input.sh --bridge-only
```

`scripts/input.sh --dry-run` は Notebook でも予定する Jetson コマンドを確認できる。通常起動は Jetson 限定で、実行は既存の scripts/python.sh へ委譲する。

両 launch は config（share/config/bridge.yaml）を受ける。input.launch.py は profile（`/data/results/runs/input/dualsense.yaml`）も受け、Joy と bridge に同じパスを渡す。車両制御・bag記録は開始しない。Docker の host network を使い、HTTP もDDSも Jetson 内 loopback に限定する。

## 入力と通信

すべて購読、QoS は best effort / volatile / keep-last depth5（sensor_data）。reliable publisher も購読できる。parameter で topic を実際の bringup に合わせる。既定名は配信済みを意味しない。欠落・古いデータは null / stale として返し、模擬データで補わない。ROS 標準 `/rosout`・`/parameter_events` は省略する。

| 既定 topic | 型 | キャッシュ |
| --- | --- | --- |
| /localization/odometry | nav_msgs/msg/Odometry | poseとvelocity。速度はchild_frame_id |
| /localization/pose | geometry_msgs/msg/PoseStamped | pose。上記と併用時は最後の受信を採用 |
| /vehicle/velocity | geometry_msgs/msg/TwistStamped | velocity |
| /joy | sensor_msgs/msg/Joy | normalized |
| /joy/raw | sensor_msgs/msg/Joy | raw |
| /joy/status | std_msgs/msg/String | YAMLの接続・設定・機器世代 |

HTTP は `127.0.0.1:8790` のみ。`GET /live/status`、`POST /live/parameters`（nodeだけで一覧取得、values追加でatomic変更）、`GET /input/status`、`GET/PUT /input/profile`、`POST /input/reload`、`POST /input/configure`。各要求は SSH adapter の nonce を header `X-ATD-Request` に持ち、応答に同じ nonce と schema/instance_id/hostname/node を返す。HTTP応答形式は `{identity,request_id,data}`、失敗は error。再試行による操作の二重送信はしない。

pose は x/y/z/yaw/frame/stamp/age_s/stale、velocity は linear_mps/angular_rps/linear/frame/stamp/age_s/stale。input は bridge接続と device_connected/ready を分離し、status/raw の世代不一致・古さを成功と扱わない。Joy ready は車両の安全・停止確認ではない。

## 設定操作

ROS node 名は prefix で閲覧範囲を制限し、書込対象は `parameter_allowlist_json` の node完全名→parameter名配列で明示する。例 `{"/atd_example":["gain"]}` は説明用で、そのnodeを導入しない。既定 `{}` は書込許可なし。descriptor の read_only/type/range/step を検査し、対象 node の SetParametersAtomically service に一度要求する。GetParametersで反映値を再取得し、requested/values/applied を返す。永続YAMLを勝手に書き換えない。timeoutは結果未確認として返す。

Joy profile は物理入力配置専用。新鮮な設定モード状態とprofile_path一致、revision一致を要求して atomic 保存する。保存・再読込・入力出力再開は別操作。再開は Joy node 自身が入力解放を検査する。未接続 service・不明な状態は成功にしない。車両の走行モード・ステアリング・throttleコマンドを受け付ける endpoint は持たない。

消費する service は対象 node の `list_parameters` / `describe_parameters` / `get_parameters` / `set_parameters_atomically`（rcl_interfaces/srv）、および atd_joy の configure（std_srvs/srv/SetBool）/reload_profile（std_srvs/srv/Trigger）。bridge 自身の提供ROS serviceは標準parameter serviceだけ。

## parameter

全設定は [config/bridge.yaml](config/bridge.yaml)、起動時固定/read_only。QoSはコードのROS側指定であり、DDS XMLにはtopic QoSを設定しない。

| parameter | 既定 | 意味 |
| --- | --- | --- |
| use_sim_time | false | ROS標準時刻 |
| port | 8790 | loopback HTTP、1024〜65535 |
| freshness_s | 2.5 | 0.1〜30秒、受信monotonic時刻で鮮度判定 |
| odometry_topic | /localization/odometry | Odometry入力 |
| pose_topic | /localization/pose | PoseStamped入力 |
| velocity_topic | /vehicle/velocity | TwistStamped入力 |
| joy_topic | /joy | 正規化入力 |
| joy_raw_topic | /joy/raw | 生入力 |
| joy_status_topic | /joy/status | 機器状態 |
| joy_node | /atd_joy_node | Joy service対象node |
| profile_path | /data/results/runs/input/dualsense.yaml | 既存results mount内の永続配置 |
| parameter_node_prefix | /atd_ | 閲覧対象nodeの完全名prefix |
| parameter_allowlist_json | {} | 変更許可nodeとparameter名のJSON object |

## 検証

`colcon test --packages-select atd_web_bridge` はキャッシュ鮮度・世代不一致・profile競合を検証する。Notebook単体でROSをimportせずWebを起動できる。ROS build/install・DDS/service結合・Jetson/実機は未検証であり、単体テスト通過をこれらの検証済みとは扱わない。
