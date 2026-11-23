# atd_web_bridge

実車では Jetson 内で DDS に参加し、位置・速度・Jetson 接続 Joy・Pico の USB 受信状態・入力の役割設定を loopback HTTP へ提供する。Notebook の共通 Web backend が SSH（鍵・known_hosts厳格）で Jetson の標準 Python/urllib を起動してアクセスする。HTTP は Python 標準ライブラリ、ROS service 仲介は rclpy。

Notebook の `atd gazebo closed-loop --web` は、Gazebo の ROS ドメインに同じ bridge を起動する。Web UI のライブ画面で Gazebo を選ぶと、Notebook 内の `127.0.0.1:8791` を通じて自己位置・計画と直近15秒の目標速度・推定速度・指令舵角・スロットルを監視できる。パラメータの変更は起動中の `/atd_gazebo_control` に反映する。速度上限は起動時の `--speed` まで、最大先読みは起動時の前方軌道長まで変更できる。速度上限または PID ゲインを変更すると速度 PID の履歴をリセットする。

node は `/atd_web_bridge`、executable は `web_bridge`。CMake / ament_cmake_auto / ament_cmake_python で install する。

```bash
./scripts/build.sh --packages-up-to atd_web_bridge
# Jetson: 起動済み ATD Docker 内の ROS 環境を利用
./scripts/bringup.sh input
# 入力・役割 node が別途起動済みなら bridge だけ
./scripts/bringup.sh bridge
```

`./scripts/bringup.sh` は対話端末で `input` / `bridge` を選択し、`--list` は一覧を表示する。`./scripts/bringup.sh --dry-run input` は Notebook でも予定する Jetson コマンドを確認できる。実行は Jetson の起動済み Docker 内で行い、ROS 環境の選択には `scripts/tools/python.sh` を利用する。[bringup の操作](../../../docs/bringup.md)を参照する。

両 launch は config（share/config/bridge.yaml）を受ける。input.launch.py は profile（`/data/results/runs/input/dualsense.yaml`）、teleop_profile（`/data/results/runs/input/teleop.yaml`）、rc_profile（`/data/results/runs/input/propo.yaml`）を各 node と bridge に渡す。Joy・teleop と Pico 受信・プロポ役割処理を起動し、`teleop:=false` / `propo:=false` で各経路を除外できる。Pico の選択は `pico_serial` / `pico_sensor_path`、各 node の設定は `joy_config` / `teleop_config` / `pico_config` / `rc_config` で指定する。子launchの引数はスコープを分け、bridge の config・profile と混在させない。基板への走行指令やbag記録は自動開始しない。inputは `recording:=true`（既定）で共通managerを起動し、割当済みJoy・プロポのイベントで記録する。`recording_profile:=inputs`（既定）、`mapping` または `calibration`、`recording_config:=絶対パス` を指定できる。Docker の host network を使い、HTTP もDDSも Jetson 内 loopback に限定する。

## 入力と通信

購読の既定 QoS は best effort / volatile / keep-last depth 5（sensor_data）。自動走行状態は停止遷移も受け取るため、配信側に合わせて reliable / volatile / keep-last depth 10 を使う。parameter で topic を実際の bringup に合わせる。既定名は配信済みを意味しない。欠落・古いデータは null / stale として返し、模擬データで補わない。ROS 標準 `/rosout`・`/parameter_events` は省略する。

| 既定 topic | 型 | キャッシュ |
| --- | --- | --- |
| /planning/status | atd_interfaces/msg/PlanningStatus | 選択ライン・前方軌道と元の車体姿勢・横ずれ・速度。有効期限と自己位置の地図・追跡世代を照合して `/live/status` の `planning` に返す |
| /control/status | atd_interfaces/msg/ControlStatus | `/live/status` の `control` に、同じ観測の後輪軸姿勢と目標点、曲率、等価前輪舵角、先読み設定を返す。舵角を計算できない場合は null |
| /atd/gazebo/pursuit_status | atd_interfaces/msg/ControlStatus | Gazebo 接続時の `control`。仮想車両の操舵を計算した画像時刻の後輪軸姿勢と先読み点を map 座標 [m] で返す。待機・失効・停止時は点を無効にする。 |
| /atd/gazebo/vehicle_hold | std_msgs/msg/String (JSON) | 模擬車両が位置を保持しているか、車速 [m/s] とシミュレーション時刻を受け取る。Web での再開は、この車両から新しい停車確認を受信すると操作できる。 |
| /control/speed_status | atd_interfaces/msg/SpeedControlStatus | `speed_control` に目標・推定速度、計算 CH2、プロポ手動 CH2、正規化された DualSense 入力、操作経路と直近 5 秒の履歴を返す |
| /vehicle/auto_drive/status | std_msgs/msg/String (JSON) | 低速自動走行で、CH3・操作元、実際の送信指令と基板報告 PWM、速度、停止理由をライブ画面へ返す。速度の元時刻、USB 送信時刻、基板報告の受信時刻を保つ |
| /localization/odometry | nav_msgs/msg/Odometry | poseとvelocity。速度はchild_frame_id |
| /localization/pose | geometry_msgs/msg/PoseStamped | pose。上記と併用時は最後の受信を採用 |
| /vehicle/velocity | geometry_msgs/msg/TwistStamped | velocity |
| /joy | sensor_msgs/msg/Joy | normalized |
| /joy/raw | sensor_msgs/msg/Joy | raw |
| /joy/status | std_msgs/msg/String | YAMLの接続・設定・機器世代 |
| /teleop/status | std_msgs/msg/String | YAMLの操作出力可否・設定モード・反映済み役割設定 |
| /teleop/command | atd_interfaces/msg/DriveCommand | 元の Joy 時刻を持つ操舵・スロットルの観測値 |
| /vehicle/joy_drive/status | atd_interfaces/msg/JoyDriveStatus | 手動走行 node が CDC0 で受信した CH3・操作経路・サーボ／ESC の PWM と Joy 運転状態 |
| /vehicle/pico/status | std_msgs/msg/String | YAMLのUSB接続・受信状態・元の受信時刻を持つ基板telemetry |
| /rc/status | std_msgs/msg/String | YAMLのプロポ役割・設定モード・反映済み設定・入力世代 |

HTTP は `127.0.0.1:8790` のみ。`GET /live/status`、`POST /live/parameters`（nodeだけで一覧取得、values追加でatomic変更）、`GET /input/status`、`GET/PUT /input/profile`、`POST /input/reload`、`POST /input/configure`。各要求は SSH adapter の nonce を header `X-ATD-Request` に持ち、応答に同じ nonce と schema/instance_id/hostname/node を返す。HTTP応答形式は `{identity,request_id,data}`、失敗は error。再試行による操作の二重送信はしない。

役割設定は `GET /teleop/status`、`GET/PUT /teleop/profile`、`POST /teleop/reload`、`POST /teleop/configure`。Notebook の API では同じパスの先頭に `/api` を付ける。`status` は node の状態に `age_s` / `stale` を追加し、未受信なら null を返す。`profile` は物理配置と独立した `version: 1` YAML で、操舵・スロットルの入力軸／差し引く軸／倍率と、enable 要求・記録開始／終了のボタンを保存する。未割り当ては空文字、初期値は全て未割り当て。入力候補は `axes` / `buttons` 配列として応答する。全ての teleop 変更要求は、直前に取得した応答の `identity.instance_id` を `instance_id` として body に含める。省略・不一致なら変更せず拒否し、接続先や bridge が変わった場合は状態と設定を再取得する。

pose は x/y/z/yaw/frame/stamp/age_s/stale、velocity は linear_mps/angular_rps/linear/frame/stamp/age_s/stale。input は bridge接続と device_connected/ready を分離し、status/raw の世代不一致・古さを成功と扱わない。Joy ready は車両の安全・停止確認ではない。

`GET /input/status` の `drive_monitor` は、割り当て後の DualSense 入力を `joy`、CH3・操作元・出力 PWM を `board` に返す。入力と基板の元時刻を個別に検査し、CDC0 の報告があるときはサーボと ESC のパルス幅を µs で返す。電源異常でパルス出力が無効のときは両方とも null。監視用の CDC1 だけでも、受信できた CH3 の S.BUS raw 値と Pico の操作経路を表示できる。CDC1 には出力パルス幅が含まれず、HOST を操作している機器も識別できない。

プロポ用は `GET /propo/status`、`GET/PUT /propo/profile`、`POST /propo/reload`、`POST /propo/configure`。Notebook API は `/api` を付ける。`status` は `{board, input, ready, rc_valid, input_current, configuring, availability}` を返す。Notebook adapter が付ける最上位 `connected` は bridge への接続、`board.connected` は Pico の USB 接続、`board.streaming` は新鮮な telemetry の受信を表す。未受信は null のままとし、CH を 0 で埋めない。telemetry の元 `stamp` を保持し、receiver の `age_s` に bridge での経過時間を加算する。定期的な状態再配信だけで古い信号を新鮮に扱わず、Pico・役割 node の世代と serial が一致しない入力も利用可能とは表示しない。`rc_valid` は受信時点の RC 有効判定を telemetry の期限内だけ表示する観測値で、イベント出力時の厳密な鮮度判定は RC node が行う。

## 記録状態の観測

Notebook の「記録した走行」画面では、この package の `extract_drive_timeline` が停止済み bag の運転入力と基板報告を読み出す。Web アプリは選択区間を Notebook 内で取得する。Jetson の bridge は走行後の閲覧には不要。[記録と時系列表示](../../../docs/recording.md#手動走行を-notebook-で振り返る)

抽出器は追跡中の `/localization/status` の符号付き速度 [m/s]、および D455 の加速度を元のヘッダー時刻で保存する。加速度 [m/s²] は bag の静的 TF で `base_link` の前方軸へ回転したセンサ位置の比力で、重力とセンサ取付位置の影響を含む。接続する TF が欠ける場合や取付姿勢が一意に定まらない場合は加速度を欠測にする。プロポ CH2 は正常かつ新鮮なプロポ運転中の S.BUS raw 値として扱う。

`GET /recording/status` は `/recording/status`（std_msgs/String）のmanager状態を元stamp・ageとともに返す。`freshness_s` と `availability.status`（missing/fresh/stale）を併記し、status未受信はnull。statusには開始処理・記録中・停止処理等、直近の操作要求と前回の終了結果が含まれる。古い状態や接続断を停止済みと補完せず、開始停止のHTTP操作は受け付けない。Notebook APIは `/api` を先頭に付ける。[記録手順](../../../docs/recording.md)

## 車両配置

両launchで `vehicle_profile`（既定 `/data/results/runs/vehicle/layout.yaml`）を指定でき、bridgeの `vehicle_profile_path` へ渡す。`atd_bringup manual.launch.py` は車体TF nodeにも同じpathを渡す。

`GET /vehicle/status` は `/vehicle/layout/status` の元stampと受信後経過時間から鮮度を返す。`GET/PUT /vehicle/profile` は保存設定・revision・path・frames・CAD参考metadataと、`saved` / `applied` / `pending_restart` / `active_profile` を返す。PUTはbridgeの `instance_id` とrevisionを要求する。TF node未起動でも保存できるが、適用済みにはしない。保存済みとactive profile/pathが一致し、新鮮な `profile_valid=true` / `tf_published=true` 状態を受信した場合だけ適用済みとする。

未計測値はnull、保存はm／rad。カメラ取付TFの有効化には全6値を要求する。保存後も既存のstatic TFは変えず、node再起動で反映する。CADはreadonly参考情報で、profileやTFへ自動転記しない。[車両配置・TFの手順](../../../docs/vehicle_layout.md)

## 設定操作

ROS node 名は prefix で閲覧範囲を制限し、書込対象は `parameter_allowlist_json` の node完全名→parameter名配列で明示する。例 `{"/atd_example":["gain"]}` は説明用で、そのnodeを導入しない。既定 `{}` は書込許可なし。descriptor の read_only/type/range/step を検査し、対象 node の SetParametersAtomically service に一度要求する。GetParametersで反映値を再取得し、requested/values/applied を返す。永続YAMLを勝手に書き換えない。timeoutは結果未確認として返す。

Joy profile は物理入力配置専用。新鮮な設定モード状態とprofile_path一致、revision一致を要求して atomic 保存する。保存・再読込・入力出力再開は別操作。再開は Joy node 自身が入力解放を検査する。未接続 service・不明な状態は成功にしない。車両の走行モード・ステアリング・throttleコマンドを受け付ける endpoint は持たない。

Teleop profile も、新鮮な teleop 設定モード状態、profile_path 一致、revision 一致を要求して atomic 保存する。保存したファイルと node の反映済み設定を比較して `applied` を返し、保存だけで反映済みとは表示しない。UI は設定開始（`enabled: true`）、保存、再読込、入力を離して操作出力再開（`enabled: false`）の順に行う。保存内容が node に反映されていない場合は再開を拒否する。再開時の入力解放判定は teleop node が行う。service の失敗や timeout 後は観測状態を失効させ、新しい状態を受け取ってから次の設定操作を受け付ける。操作出力の再開は車両の走行許可を意味しない。

Propo profile は `recording`・`auxiliary_1`・`auxiliary_2` の役割を CH4〜CH16 または未割り当て（0）へ設定する。6chに限定せず、10chプロポにも同じprofile v1を使う。初期値は全て未割り当て、Pico の操舵 CH1・スロットル CH2・操作権切替 CH3 は変更しない。各役割の raw low/high 閾値・反転と共通 debounce を保存する。応答には `roles`・`channels`・`fixed_roles` を含め、UIはこの候補から設定する。PWM相当µsはUI内での監視用換算で、APIは元の全16ch rawを保持する。変更要求は teleop と同じく bridge の `instance_id` に結び付ける。設定開始、revision を付けた保存、再読込、イベント出力再開を別操作とし、`applied` は保存内容・反映済み設定・path と新鮮な RC node 状態を比較する。設定モード中の保存は Pico 未接続でも可能だが、再開には反映済み設定と同じ Pico の新鮮で正常な RC 入力が必要。再開時は RC node が入力を再検査し、スイッチの基準位置を取り直す。Pico の走行制御や firmware の CH 設定は変更しない。

消費する service は対象 node の `list_parameters` / `describe_parameters` / `get_parameters` / `set_parameters_atomically`（rcl_interfaces/srv）、および atd_joy / atd_teleop / atd_rc_input の configure（std_srvs/srv/SetBool）/reload_profile（std_srvs/srv/Trigger）。bridge 自身の提供ROS serviceは標準parameter serviceだけ。

## parameter

全設定は [config/bridge.yaml](config/bridge.yaml)、起動時固定/read_only。QoSはコードのROS側指定であり、DDS XMLにはtopic QoSを設定しない。

| parameter | 既定 | 意味 |
| --- | --- | --- |
| use_sim_time | false | ROS標準時刻 |
| port | 8790 | loopback HTTP、1024〜65535 |
| recording_status_topic | /recording/status | 記録managerの状態を観測 |
| vehicle_status_topic | /vehicle/layout/status | 車体TF nodeの発行状態 |
| vehicle_profile_path | /data/results/runs/vehicle/layout.yaml | 車両配置の永続設定 |
| freshness_s | 2.5 | 0.1〜30秒、受信monotonic時刻で鮮度判定 |
| localization_topic | /localization/status | LocalizationStatus。使用地図・追跡状態・同じ観測時刻の姿勢と車体速度をまとめて取得 |
| planning_topic | /planning/status | 前方軌道の入力 |
| control_topic | /control/status | 操舵計算の監視入力 |
| speed_control_topic | /control/speed_status | 速度・スロットル比較の監視入力 |
| auto_drive_topic | /vehicle/auto_drive/status | CH3・操作元、送信指令・基板出力、停止理由の監視入力 |
| odometry_topic | /localization/odometry | Odometry入力 |
| pose_topic | /localization/pose | PoseStamped入力 |
| velocity_topic | /vehicle/velocity | TwistStamped入力 |
| joy_topic | /joy | 正規化入力 |
| joy_raw_topic | /joy/raw | 生入力 |
| joy_status_topic | /joy/status | 機器状態 |
| joy_node | /atd_joy_node | Joy service対象node |
| profile_path | /data/results/runs/input/dualsense.yaml | 既存results mount内の永続配置 |
| teleop_status_topic | /teleop/status | 操作割り当て node の状態 |
| teleop_command_topic | /teleop/command | 割り当て後の DualSense 操舵・スロットル |
| joy_drive_status_topic | /vehicle/joy_drive/status | Pico の CDC0 状態報告と DualSense の運転状態 |
| teleop_node | /atd_teleop_node | Teleop service 対象 node |
| teleop_profile_path | /data/results/runs/input/teleop.yaml | 操作の役割設定の永続配置 |
| pico_status_topic | /vehicle/pico/status | USB receiver の基板状態 |
| rc_status_topic | /rc/status | プロポ役割 node の状態 |
| rc_node | /atd_rc_input_node | プロポ役割 service 対象 node |
| rc_profile_path | /data/results/runs/input/propo.yaml | プロポの Jetson 側役割設定の永続配置 |
| parameter_node_prefix | /atd_ | 閲覧対象nodeの完全名prefix |
| parameter_allowlist_json | {} | 変更許可nodeとparameter名のJSON object |

## 検証

`colcon test --packages-select atd_web_bridge` はキャッシュ鮮度・世代不一致・profile競合、操作割り当ての保存と反映の区別、設定 service の単発要求・timeout 後の状態失効を検証する。Pico は受信済み telemetry 自体の古さ、切断、S.BUS異常、役割 node との入力世代の不一致、固定 CH の編集拒否も検査する。service の単体テストには in-process fake を用い、ROS 通信は行わない。Notebook単体でROSをimportせずWebを起動できる。ROS build/install・DDS/service結合・Jetson/実機は未検証であり、単体テスト通過をこれらの検証済みとは扱わない。

ESC 試験専用の起動では、`GET /esc/status`、`POST /esc/prepare`（接続中の instance_id と手順列）、`POST /esc/cancel`（試験 ID）を使う。prepare は Jetson の試験キューへ保存する。出力を開始する操作は Jetson の実行端末が担当する。[手順](../../../docs/bringup.md#タミヤ-esc-の手順試験)
