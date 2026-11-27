# atd_mapping

保存 cuVSLAM 地図の複製を bag で再生し、VGL の姿勢 hint と保存地図への localize 成功を確認した後の可視化点群・SLAM 軌跡を取得する。`kart_mapping` の capture 部分を参考にした独立 package。`kart_bringup` や車両・センサー起動に依存せず、駐車・制御・録画は起動しない。

Python は低頻度の処理監視・JSON保存と ROS service/topic 仲介に用い、画像計算は NVIDIA の C++ components が担当する。CMake / ament_cmake_auto / ament_cmake_python で install し、ROS の system Python と互換な環境で実行する。

## 操作と出力

Notebook の共通 Web UI が地図・VGL・モデルを検査し、元地図を staging に複製して job JSON を作る。通常は UI の点群取得操作から起動する。直接の実行入口は次のとおり。

```bash
./scripts/build.sh --packages-up-to atd_mapping
# ROS 環境と workspace install/setup.bash をsourceしたNotebook Docker内
ROS_DOMAIN_ID=94 ros2 run atd_mapping capture_snapshot --job /absolute/path/capture_job.json
```

`capture_snapshot` が `capture.launch.py job_file:=...` と `ros2 bag play --clock` を管理し、終了・取消時は子プロセス群も終了する。`capture.launch.py` は components だけの内部入口で、単独では bag 再生・snapshot 保存を行わない。入力は rectified 左右画像、CameraInfo、base frame から両 optical frame までの `/tf_static`。bag 内の動的 `/tf` は再生しない。Stereo VO のみで、IMU/RGBD はこの処理では利用しない。

VGL hint を cuVSLAM の initial_pose へ接続し、`localized_in_exist_map=Yes` と `vo_status=OK` の診断を受信するまで点群を採用しない。localize 喪失時は cloud/path/TF を破棄し、次の成功区間から取り直す。元地図へ保存する parameter は空に固定する。原点 hint への fallback は行わない。

Notebook の点群取得では `slam_max_map_size=0` を指定し、読み込んだ地図の作業コピーで姿勢グラフの保持数を無制限にする。点群取得時の値は `snapshot.json` の `provenance.parameters.cuvslam.slam_max_map_size` で確認できる。地図作成時に縮約された姿勢を復元するには、元の bag から地図を作り直す必要がある。

成功時だけ `output/snapshot.json` を atomic 保存する。schema は `atd.snapshot.v1`、frame は `map`、`points`、`trajectory`、`raw_points`、`raw_path` と `provenance` を持つ。provenance は map_id/bag、採用した parameter、入力設定、初期VGL hint、最後のlocalize区間、diag/path/cloud/TF stamp、末尾画像stamp、適用TF、地図・config・ONNX・engine hash、publisher GID、点数を記録する。

diag・SLAM path・map→odom の stamp が入力末尾許容範囲に到達し、更新が settle_s の間落ち着くことを要求する。可視化 cloud は更新駆動なので末尾stampとの一致を要求せず、**最後のlocalize成功区間の最新非空 cloud** を用いる。`cloud_tail_age_s` / `cloud_within_tail_tolerance` を残し、古い場合はログとUIで表示する。全地図の landmark 網羅性は保証しない。

点群・軌跡が既に map frame なら変換しない。odom 等なら最終timestampの map←source TF を一度だけ適用する。TF不足・非有限値は identity 変換で補わない。`/tf` は cuVSLAM の `/visual_slam` が配信元で、その publisher GID を記録する。VGL は TF 配信を無効にしても `/tf` の publisher を登録するため、この登録を許容する。想定外の配信者や取得中の配信元交代はエラーになる。[NVIDIA VGL の実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mapping_and_localization/blob/release-5.0/isaac_ros_visual_global_localization/src/visual_global_localization_node.cpp#L322-L355)

### 点群取得の起動待ちで止まったとき

両 component の `Loaded node` が出た後、サービス、左右画像・CameraInfo と姿勢 hint の購読、出力 publisher が揃うと bag 再生を開始する。起動待ちでタイムアウトした場合は、ジョブログで未成立の接続を確認する。その段階の `SIGINT` は capture が子プロセスを終了したときにも表示される。VGL 読み込み時の `0 map-points` は VGL 地図の件数で、cuVSLAM の表示用点群の点数は再生中に自己位置合わせが成功してから決まる。

再生終了後は、診断・VGL hint・点群・軌跡・TF が揃うまで最大 `drain_timeout_s` 待つ。末尾の検査で失敗した場合は、ジョブログの `ValueError` で「成功を一度も確認できない」「成功後に追跡条件を失った」「入力末尾より診断や軌跡・TF が遅れた」を判別できる。診断値 `localized_in_exist_map` と `vo_status`、入力末尾と受信データの stamp を確認する。`VGL hint=未受信` は collector が姿勢 hint を受信していないことを示す。点群・軌跡・TF の stamp は最終 localize 成功区間で採用したデータのもの。

旧版で起動待ちがタイムアウトした場合は、修正を反映した Notebook の Docker 内で `atd-build --packages-up-to atd_mapping` を実行する。Web UI で同じ保存地図を選び、「保存地図から点群を取得」→「点群の取得を開始」で再試行できる。元の bag、保存地図、Notebook 用 engine は再利用する。

## job JSON

| field | 内容 |
| --- | --- |
| map_id | アプリの地図ID |
| map_dir | staging の `nvidia/cuvslam_map` 絶対パス |
| bag / output | bag と空の取得出力directoryの絶対パス |
| workflow | left_image/right_image/left_info/right_info、base_frame/left_frame/right_frame、tf_static(`/tf_static`)、ros_domain_id(94)、ground_constraint(地図作成時のbool) |
| capture | 下表の再生・timeout設定。未指定の項目は package既定 |
| vgl | map_root(staging/nvidia)、model_dir(Notebook engine)、config_dir(staging/vgl_runtime_config)、input_shape(`[1,3,H,W]`) |

backend が行う元資産と VGL の対応・provenance検査に加え、ROS側でも必要ファイルの存在・hash、bag型と静的TF、画像解像度、実GPUでのengine deserializationを検査する。

| capture option | 既定 | 範囲・意味 |
| --- | --- | --- |
| replay_rate | 1.0 | 0.05〜4、bag再生倍率 |
| ready_timeout_s | 120.0 | 1〜600秒、service/component/購読接続待ち |
| drain_timeout_s | 30.0 | 1〜300秒、再生完了後の処理待ち |
| settle_s | 3.0 | 0.1〜30秒、更新収束待ち。drainより短いこと |
| tail_tolerance_s | 0.5 | 0.01〜5秒、diag/path/TFの末尾許容幅 |

これらは ROS parameter ではなく、[config/capture.json](config/capture.json) と同じ既定を持つjob設定。Collector の既定node名は `/atd_map_collector`、独自ROS parameterはなく `use_sim_time=true` を明示する。C++ component の全parameterは [capture_cuvslam.yaml](config/capture_cuvslam.yaml)・[capture_vgl.yaml](config/capture_vgl.yaml) に列挙し、出典と対応SDKをコメントで記録する。実行時は job の asset path、frame、bag clock、地図作成時に記録した ground_constraint を反映する。ATD の新規公式地図は cfg_planar=false を明示し、capture の odometry/slam 両 ground constraint も false に揃える。記録がない地図を推測で平面拘束ありにしない。containerは `/atd_mapping_container`、componentsは `/visual_slam`・`/visual_global_localization`。launch引数は `job_file` 一つ。

## Collector の入出力とQoS

すべて入力、best effort / volatile / keep-last depth5（sensor_data）。VGL hintだけ reliable / volatile / depth10。ROS標準 `/rosout`・`/parameter_events` は省略する。

| topic | 型 | 用途 |
| --- | --- | --- |
| /visual_slam/vis/landmarks_cloud | sensor_msgs/msg/PointCloud2 | 最新の非空可視化landmark |
| /visual_slam/tracking/slam_path | nav_msgs/msg/Path | 最終成功区間の軌跡 |
| /tf | tf2_msgs/msg/TFMessage | map←source最終TF |
| /diagnostics | diagnostic_msgs/msg/DiagnosticArray | cuVSLAM localize/VO状態 |
| /atd_mapping/pose_hint | geometry_msgs/msg/PoseWithCovarianceStamped | VGL hintの受信確認 |

Collectorはtopicを発行しない。`/visual_slam/localize_in_map`（isaac_ros_visual_slam_interfaces/srv/LocalizeInMap）の可用性を確認するが、VGLのhint経路を使用し、このCLIから原点でservice要求しない。C++ components の画像入力は workflow の4topic（sensor_msgs/Image, CameraInfo）、VGL request_hintは `/atd_mapping/request_hint`、VGL診断は `/atd_mapping/vgl_diagnostics` に分離する。DDS は Fast DDS SHM が既定で、探索には UDPv4 loopback を使う。UDP で比較する場合は `ATD_REPLAY_DDS=udp` を指定する。topic QoSはROS側で指定し、bag `/tf_static` のtransient_localは [replay_qos.yaml](config/replay_qos.yaml) で指定する。

## 検証

`colcon test --packages-select atd_mapping` は localize epoch、古い診断、TF末尾選択・一回変換、bag metadata、VGL/config解決、publisher交代検出を検証する。ホストのunit/syntax検証と、ROS build/install、実GPU deserialization、bag再生、実地図のlocalize/可視化点群の結合動作は分けて記録する。ROS/GPUの結合は未検証。
