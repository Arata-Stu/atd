# Notebook の Gazebo ステレオ走行

シティサーキット東京ベイの屋外 SKY TRACK を1/5で再現し、車載 D455 相当のステレオ画像と走行軌跡を取得する。[お台場の2周記録・地図作成・ループ閉じ込みの判定](gazebo_circuits.md)

## Notebook で再生・測定する

NVIDIA GPU と NVIDIA Container Toolkit が使える Ubuntu の Notebook ホストで、ATD のルートから実行する。Notebook ホストの `atd` メニューからも `gazebo` を選べる。

```bash
atd gazebo prepare
atd gazebo run --course odaiba --speed 3
```

`prepare` は初回・Dockerfile 更新時にオンラインで実行する。ROS 2 Lyrical と Jetty の配布パッケージから専用イメージを作り、容量の明細を出力する。以降はコース・画像素材・プログラムを clone から読めるためオフラインで使える。既存の Isaac ROS 開発コンテナを起動する必要はない。Gazebo はサーバーモードで OGRE 2 の EGL 描画を使う。[公式の対応構成](https://gazebosim.org/docs/jetty/ros_installation/)・[Gazebo のヘッドレス描画](https://gazebosim.org/api/sim/10/headless_rendering.html)

`run` は動画用と速度計測用にシミュレーションを最初から個別に実行する。端末に各試行の保存先が出る。標準では `data/results/runs/gazebo/odaiba/`。保存先を設定済みなら Notebook 用 results の同じ相対位置に保存する。

| 操作 | 成果物 |
|---|---|
| `prepare` | `storage.json`、ビルド前後の `docker system df -v` と `docker buildx du`、`build.log` |
| `video` | `stereo.mp4`（左目／右目の横並び）、`left.mp4`、`right.mp4`、`video.json` |
| `bag` | 指定周回分（既定1周）のステレオ MCAP。完了後、Notebook の `data/record/rosbags/` から Web UI で選択できる |
| `benchmark` | `benchmark.json` に実時間で受信したペア数・Hz、シミュレーション時刻のHz、欠落・重複、最大到着間隔 |
| `diagnose` | 既定20実秒間の ROS 受信を `benchmark.json`、同じ時間帯の Gazebo Transport での左右受信と world 時刻を `gz_probe.json` に記録 |
| 各走行 | `run.json`、`scene/scene.json`、台の実際の位置 `scene/rig_motion.jsonl`、サーバー・ブリッジ・受信側のログ |

`stereo.mp4` を Notebook のブラウザにドラッグすると、左右を同時に再生・シークできる。実時間の配信と独立に、動画は同じタイムスタンプを持つ左右画像をシミュレーション時刻で30 fpsにサンプリングする。動画は既定1周で、例えば `atd gazebo video --speed 3 --laps 2` で2周分を保存できる。受信が滞り前のペアを保持した動画フレームの数は `video.json` の `held_video_frames` で確認できる。

## 記録済みのステレオ bag を RViz / Foxglove で見る

Ubuntu Notebook のデスクトップで使う。初回はホストで `atd dev --build` を実行して RViz を含む開発コンテナを起動し、その中で `atd build --packages-up-to atd_bringup` を実行する。すでに開いているコンテナで `command -v rviz2 && ros2 pkg prefix rviz_default_plugins` が通れば、その中では ROS パッケージのビルドだけで試せる。以降はホストまたは起動済みの対応コンテナで次のコマンドを使う。

```bash
atd replay
```

引数なしの `atd replay` または `atd` のメニューから `replay` を開き、追跡方式（VO は左右画像、VIO は左右画像と統合 IMU）、再生する bag の順に選ぶ。VIO には `/camera/camera/imu` を記録した bag を使う。cuVSLAM と RViz が起動し、左右の画像・CameraInfo と車体から両カメラへの静的 TF を記録から確認する。購読の準備が整うと 0.5 倍速の再生が始まる。RViz は左右の IR 画像を Best Effort で購読し、`map` 座標でのランドマークと軌跡も表示する。bag の末尾でも画面は開いたままで、RViz を閉じるか端末で Ctrl+C を押すと一式が終了する。地図はメモリ上で生成される。

旧形式の Gazebo bag も再生一覧に表示する。例えば名前に `20261121T125125Z` が入った記録は `2026-11-21 21:51:25 JST` と表示され、対応する走行ログがあれば解像度・撮影レート・IMU 設定も読める。パスを直接指定する際は元のディレクトリ名を使う。

Gazebo の bag を選ぶと、RViz に次の3つが表示される。

| RViz の表示 | 読み方 |
|---|---|
| 青の **Gazebo ground truth** | Gazebo の物理ステップ後に測った後輪軸の軌跡。再生時計 `/clock` に沿って伸びる。 |
| 橙の **SLAM trajectory** | ステレオ画像から cuVSLAM が推定した `map` 内の車体軌跡。 |
| 白の **Position error at latest estimate** | 直近の推定位置と、その**画像時刻**の正解位置を結ぶ。線の長さが位置誤差で、追跡が途切れると最後に比較できた時刻の線が残る。 |

正解の記録区間で最初の有効な cuVSLAM 姿勢を受け取ると、その画像時刻に正解を補間し、初期位置と向きを一度だけ合わせる。以降は縮尺と変換を固定する。処理遅延があると青と橙の先端が示す時刻は異なるため、同時刻のずれは白線で読む。正解の配信先は `/atd/replay/ground_truth_path`、白線は `/atd/replay/position_error`。

新しい `atd gazebo bag` は通常シミュレーション時間で 20 Hz を目安に正解を記録し、bag に `atd_ground_truth/rig_motion.jsonl` を同梱する。既存の Gazebo bag でも、元の名前の bag と、同じ試行の `run.json` および `scene/rig_motion.jsonl` が Notebook の結果ディレクトリに残っていれば自動で照合する。お台場の保存先は `/data/results/runs/gazebo/odaiba/<run-id>/`。古い記録では正解のサンプル間隔が約 0.5 秒になることがあり、コーナーのサンプル間は直線補間される。正解が見つからない場合は端末にその旨を表示し、推定軌跡を再生する。

`atd replay --rate 1` で等速になる。bag 名を指定する場合は `atd replay "親ディレクトリ/bag名" --rate 0.5` のように `/data/record/rosbags/` からの相対パスを渡す。コンテナ内の絶対パスも使える。再生用の domain は既定 75（`ATD_REPLAY_ROS_DOMAIN_ID` で変更可）。専用の Fast DDS 設定と bag の `/clock` を使う。

`Failed to track: Timestamps are non-monotonic` が続いてRVizの表示が点滅する場合は、再生中に別端末の Notebook の Isaac ROS コンテナで次を実行する。通常、オドメトリの配信元は `/visual_slam` の1つ。再生前に同名のトラッカーを2つ検出した場合、`atd replay` は競合を報告する。

```bash
export ROS_DOMAIN_ID=75
export FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/atd/docker/gazebo_monza/fastdds.xml
ros2 topic info /visual_slam/tracking/odometry --verbose --no-daemon
```

同じbagを別の空いているdomainで比較するには `ATD_REPLAY_ROS_DOMAIN_ID=76 atd replay` を使う。ログを照合する際は `ros2 launch` が表示した `component_container_mt` の PID とログファイル名の PID を合わせる。

`_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_` は、DDS が配信を検出し、その配信元を ROS ノードとまだ対応付けられていない状態。再生開始時は名前の解決を待ち、実際に別の配信元を確認した場合は名前を表示して再生を止める。待機中に調べるには、端末に表示された診断コマンドを別の Notebook コンテナ端末で実行する。Gazebo の既定 domain は 74、replay は 75。終了後に通常のシェルで実行した `ros2 topic list` とは、探索する時刻や domain が異なる。[ROS 2 の配信元名の実装](https://github.com/ros2/rmw_dds_common/blob/lyrical/rmw_dds_common/src/graph_cache.cpp#L514-L533)

### 同じ bag でステレオ VO とステレオ＋IMU を比較する

Notebook ホストで次を実行すると、848×480・60 Hz のステレオと生の加速度200 Hz・ジャイロ400 Hz、統合 IMU 400 Hz を同じ時刻系で記録できる。`--resolution` は `424x240`、`640x480`（既定）、`848x480`、`1280x720` の4通り。`--camera-hz` は30・60・90 Hz（既定）から選び、左右画像と CameraInfo に共通で適用する。IMU は既定で `off`、`--imu 200` では3ストリームとも200 Hzになる。

```bash
atd gazebo bag --course odaiba --speed 3 --laps 2 --resolution 848x480 --camera-hz 60 --imu 400
```

IMU を有効にした bag には発進前の静止区間が最低2シミュレーション秒入る。保存後に各センサーの時刻、固定 TF、周回中の欠落を確認する。各 IMU トピックの実効 Hz は `run.json` の `bag_audit.imu_streams` で確認できる。

Notebook の Isaac ROS コンテナに入り、初回のワークスペースでは `atd build --packages-up-to atd_bringup` を実行する。次の2回の再生で同じ bag を選び、1回目の RViz を閉じてから2回目を起動する。

```bash
atd replay --tracking vo
atd replay --tracking vio
```

`vo`（既定）は左右画像、`vio` は左右画像と統合 IMU を使う。VIO 起動時は bag に記録された統合 IMU のフレーム名・時刻から座標系と周期を読み取る。以前のステレオ専用 bag は `vo` で評価できる。統合 IMU がない走行を `vio` と比較するには、IMU を有効にして収録する。Notebook UI の「VSLAM 軌跡誤差」で方式と地図上限を確認し、同じ周回・区間の GT 誤差を比較する。

### Notebook UI で区間別の誤差を見る

Notebook の Isaac ROS コンテナでは、初回に `atd build --packages-up-to atd_bringup` を実行する。Web 環境を初めて準備する場合は `atd-web prepare` を使う。GT のある bag を `atd replay` で再生すると、比較結果が自動で `/data/results/runs/replay/<実行ID>/` に保存される。再生ごとに新しい ID が付く。

`atd-web` を起動し、**VSLAM 軌跡誤差** で結果を選ぶ。グラフ上で誤差が増え始める地点をクリックすると、コース図にもその地点の正解と推定が表示される。逆にコース図からグラフの地点を選ぶこともできる。

`atd replay` と `atd gazebo map` は、既定で cuVSLAM の姿勢グラフの保持数を無制限（`slam_max_map_size=0`）にする。同じ bag の計時線再訪時の誤差を比較するには、再生速度をそろえ、2回目の `atd replay` に `--slam-max-map-size 300` を付ける。結果一覧には各回の地図上限が表示される。設定を記録していない旧結果は「地図上限 未記録」と表示する。

- **進行方向 (m)** は同じ画像時刻の「推定位置 − 正解位置」を正解車体の前方に射影する。正なら実際より先に、負なら手前に推定している。
- **横方向 (m)** は走行方向に対して左が正。**向き (°)** は正解に対して反時計回りが正で、±180°の範囲で表す。
- GT は全走行を表示する。対応する推定が 0.25 秒を超えて途切れた箇所では誤差の線を分ける。中断した再生では未再生区間にも推定値がない。古い bag に周回情報を対応付けられない場合は、累積距離で表示する。

保存物は `summary.json`（bag・再生速度・終了状態）、`reference.json`（時刻付き正解軌跡）、`samples.jsonl`（画像時刻・同時刻の正解と推定・符号付き誤差）。再生完了は bag の末尾に到達したことを表す。

### 手動で RViz または Foxglove を使う場合

ビューアや再生の起動を個別に操作したい場合は、同じコンテナの3端末それぞれに次を設定する。

```bash
export ROS_DOMAIN_ID=74
export FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/atd/docker/gazebo_monza/fastdds.xml
```

端末1は `ros2 launch atd_bringup bag_vslam.launch.py`。cuVSLAM を `component_container_mt` にロードする。ログに `Loaded node '/visual_slam'` が出たら準備完了。

端末2は RViz の場合、次の ATD 設定を使う。Foxglove の場合は `ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8765 use_sim_time:=true` を起動し、Notebook のアプリで `ws://127.0.0.1:8765` に接続する。3D の基準フレームは `map`、点群は `/visual_slam/vis/landmarks_cloud`、軌跡は `/visual_slam/tracking/slam_path`。画像は `/camera/camera/infra1/image_rect_raw` を選ぶ。

```bash
rviz2 -d "$(ros2 pkg prefix --share atd_bringup)/config/bag_vslam.rviz" \
  --ros-args -p use_sim_time:=true
```

端末3で bag を選んで再生する。0.5倍速で開始し、シミュレーション時刻とカメラの静的 TF は bag から供給する。

```bash
bag_meta=$(find /data/record/rosbags -name metadata.yaml -type f | sort | fzf --prompt='再生する bag > ')
if [[ -n "$bag_meta" ]]; then
  ros2 bag play "${bag_meta%/metadata.yaml}" --clock --rate 0.5 \
    --qos-profile-overrides-path /workspaces/atd/ros2_ws/src/atd_mapping/config/replay_qos.yaml \
    --topics /tf_static /camera/camera/infra1/{image_rect_raw,camera_info} /camera/camera/infra2/{image_rect_raw,camera_info}
fi
```

## Gazebo の時刻で cuVSLAM を起動する

**1周を記録して後から地図を作る場合**は、Notebook ホストで `atd gazebo bag --course odaiba --speed 3` を実行し、完了後に Notebook の Web UI「地図とライン」で bag を選ぶ。走行中に地図を作りたい場合は `atd gazebo map --course odaiba --speed 3` を使う。以下の `serve` と `bringup gazebo` は連続走行中のステレオ VO を観測する操作。[記録とオフライン地図作成の手順・保存容量](gazebo_circuits.md#1周でcuvslam地図を作る)

Ubuntu Notebook で ATD 開発コンテナ（`atd dev`）を起動しておく。初回のワークスペースでは次のコマンドで launch をインストールする。Gazebo の専用イメージは `atd gazebo prepare` 済みのものを使える。

```bash
atd build --packages-up-to atd_bringup
```

**端末1・Notebook ホスト**で Gazebo を起動する。左右の画像と CameraInfo に対応する購読がそろうまで世界を一時停止する。

```bash
atd gazebo serve --course odaiba --speed 3 --wait-for-stereo
```

**端末2・同じ Notebook のホストまたは ATD 開発コンテナ**でステレオ VO を起動する。地図や D455 実機の接続は不要。

```bash
atd bringup gazebo
```

全周ではゴールを越えると同じ向きで次周に入り、`/clock` は単調に進む。左右画像・CameraInfo の時刻は Gazebo の撮影時刻。cuVSLAM とその component container は `use_sim_time=true` で動作し、`/visual_slam/tracking/odometry` と `odom → base_link` の TF を出力する。例えば実時間1秒で `/clock` が0.47秒進むと、姿勢と速度の時間軸も0.47秒進む。実時間の90 Hz達成は起動条件に含まれない。今回は画像による相対姿勢推定を対象とし、車両 I/O は Jetson 側の従来の profile が担当する。[ROS 2 の時計](https://design.ros2.org/articles/clock_and_time.html)

両端末は既定で ROS domain 74 を使用する。変更時は両方で同じ `ATD_GAZEBO_ROS_DOMAIN_ID` を設定する。連続配信では Gazebo と Isaac に同じ SHM＋loopback の Fast DDS プロファイルを適用し、Isaac ROS CLI 標準構成に合わせて host network と IPC を共有する。動画・計測は従来どおり専用コンテナの IPC で完結する。[Isaac ROS CLI の起動設定](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/blob/main/scripts/run_dev/run_dev.py)・[Fast DDS の Docker 通信](https://fast-dds.docs.eprosima.com/en/3.x/docker/shm_docker.html)

**別端末の Notebook ATD 開発コンテナ**で、シミュレーション時計の有効化・進行と VO の出力を確認できる。

```bash
export ROS_DOMAIN_ID="${ATD_GAZEBO_ROS_DOMAIN_ID:-74}"
export FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/atd/docker/gazebo_monza/fastdds.xml
ros2 param get /visual_slam use_sim_time
ros2 topic echo /clock --once
ros2 topic echo /visual_slam/status --once
ros2 topic echo /visual_slam/tracking/odometry --field header.stamp --qos-reliability best_effort --once
```

終了は両端末で Ctrl+C。Gazebo を起動し直すと `/clock` はゼロから始まるので、VO も新しいセッションで起動する。`atd gazebo serve` 単独なら購読を待たずに配信する。

### 取付高・時間を変える

```bash
./scripts/gazebo.sh video --camera-height 0.14 --camera-pitch-deg 3
./scripts/gazebo.sh benchmark --duration 60
```

`--camera-height` は床から光学中心までの m。初期値は **0.12 m**、後軸から前方に 0.18 m、水平と仮定している。軸中心高さの仮定値は 0.0325 m。車体設定の後輪軸基準の z と区別する。左右の基線は公称 0.095 m、水平画角は公称 87°の `mono8` 理想ピンホールカメラ。全解像度で水平画角を固定し、縦の画角と CameraInfo の焦点距離・主点は選択した解像度に従う。640×480 の例では焦点距離は約337.2 px、主点は (320, 240) で、撮影レートだけを変えても同じ値になる。実機の個体別校正や解像度ごとのクロップは反映していない。実機の取付高は未測定。モノクロ画像は可視光材質の描画で、D455 の投光パターン、露出制御、振動による像の変化までは再現していない。[実車の座標定義](vehicle_layout.md)・[D455 仕様](https://realsenseai.com/products/stereo-depth-camera-d455/)

動画の `--duration` は1周を走るシミュレーション秒数。全周動画は `--laps` で周回を増やせる。ベンチマークの `--duration` は実時間の計測窓で、走行速度は `--speed` で独立に選べる。移動は地図上の走路中心付近をたどる軌道再生で、映像と配信の検証用。タイヤや ESC、制御器による閉ループ走行の比較は既存の [2D シミュレーター](simulation.md) で行う。

### 左右画像の受信が遅いとき

Notebook ホストで次を個別に実行する。先の試行の終了コードが2でも次を実行できる。イメージを準備済みなら、この診断と DDS 設定は checkout の更新で使える。

```bash
atd gazebo diagnose --dds udp
atd gazebo diagnose --dds shm
```

両方とも同じシーン・計測時間を使う。`udp` は UDPv4 loopback、`shm` は Fast DDS の共有メモリも使える設定で、動画・計測・連続配信の既定値。動画・計測では Gazebo コンテナ専用の `/dev/shm` 上限を128 MiBにする。`serve` では Isaac ROS と IPC を共有し、別コンテナの cuVSLAM にも共有メモリで画像を渡せる。[Fast DDS の SHM transport](https://fast-dds.docs.eprosima.com/en/3.x/fastdds/transport/shared_memory/shared_memory.html)

| 診断に表示される結果 | 確認する箇所 |
|---|---|
| Gazebo 側ですでに映像が少なく、world 時刻もゆっくり進む | `gz_probe.json` の `world.real_time_factor`。1.0 が実時間相当。描画とシミュレーション全体の負荷を確認する |
| Gazebo 側は左右とも十分届き、ROS で片眼または両眼の枚数が落ちる | ブリッジ、DDS、ROS 購読処理。2回の結果で共有メモリを有効にした際の変化を見る。UDP 受信バッファ不足の増分はホスト上の他プロセスの通信も含む |
| ROS 側でも左右の枚数は多いが、ペアが少ない | `pairing.shared_image_stamps` で時刻が一致した総数、`shared_stamps_evicted_before_match` で相手が届くまでに保持上限を超えた数を確認する |

診断用の Gazebo 購読にも画像転送の負荷がかかる。選択した撮影レートの最終判定には同じ DDS 設定・同じ `--camera-hz` で `atd gazebo benchmark` を使用する。ROS が最初の校正ペアを得られなかったときは Gazebo の診断購読期間全体を明示し、異なる観測時間の枚数を直接比較しない。

## 撮影レートと容量の読み方

- `benchmark.json` の `unique_ordered_pairs` は画像と時刻を左右で照合して、重複を除いて数えた組数。60実秒での目標は30・60・90 Hzの順に1,800・3,600・5,400組。購読接続と校正を確認し、ウォームアップを除いた区間で `paired_wall_hz` を測る。`paired_sim_hz` は画像に付いたシミュレーション時刻による周期。両者の差は実時間への追従を見る手掛かりになる。
- `pass_target_hz` は実時間のペア数/秒が指定レートの99%以上、撮影時刻で測ったレートが指定値の±1%、観測したペア間の欠落推定・逆順がゼロの場合に成立する。90 Hz設定では `pass_90hz` にも同じ結果を記録する。到着間隔の最大値も記録する。結果が閾値未達ならベンチマークは終了コード2を返す。描画やROSの起動失敗とはログと終了コードで区別する。この試行の受信側は計測専用で、cuVSLAM同時実行時の処理性能は別途測定する。
- `storage.json` の `filesystems[].net_used_bytes` はビルド前後の実ディスク差分、`peak_added_bytes_sampled` はビルド中に0.5秒間隔で観測した最大増分。共有レイヤー込みの論理イメージサイズ `image_size_after_bytes` と使い分ける。ファイルシステム全体の空き容量を読むため、測定中は同じディスクで別のビルドや大きなコピーを避ける。ビルドキャッシュ・既存イメージを自動削除する処理はない。生成した動画などの占有量は各試行の `output-size-kib.txt` に KiB で残る。

ROS は Notebook のループバックで Fast DDS を使う。標準ドメインは 74。動画・計測の起動中は ROS に `/clock`、`/tf_static`、`/camera/camera/infra{1,2}/image_rect_raw` と両方の `camera_info` が出る。Simulation 用の別プロセスで購読するときは同じドメイン・ループバック設定を使う。Jetson への接続や車両への指令は不要。

IMU を有効にした走行では、次の `sensor_msgs/msg/Imu` も配信する。Hz はシミュレーション時刻での設定値。

| ROS トピック | データ | frame_id |
|---|---|---|
| `/camera/camera/accel/sample` | 生の加速度（200 Hz、m/s²） | `camera_accel_optical_frame` |
| `/camera/camera/gyro/sample` | 生の角速度（200 / 400 Hz、rad/s） | `camera_gyro_optical_frame` |
| `/camera/camera/imu` | ジャイロ時刻に合わせた加速度と角速度。400 Hz 設定では加速度の間の時刻を線形補間 | `camera_gyro_optical_frame` |

IMU は左右レンズの中点にあると仮定し、Gazebo で観測したカメラ台の姿勢変化から比力と角速度を計算する。水平静止時の加速度は optical 座標の y 方向に約 −9.81 m/s²。路面の凹凸による振動、センサーのノイズやバイアスはモデル化していない。実機に対する VIO 精度は実機の bag で検証する。
