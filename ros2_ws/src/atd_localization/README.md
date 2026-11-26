# atd_localization

D455 の左右 rectified IR と CameraInfo、実測した車体・カメラ間の静的 TF を使い、後輪軸中央の `base_link` を保存地図上で追跡する。既定では cuVGL が初期姿勢候補を出し、cuVSLAM が保存地図への位置合わせとステレオ VO を担当する。ライブ・bag 再生ともに `localization.launch.py` を使用する。

## Jetson で手動走行を観測する

Notebook で作った cuVSLAM・VGL 地図と HDMap・Raceline を Jetson に転送し、同じモデル export から Jetson 用 engine を準備する。[起動・転送手順](../../../docs/bringup.md)・[VGL engine の準備](../../../docs/vgl_models.md)

Jetson ホストの ATD checkout で実行する。起動時に地図を選択でき、`map_id:=course_a` で直接指定もできる。

```bash
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh localization
```

運転はプロポと Pico が担当する。Notebook の ATD Studio「ライブ」で Jetson に接続し、選んだ地図の Raceline、現在位置・向き、符号付き前後速度、追跡状態を確認する。

## Notebook で別の走行 bag を検証する

通常は Notebook Docker で下記 build 後に `atd-web` を起動し、[自己位置検証](http://127.0.0.1:8766/#localization) から地図と bag を選ぶ。検証結果の地図で Raceline、区間別の推定軌跡、喪失前と復帰時の観測地点を確認できる。[UI の操作](../../../apps/web/README.md#自己位置検証)

Linux Notebook のホストまたはコンテナで `atd replay` を開き、「保存地図で自己位置を検証」→ 地図補正方式 → RViz または画面なし → 保存地図 → 別走行の bag を選ぶ。等速で再生し、末尾で結果を保存する。地図を指定して RViz を開き、bag だけ選ぶこともできる。

```bash
atd replay --validate-map odaiba_s1_cu --rate 1 --rviz
```

RViz は `map` を固定フレームにし、HDMap に登録された境界・走行ラインと、地図上の推定姿勢（橙の矢印）・走行軌跡を重ねる。追跡を失った区間では線を切り、復帰後の最初の姿勢から描画を再開する。再生の末尾で結果を保存し、ウィンドウを閉じるまで最終軌跡を確認できる。再生中にウィンドウを閉じるとその検証を中止する。直接コマンドで `--rviz` を省略すると画面なしで実行する。

TUI と直接指定は同じ通信設定を使う。bag 配信元・cuVSLAM・cuVGL は同一の component container 内でステレオ画像を渡す。別プロセスとの DDS 通信は既定で loopback UDPv4 を使う。別プロセス間の Fast DDS 共有メモリも有効にして比較する場合は、起動時に `ATD_REPLAY_DDS=shm atd replay` と指定する。実行時の DDS プロファイルと左右画像のプロセス内購読数は検証結果に記録される。

ROS のコマンドからも Notebook Docker 内で実行できる。選択を省く場合は `--map course_a --bag session_b/lap_2` のように ID と記録ルートからの相対パスを指定する。

```bash
/workspaces/atd/scripts/build.sh --packages-up-to atd_localization
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run atd_localization validate_bag
```

既定の隔離 ROS domain は 95。地図作成 bag の識別情報、別走行のステレオ入力、静的 TF、対象 GPU 用 engine を検査する。`/data/results/runs/localization/<run>/report.json` に初期位置確定までの時間・追跡時間率・喪失等、`trajectory.csv` に有効な軌跡と車体速度、`hdmap.json` に検証時の HDMap と Raceline を保存する。絶対位置誤差の評価には別途基準位置が必要になる。

### 地図補正を cuVGL に任せる

Notebook コンテナで同じ地図と bag を指定して比較する。

```bash
atd replay --validate-map odaiba_s1_cu --bag "$BAG" --rate 1
atd replay --validate-map odaiba_s1_cu --bag "$BAG" --rate 1 --map-correction-source vgl --vgl-frequency 1
```

既定では cuVGL の初回候補を受けて cuVSLAM が保存地図に照合し、地図補正を担当する。`vgl` では cuVSLAM は局所 VO と `odom → base_link` を担当し、cuVGL が照合画像と同時刻のオドメトリから `map → odom` を計算して TF を発行する。両者は同じ component container の画像を使用する。

`--vgl-frequency` は画像時刻に対する照合頻度（Hz）。`0` は初回照合に成功すると待機する。検証中に別の Notebook コンテナ端末から次のコマンドを実行すると、次の照合を要求できる。

```bash
atd-shell env ROS_DOMAIN_ID=95 ros2 service call /localization/request_hint std_srvs/srv/Trigger '{}'
```

直接 TF を購読する場合、VGL の補正には照合画像の時刻が付く。`/localization/status` は、確認できた直近の補正をその後の VO に適用して画像時刻ごとの地図上の姿勢を配信する。初回＋手動の設定では、次の照合までに蓄積した局所 VO のドリフトが残る。初回照合に成功しても画像と同時刻のオドメトリがまだなく、補正 TF を取得できなかった場合は、VO の開始後に上記サービスで再照合できる。

## ROS 出力

既定の構成では cuVSLAM が `map → odom → base_link` の動的 TF を発行する。`/localization/pose_hint` は VGL の初期位置候補。`localization_monitor` は診断、同じ観測時刻の `map → odom`・VO odometry、CameraInfo から `/localization/status`（`atd_interfaces/msg/LocalizationStatus`）を配信する。

追跡中は新しい画像時刻の観測が揃った時点で、その姿勢を1回配信する。計画・制御の更新はこの入力に従い、ステレオ30 Hz・90 Hzの切り替えにモニター側の設定変更は不要。Gazebo の閉ループでは同じ追跡区間内で観測済みの直近の地図補正 TF を使える。追跡喪失は診断の受信時に通知し、入力途絶の検査・有効な姿勢を待つ間の状態通知は100 ms周期で行う。

初回の照合は cuVGL が画像を受け取って開始する。既定の構成では、cuVSLAM は候補を受け取って保存地図に照合し、失敗時には `/localization/request_hint` トピックで cuVGL に次の候補を要求する。ATD の監視ノードは候補・診断・TF・オドメトリを観測し、追跡状態を配信する。bag 検証では候補の購読準備を待ってからセンサー再生を開始する。

`state` は `waiting_hint`、`localizing`、`tracking`、`lost`、`stale`。姿勢と車体 x 方向速度 [m/s] は `tracking` 時だけ有効。地図 ID・cuVSLAM 地図と HDMap の SHA-256・セッション ID を同じメッセージに持たせ、別の地図や追跡区間の姿勢を識別する。
