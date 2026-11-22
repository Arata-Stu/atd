# atd_localization

D455 の左右 rectified IR と CameraInfo、実測した車体・カメラ間の静的 TF を使い、後輪軸中央の `base_link` を保存地図上で追跡する。cuVGL が初期姿勢候補を出し、cuVSLAM が保存地図への位置合わせとステレオ VO を担当する。ライブ・bag 再生ともに `localization.launch.py` を使用する。

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

CLI でも Notebook Docker 内で実行できる。選択を省く場合は `--map course_a --bag session_b/lap_2` のように ID と記録ルートからの相対パスを指定する。

```bash
/workspaces/atd/scripts/build.sh --packages-up-to atd_localization
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run atd_localization validate_bag
```

既定の隔離 ROS domain は 95。地図作成 bag の識別情報、別走行のステレオ入力、静的 TF、対象 GPU 用 engine を検査する。`/data/results/runs/localization/<run>/report.json` に初期位置確定までの時間・追跡時間率・喪失等、`trajectory.csv` に有効な軌跡と車体速度、`hdmap.json` に検証時の HDMap と Raceline を保存する。絶対位置誤差の評価には別途基準位置が必要になる。

## ROS 出力

cuVSLAM が `map → odom → base_link` の動的 TF を発行する。`/localization/pose_hint` は VGL の初期位置候補。`localization_monitor` は診断、同じ観測時刻の `map → odom`・VO odometry、CameraInfo から `/localization/status`（`atd_interfaces/msg/LocalizationStatus`）を配信する。

追跡中は新しい画像時刻の観測が揃った時点で、その姿勢を1回配信する。計画・制御の更新はこの入力に従い、ステレオ30 Hz・90 Hzの切り替えにモニター側の設定変更は不要。Gazebo の閉ループでは同じ追跡区間内で観測済みの直近の地図補正 TF を使える。追跡喪失は診断の受信時に通知し、起動時の VGL 要求・入力途絶の検査・有効な姿勢を待つ間の状態通知は100 ms周期で行う。

初回の候補が購読準備より早く届いた場合に備え、左右 CameraInfo と cuVSLAM 診断、候補の両購読者、VGL サービスの準備を確認してから、位置合わせ待ちの間に `/localization/request_hint` を一度要求する。応答失敗・タイムアウト時は合計3回まで試す。位置合わせの成否は、その後の候補・診断・TF・オドメトリで判定する。

`state` は `waiting_hint`、`localizing`、`tracking`、`lost`、`stale`。姿勢と車体 x 方向速度 [m/s] は `tracking` 時だけ有効。地図 ID・cuVSLAM 地図と HDMap の SHA-256・セッション ID を同じメッセージに持たせ、別の地図や追跡区間の姿勢を識別する。
