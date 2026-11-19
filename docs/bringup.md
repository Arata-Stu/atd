# Jetson の bringup

`scripts/bringup.sh` で ROS 起動 profile を選び、Jetson の起動済み ATD Docker 内で実行する。先に `scripts/dev.sh` でコンテナを起動し、必要な package を build する。

```bash
# Jetson ホスト、プロジェクト直下
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh
```

引数なしでは fzf（未導入時は番号）で profile を選ぶ。`manual` / `d455` ではストリーム設定、`localization` では使用する保存地図を選ぶ。

| profile | 起動する処理 | 起動先 |
| --- | --- | --- |
| `manual` | D455、車体static TF、Joy・Pico入力、役割処理、地図用記録待機、bridge | `atd_bringup manual.launch.py` |
| `localization` | manual 一式と cuVGL・cuVSLAM・追跡状態の配信 | `atd_bringup localization.launch.py` |
| `d455` | D455 と車体static TF | `atd_bringup d455.launch.py` |
| `esc-trial` | ESC 試験用 bridge と端末での手順実行 | `atd_bringup esc_trial_session` |
| `input` | Joy・Pico入力、役割処理、入力記録待機、bridge | `atd_web_bridge input.launch.py` |
| `bridge` | 観測・設定bridgeのみ | `atd_web_bridge bridge.launch.py` |

`manual` は手動走行時のセンサ・入力監視をまとめる名称で、車両出力や rosbag 記録は自動開始しない。`manual` / `input` は記録managerを1つ起動して待機し、Joy・プロポの割当操作で開始・停止する。プロポの基本走行は引き続き Pico 内で完結する。全 profile は Jetson ローカル専用。Notebook は Web UI と解析を担当し、車載 DDS に参加しない。

## D455 の TUI

```bash
./scripts/bringup.sh manual          # TUIで設定してからまとめ起動
./scripts/bringup.sh d455            # TUIで設定、D455・車体TFだけ起動
./scripts/bringup.sh --configure manual camera_serial:=SERIAL
./scripts/bringup.sh --dry-run manual # 実行予定と初期設定の確認。Notebookでも可
```

初期設定は次のとおり。保存済み設定がある場合はそれを編集する。

| ストリーム | 初期値 | TUIで変更する項目 |
| --- | --- | --- |
| Infra1 / Infra2 | 各 ON、640 × 480 @ 90 Hz、Y8 | 各ON／OFF、共通の解像度・FPS |
| RGB | ON、640 × 480 @ 30 Hz、RGB8 | OFF または解像度・FPS |
| Depth | OFF | OFF または解像度・FPS、Z16 |
| 加速度 | ON、200 Hz | OFF または対応Hz |
| ジャイロ | ON、400 Hz | OFF または対応Hz |

TUI は Jetson Docker 内の librealsense SDK で接続中の D455 を列挙し、選んだ機器が報告するモードを候補にする。複数台では serial を選択する。各候補の対応だけでなく、保存前・起動前にストリームの同時構成を SDK の `config.can_resolve` で検査する。要求した初期値や保存済み値が実機・firmware・USB接続で利用できなければ理由を表示して止まり、別のHzや解像度へ黙って変更しない。カメラを他の node / Viewer で使用したまま設定しない。

設定は `/data/results/runs/bringup/d455.yaml` に保存する。各ストリームは OFF にでき、無効にしたストリームのtopicは出ない。全OFFの直接起動ではSDK照会とdriver起動を省き、カメラ未接続でも車体TFなどを起動できる。IMUは既定で加速度・ジャイロの別topicを配信し、結合は行わない（`unite_imu_method=0`）。カメラの内部 static TF は RealSense driver が担当する。

対応モードの列挙・同時構成検査の実装と、実カメラでの起動・実効FPS・画像欠落・IMU周期の確認は別である。TUI は実機のSDK応答が必要であり、`--dry-run` は TUI・SDK・ROS を実行しない。

## 引数と保存設定による直接起動

```bash
./scripts/bringup.sh --list
./scripts/bringup.sh input
./scripts/bringup.sh bridge
./scripts/bringup.sh manual camera_config:=/data/results/runs/bringup/d455.yaml
./scripts/bringup.sh d455 enable_color:=false enable_depth:=false
./scripts/bringup.sh d455 infra_profile:=640x480x90 accel_fps:=200 gyro_fps:=400
./scripts/bringup.sh manual pico_serial:=PICO_SERIAL
./scripts/bringup.sh manual vehicle_profile:=/data/results/runs/vehicle/layout.yaml
./scripts/bringup.sh --dry-run        # 従来どおりinputの実行予定
```

TTY で D455 profile を選び、launch引数を付けなかった場合に設定TUIへ進む。`name:=value` を付けた場合と非対話端末では、TUIを経由せず保存設定と上書き値で起動する。引数がある場合も `--configure` を付ければ TUI を開く。非対話では profile の明示が必要で、`--configure` は使えない。

主な D455 引数は `camera_config`、`camera_serial`、`enable_infra1` / `enable_infra2` / `enable_color` / `enable_depth` / `enable_accel` / `enable_gyro`、`infra_profile` / `color_profile` / `depth_profile`（`幅x高さxFPS`）、`accel_fps` / `gyro_fps`、`unite_imu_method`。`vehicle_config` / `vehicle_profile` は車体TF用。詳細は [atd_sensors](../ros2_ws/src/atd_sensors/README.md) と [atd_bringup](../ros2_ws/src/atd_bringup/README.md)を参照する。

`manual` は既存 `input` の引数も受ける。`profile`・`teleop_profile`・`rc_profile` が入力の永続設定、`joy_config`・`teleop_config`・`pico_config`・`rc_config` が各 node の起動設定、`config` がbridge設定。`pico_serial` / `pico_sensor_path` で Pico を選び、`teleop:=false` / `propo:=false` で各入力経路を除外する。`bridge` はbridge用の `config` と `vehicle_profile` を受ける。[bridge package](../ros2_ws/src/atd_web_bridge/README.md)

`d455` はbridgeを含まないため、UIから車体配置を編集する場合は別ターミナルで `bridge` を起動するか、`manual` を使う。`manual` はbridgeを1つだけ起動する。`input` / `bridge` と重複して起動しない。

## rosbagの記録

`manual` は `recording_profile:=mapping`、`input` は `recording_profile:=inputs` が既定。車両モデル用のプロポ手動走行には `./scripts/bringup.sh manual recording_profile:=calibration` を使う。`recording:=false` でmanagerを除外し、`recording_config:=絶対パス` で独自YAMLを使える。保存先は `/data/record/rosbags/<bag_id>/`、形式はMCAP。開始処理・記録中・終了処理・保存確定をNotebook UIのコントローラ画面で観測できる。[ボタン割当と記録手順](recording.md)

## 車両の配置設定

`atd_vehicle_description` が `base_link`（後輪軸中央）から車軸・D455取付の static TF を発行する。Notebook UIの「車両設定」で mm / 度を入力し、Jetsonへ保存する。保存と適用は別で、TF node の再起動後に反映される。未計測値は補完しない。[測定・UI・TFの手順](vehicle_layout.md)

起動はフォアグラウンドで行い、終了は Ctrl+C。Docker の準備、ROS package の build、node 起動、設定保存、再起動による適用、実機確認はそれぞれ分けて扱う。

## profile の追加方針

日常の入口は `scripts/bringup.sh`、一覧は `scripts/lib/bringup.sh`、構成の組み合わせは `atd_bringup`、各 node の実装・launch・config は担当 package 内に置く。新しい profile は実装済みの launch・引数・対象ホストと対応させ、一覧・操作説明・起動テストを更新する。自律走行の profile は対応する実装に合わせて追加する。

## プロポ手動走行中に自己位置を確認する

1. Notebook で左右の Infra と車体 TF を記録した走行 A から cuVSLAM 地図と VGL 地図を作り、点群取得・HDMap 編集・Raceline 生成を完了する。地図の車体フレームは後輪軸中心の `base_link` を使う。
2. 同じコースを別に走行した bag B を Notebook に転送する。Notebook Docker 内で初回・パッケージ更新時に `atd-build --packages-up-to atd_localization` を実行し、`atd-web` を起動する。[自己位置検証](http://127.0.0.1:8766/#localization)で地図と bag B を選んで検証を開始する。結果では Raceline と推定軌跡を重ね、喪失前後の観測地点、初期位置の確定時間、追跡時間率、末尾の状態を確認できる。CLI での再生・保存先は [検証 package](../ros2_ws/src/atd_localization/README.md) を参照。
3. Notebook の「データ転送」から同じ地図を Jetson に送る。VGL の export も転送し、その Jetson 用の TensorRT engine を準備する。D455 の左右 IR 解像度・optical frame と車体取付 TF は地図作成時に合わせる。[VGL のモデル転送・engine build](vgl_models.md)
4. Jetson ホストのリポジトリで実行する。既に `manual` で起動中の場合は先にその launch を終了する。

   ```bash
   ./scripts/build.sh --packages-up-to atd_bringup
   ./scripts/bringup.sh localization
   ```

   地図を直接指定する場合は `./scripts/bringup.sh localization map_id:=course_a`。対応する engine が複数あるときは、表示された候補を `engine_dir:=models/vgl/<asset>/engines/<target>/<build>` で選ぶ。左右 IR を有効にした保存済み D455 設定が使われる。運転にはプロポ・Pico の手動モードを使う。
5. Notebook Docker で `atd-web` を起動し、「データ転送」の SSH 接続先を確認して [ライブ画面](http://127.0.0.1:8766/#live)を開く。「Jetson の地図を表示」で青い Raceline に緑の実走軌跡と車両前方を示す矢印が重なる。車速は VO の車体前後速度 [m/s] で、後退は負。初期位置待ち・地図照合中・追跡中・追跡喪失・更新途絶を区別する。同じ ID でも cuVSLAM 地図や HDMap の版が異なる場合は重畳を保留する。再照合を確認できない場合は、プロポで停車し、この profile を起動し直して初期位置を確認する。

地図作成・モデル選択・別 bag の再生と Jetson での手動走行はそれぞれ個別に検証する。実機の GPU における初期化時間・追跡率・位置精度はまだ未計測。[NVIDIA cuVSLAM](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)

## タミヤ ESC の手順試験

1. Jetson で Rev6 Pico・プロポ受信機・ESC を準備し、最初は駆動輪を浮かせる。プロポのスロットルを中立、CH3 を HOST 側にする。Jetson ホストで `./scripts/build.sh --packages-up-to atd_bringup` を実行し、対話端末で `./scripts/bringup.sh esc-trial` を起動する。制御用 USB CDC0 が複数見つかった場合は、使う機体の `port:=/dev/serial/by-id/…-if00` を指定する。
2. Notebook Docker で `atd-web` を起動し、Studio の「データ転送」で Jetson の SSH 接続先を設定する。「ESC 試験」で動作、出力の大きさ、保持時間を並べて予約する。ステアリングは試験中、中立に固定する。
3. Jetson 端末に表示された手順・対象ポートを確認し、試験 ID を入力する。初期例では前進 → 制動 → 停止確認 → 中立 → 後退 → 中立の順に出力する。「停止確認」では直前の負指令を保持する。車輪の停止を目視して端末に `STOP` を入力する。設定時間内に確認できなければ試験を中止する。後退中に「前進」を置くと直接正の指令に切り替わる。
4. 試験中は UI でステップ、符号付き指令、基板報告の ESC パルス幅 [µs]、推定 ESC 状態を確認する。UI の中止は Jetson への要求。Jetson 端末の Ctrl-C でも試験を中止できる。プロポに切り替えると Pico が手動操作を引き継ぐ。試験終了時は HOST 出力を解除する。試験ごとの手順・結果は Jetson の `/data/results/runs/esc_trials/<id>.json`、時系列は同名の `.jsonl` に残る。

各手順は、保持時間内に Pico から指令と一致する新しい PWM 報告を受けて次へ進む。状態報告は 20 Hz のため、ごく短い指令は基板で出力されても観測できない場合がある。その場合は未確認として試験を終了し、時間を延長して後退へ移ることはない。

既定の目視試験はカメラ・地図・Joy を使わない。実車の自己位置推定が別途稼働している場合は `./scripts/bringup.sh esc-trial stop_source:=localization` で速度による停止確認を選べる。D455・車両 TF と `atd_localization/localization.launch.py` を同じ Jetson で起動しておく。試験セッションが Web bridge を起動するため、同じポートの bridge は一つにする。停止推定には同じ追跡区間の新しい車体速度・ヨーレートを使う。駆動輪を浮かせた場合は車体が動かず、車輪だけが回転するため、停止確認は目視を選ぶ。

前進・制動・後退はユーザー指定の ESC 挙動に基づく推定。Rev6 の基板報告で確認できるのは操作経路と出力設定のパルス幅。実際の回転方向、制動力、ニュートラルの必要時間は対象のモータ・ESC で確認する。
