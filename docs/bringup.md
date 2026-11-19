# Jetson の bringup

`scripts/bringup.sh` で ROS 起動 profile を選び、Jetson の起動済み ATD Docker 内で実行する。先に `scripts/dev.sh` でコンテナを起動し、必要な package を build する。

```bash
# Jetson ホスト、プロジェクト直下
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh
```

引数なしでは fzf（未導入時は番号）で profile を選ぶ。`manual` / `d455` ではストリーム設定、`localization` / `autonomous` では保存地図と走行ラインを選ぶ。

| profile | 起動する処理 | 起動先 |
| --- | --- | --- |
| `joy-drive` | DualSense 手動走行、入力監視、入力記録待機、bridge。カメラ不要 | `atd_bringup joy_drive.launch.py` |
| `manual` | DualSense 手動走行、D455、車体static TF、Joy・Pico入力、地図用記録待機、bridge | `atd_bringup manual.launch.py` |
| `localization` | センサ・入力と cuVGL・cuVSLAM・前方軌道・制御計算を監視。DualSense 走行は `joy_drive:=true` で有効 | `atd_bringup localization.launch.py` |
| `autonomous` | 低速・前進限定の自動走行。Pico の CH3 でプロポと自動運転を切替 | `atd_bringup autonomous.launch.py` |
| `d455` | D455 と車体static TF | `atd_bringup d455.launch.py` |
| `esc-trial` | ESC 試験用 bridge と端末での手順実行 | `atd_bringup esc_trial_session` |
| `input` | Joy・Pico入力、役割処理、入力記録待機、bridge | `atd_web_bridge input.launch.py` |
| `bridge` | 観測・設定bridgeのみ | `atd_web_bridge bridge.launch.py` |

記録 manager は待機しており、Joy・プロポの割当操作で記録を開始・停止する。プロポの基本走行は Pico 内で完結する。全 profile は Jetson ローカル専用。Notebook は Web UI と解析を担当する。

## DualSense で手動走行する

1. Jetson に Rev6 Pico を USB 接続し、Jetson ホストで `./scripts/bluetooth.sh` を実行して DualSense に接続する。初回は駆動輪を浮かせ、プロポの CH3 をプロポ側、スロットルを中立にする。[DualSense のペアリング手順](setup/jetson.md#dualsense)
2. Jetson ホストで上記の build 後、`./scripts/bringup.sh joy-drive` を実行する。初回の軸設定には Notebook の「コントローラ」で物理配置と「操作の役割」を保存し、全入力を離して「操作要求を再開」を使う。たとえば操舵は左スティック、スロットルは R2 から L2 を引く割り当てにできる。
3. CH3 を Jetson 側に切り替え、Joy の操舵とスロットルを一度中立にする。運転許可ボタンを割り当てた場合は、中立のままボタンを押し、運転中は保持する。Jetson の端末で HOST の有効化を確認してから小さく操作し、左右と前進側の向きを確かめる。
4. CH3 をプロポ側に戻すと、Pico が受信機の CH1・CH2 でサーボと ESC を直接駆動する。再び Jetson を選択したときや USB・Joy の再接続後は、新しい中立入力から再開する。制動から後退への中立操作は運転者が行う。

D455 も使う日は `./scripts/bringup.sh manual`、保存地図・速度比較を併用する日は `./scripts/bringup.sh localization joy_drive:=true`。D455 を使ってプロポ走行だけを観察する場合は `./scripts/bringup.sh manual joy_drive:=false`。制御用は CDC0（if00）、テレメトリ用は CDC1（if02）。複数の Pico を接続している場合は `pico_serial` で1台に絞る。ESC 手順試験へ移るときは走行の bringup を先に終了する。[基板の操作条件](../ros2_ws/src/atd_vehicle_interface/README.md#dualsense-手動走行)

Notebook の Docker 内で `atd-web` を起動し、[コントローラ](http://127.0.0.1:8766/#input) を開く。CH3 を切り替えると、スイッチの位置と現在の運転元を確認できる。Jetson 側に切り替えて Joy が中立になるのを待っている間は、運転元が待機になる。DualSense の操舵・スロットルは −1〜+1、Pico が報告したサーボ・ESC のパルス幅は µs で表示する。プロポ側で Joy を動かしても、運転元はプロポのままになる。走行後は同じ画面の「記録した走行」でこれらの信号を振り返れる。[記録と時系列表示の手順](recording.md#手動走行を-notebook-で振り返る)

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

1. Notebook で左右の Infra と車体 TF を記録した走行 A から cuVSLAM 地図と VGL 地図を作り、点群取得・HDMap 編集を行い、試したい Raceline・Centerline・Customline を生成して保存する。地図の車体フレームは後輪軸中心の `base_link` を使う。
2. 同じコースを別に走行した bag B を Notebook に転送する。Notebook Docker 内で初回・パッケージ更新時に `atd-build --packages-up-to atd_control atd_vehicle_calibration` を実行し、`atd-web` を起動する。[自己位置検証](http://127.0.0.1:8766/#localization)で地図と bag B を選んで検証を開始する。結果では Raceline と推定軌跡を重ね、喪失前後の観測地点、初期位置の確定時間、追跡時間率、末尾の状態を確認できる。CLI での再生・保存先は [検証 package](../ros2_ws/src/atd_localization/README.md) を参照。
3. Notebook の「データ転送」から同じ地図を Jetson に送る。VGL の export も転送し、その Jetson 用の TensorRT engine を準備する。D455 の左右 IR 解像度・optical frame と車体取付 TF は地図作成時に合わせる。[VGL のモデル転送・engine build](vgl_models.md)
4. Jetson ホストのリポジトリで実行する。既に `manual` で起動中の場合は先にその launch を終了する。

   ```bash
   ./scripts/build.sh --packages-up-to atd_bringup
   ./scripts/bringup.sh localization
   ```

   地図を直接指定する場合は `./scripts/bringup.sh localization map_id:=course_a`。対応する engine が複数あるときは、表示された候補を `engine_dir:=models/vgl/<asset>/engines/<target>/<build>` で選ぶ。左右 IR を有効にした保存済み D455 設定が使われる。運転にはプロポ・Pico の手動モードを使う。
5. Notebook Docker で `atd-web` を起動し、「データ転送」の SSH 接続先を確認して [ライブ画面](http://127.0.0.1:8766/#live)を開く。「Jetson の地図を表示」で基準ラインと実走軌跡、車両の向きを重ねて確認できる。車速は VO の車体前後速度 [m/s] で、後退は負。初期位置待ち・地図照合中・追跡中・追跡喪失・更新途絶を区別する。同じ ID でも cuVSLAM 地図や HDMap の版が異なる場合は重畳を保留する。再照合を確認できない場合は、プロポで停車し、この profile を起動し直して初期位置を確認する。

同じ `localization` 起動で `atd_planning` が後輪軸中心の姿勢を選んだラインへ投影する。プロポを手動側にして走行し、ライブ画面でライン上の前方軌道、横ずれ [m]、目標速度と推定速度 [m/s] を確認する。動かすのはプロポと Pico で、planner が配信するのは観測用の `/planning/status`。対話起動では地図の選択後、生成・保存済みの候補からラインを選べる。地図 ID を直接指定した場合の既定は Raceline。選んだラインが未生成なら前方軌道は無効になり、理由が表示される。

```bash
# Jetson ホスト。地図は起動時に選択
./scripts/bringup.sh localization line:=centerline
./scripts/bringup.sh localization line:=custom
```

別 bag の同じ走行を比較するには、Notebook の自己位置検証で走行結果を開き、基準ラインを選んで前方軌道を計算する。時刻スライダーと再生で、手動走行の姿勢に対応した計画を確認できる。各ラインの目標速度はそのラインの保存プロファイルから求める。[planner の仕様](../ros2_ws/src/atd_planning/README.md)

同じ画面で Pure Pursuit の目標点・先読み距離・目標曲率・等価前輪舵角を確認できる。Jetson のライブ計算は起動時に車両配置の実測ホイールベースを読み込む。未測定でも目標点・曲率を表示する。最小距離・先読み時間・最大距離は起動引数で指定する。以下は既定値で、長い先読みを指定すると planner の前方軌道も延びる。

```bash
./scripts/bringup.sh localization min_lookahead_m:=0.35 lookahead_time_s:=0.4 max_lookahead_m:=1.5
```

Notebook の別 bag 検証では同じ設定を画面で変更して、同じ時刻の結果を比較できる。この計算は保存済みの姿勢と速度を使う。実 bag から初めて軌跡を作る段階では、記録した D455 の左右画像・CameraInfo・TF を再生し、Notebook の GPU で cuVGL による初期姿勢候補と cuVSLAM による地図上の追跡を計算する。[Pure Pursuit の仕様](../ros2_ws/src/atd_control/README.md)

地図作成・モデル選択・別 bag の再生と Jetson での手動走行はそれぞれ個別に検証する。実機の GPU における初期化時間・追跡率・位置精度はまだ未計測。[NVIDIA cuVSLAM](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)

同じ起動で `atd_speed_control` が FF＋PID の計算スロットル、プロポの CH2 raw、DualSense の正規化入力を時刻で照合する。Notebook の「車両校正」で速度比較に採用した表を「ライブ」から Jetson の観測に適用する。再起動時は `/data/results/runs/vehicle/throttle_feedforward.json` から復元する。`feedforward_path` で保存先を指定できる。ゲインを指定する例は `./scripts/bringup.sh localization speed_kp:=40.0 speed_ki:=8.0 speed_kd:=1.0`。この値は観測を始めるための仮設定で、実車の同定結果は FF 表に含まれる。[別 bag とライブの比較操作](../apps/web/README.md#速度制御を手動入力と比較する)

## 低速前進の自動走行

**走行前の準備**　Jetson で手動走行用の `localization` を使い、同じ車両・コースで追跡できることを確認する。後輪軸中心の車体座標、実測したカメラ取付姿勢とホイールベース、速度付きの保存済み Raceline・Centerline・Customline のいずれかを使う。

1. 手動走行の校正結果から採用した前進 FF 表を Jetson の `/data/results/runs/vehicle/throttle_feedforward.json` に配置する。速度上限と走りたい目標速度が校正範囲に入る表を使う。[速度 FF 表の準備](vehicle_calibration.md)
2. 操舵角とサーボ PWM、前進側のプロポ CH2 raw と ESC PWM、低速前進から一続きで止まれたブレーキ量と保持時間を測り、Jetson の `/data/results/runs/vehicle/auto_drive.yaml` に記入する。[校正ファイルの作り方](vehicle_calibration.md#7-低速自動走行用の出力を校正する)
3. Jetson ホストで既存の手動走行・ESC 試験を終了し、プロポの CH3 を手動側にして起動する。

   ```bash
   ./scripts/build.sh --packages-up-to atd_bringup
   ./scripts/bringup.sh autonomous
   ```

   対話画面で地図とラインを選ぶ。引数での指定例は `./scripts/bringup.sh autonomous map_id:=course_a line:=centerline max_speed_mps:=0.4`。既定は 0.5 m/s、設定できる上限は 1.0 m/s。速度目標を FF＋PID の計算前に制限し、前進スロットルもこの速度に対応する校正範囲に制限する。

4. 追跡が成立して停車を確認したら CH3 を Jetson 側に切り替える。切替後の同じ観測時刻・地図・追跡区間に対する自己位置、Pure Pursuit、FF＋PID の結果と、基板の中立出力がそろうと発進する。CH3 を手動側へ戻すと Pico 自身が受信機の CH1・CH2 に切り替える。Rev6 の手動側のスイッチ確定時間は 40 ms で、Jetson との往復を必要としない。
5. 追跡喪失・観測の期限切れ・過速度では前進指令を取り消す。直前に正の速度と Pico の前進出力を確認済みなら、計測したブレーキを一続きで送り、中立にして HOST 出力を解除する。前進を確認できない場合は直接中立にする。USB 断では Pico の通信監視が HOST 出力を解除する。追跡が戻った後は、停車して CH3 を手動側から Jetson 側へ入れ直す。

初回は駆動輪を浮かせて操舵方向・CH3 の復帰を確認する。接地後の低速試験では、追跡喪失時の実停止距離と後退に入らないことを確認する。Jetson Docker 内の `ros2 topic echo /vehicle/auto_drive/status` で許可状態、理由、制動中かどうか、基板報告の PWM を確認できる。記録時は同じ状態と `/localization/status`、`/planning/status`、`/control/status`、`/control/speed_status` が bag に入る。追跡喪失中の実停止を PWM だけから判定することはできない。

Notebook の Docker 内で `atd-web` を起動し、「ライブ」を開くと、CH3 と操作元、停止理由、目標・推定速度、USB で送った指令と Pico の報告 PWM を同じ時間軸で確認できる。[Notebook での見方](../apps/web/README.md#自動走行の指令と基板出力を比較する)

## タミヤ ESC の手順試験

1. Jetson で Rev6 Pico・プロポ受信機・ESC を準備し、最初は駆動輪を浮かせる。プロポのスロットルを中立、CH3 を HOST 側にする。Jetson ホストで `./scripts/build.sh --packages-up-to atd_bringup` を実行し、対話端末で `./scripts/bringup.sh esc-trial` を起動する。制御用 USB CDC0 が複数見つかった場合は、使う機体の `port:=/dev/serial/by-id/…-if00` を指定する。
2. Notebook Docker で `atd-web` を起動し、Studio の「データ転送」で Jetson の SSH 接続先を設定する。「ESC 試験」で動作、出力の大きさ、保持時間を並べて予約する。ステアリングは試験中、中立に固定する。
3. Jetson 端末に表示された手順・対象ポートを確認し、試験 ID を入力する。初期例では前進 → 制動 → 停止確認 → 中立 → 後退 → 中立の順に出力する。「停止確認」では直前の負指令を保持する。車輪の停止を目視して端末に `STOP` を入力する。設定時間内に確認できなければ試験を中止する。後退中に「前進」を置くと直接正の指令に切り替わる。
4. 試験中は UI でステップ、符号付き指令、基板報告の ESC パルス幅 [µs]、推定 ESC 状態を確認する。UI の中止は Jetson への要求。Jetson 端末の Ctrl-C でも試験を中止できる。プロポに切り替えると Pico が手動操作を引き継ぐ。試験終了時は HOST 出力を解除する。試験ごとの手順・結果は Jetson の `/data/results/runs/esc_trials/<id>.json`、時系列は同名の `.jsonl` に残る。

各手順は、保持時間内に Pico から指令と一致する新しい PWM 報告を受けて次へ進む。状態報告は 20 Hz のため、ごく短い指令は基板で出力されても観測できない場合がある。その場合は未確認として試験を終了し、時間を延長して後退へ移ることはない。

既定の目視試験はカメラ・地図・Joy を使わない。実車の自己位置推定が別途稼働している場合は `./scripts/bringup.sh esc-trial stop_source:=localization` で速度による停止確認を選べる。D455・車両 TF と `atd_localization/localization.launch.py` を同じ Jetson で起動しておく。試験セッションが Web bridge を起動するため、同じポートの bridge は一つにする。停止推定には同じ追跡区間の新しい車体速度・ヨーレートを使う。駆動輪を浮かせた場合は車体が動かず、車輪だけが回転するため、停止確認は目視を選ぶ。

前進・制動・後退はユーザー指定の ESC 挙動に基づく推定。Rev6 の基板報告で確認できるのは操作経路と出力設定のパルス幅。実際の回転方向、制動力、ニュートラルの必要時間は対象のモータ・ESC で確認する。
