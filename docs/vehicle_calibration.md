# 車両モデル作成と校正

プロポで手動走行した bag を Notebook へ移し、既存の offline VSLAM と `atd_vehicle_calibration analyze` で入力・車体運動の対応を確認する。最初は短い1 bag で記録・姿勢・解析・図の確認まで通し、別の走行で再現性を調べる。

今回作るのは**観測応答表**であり、車両の遅延・時定数や制御器を自動で確定するものではない。実車の校正値は未取得で、実測 bag・ROS/GPU 環境での校正と検証は未実施。エンコーダは使わず、RPM 換算も行わない。

## 1. 実測と記録設定を確認する

「車両設定」でホイールベースと D455 の取付位置・姿勢を入力し、停車中にノードを再起動して適用を確認する。原点 `base_link` は左右後輪の回転中心の中点、x 前・y 左・z 上。床からの高さや筐体中心を、そのまま取付 TF にしない。未計測をゼロや CAD 外接寸法で補完しない。[車両配置と TF](vehicle_layout.md)

Jetson ホストの ATD プロジェクト直下で実行する。開発 Docker は事前に起動しておく。

```bash
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh --configure manual recording_profile:=calibration
```

TUI で左右 Infra・加速度・ジャイロを有効にし、`unite_imu_method=0` で生の accel／gyro を別々に配信する。接続中の D455 が対応する組合せを選び、要求 FPS を実効値とみなさない。確認済み設定を使う場合は、次回から `./scripts/bringup.sh manual recording_profile:=calibration` で起動できる。`manual` の既定は `mapping` なので校正では明示する。

[calibration.yaml](../ros2_ws/src/atd_recording/config/calibration.yaml) の必須入力は次の8 topic。

| 入力 | topic |
| --- | --- |
| 左右の rectified IR | `/camera/camera/infra1/image_rect_raw`、`/camera/camera/infra2/image_rect_raw` |
| 左右の CameraInfo | `/camera/camera/infra1/camera_info`、`/camera/camera/infra2/camera_info` |
| 車体取付・カメラ内部の static TF | `/tf_static` |
| 生加速度・角速度 | `/camera/camera/accel/sample`、`/camera/camera/gyro/sample` |
| プロポ入力・基板状態 | `/vehicle/pico/telemetry` |

RGB・Depth は必須ではない。起動だけで記録は始まらず、割り当てた Joy の記録ボタンまたはプロポの記録スイッチで開始・停止する。プロポで運転し、Joy は記録ボタンだけに使うこともできる。[割当と記録状態](recording.md)

## 2. 最初の1 bag を取る

同じ取付・設定のまま、短い区間に分けて収集する。これは試行の組合せであり、安全な速度・入力上限を決めたものではない。

| 区間 | 操作と確認するもの |
| --- | --- |
| 静止 | 操作せず、VSLAM の見かけの移動、gyro バイアス・振動、RC 中間値を確認 |
| 低速直進 | 小さい入力を段階的に変えて保持し、加速・定常速度を確認。実際に確認したニュートラルへ戻して惰行も記録 |
| 低速の左右旋回 | 左右を別々に、操舵量を保持して yaw rate・曲率・中心ずれを確認 |

搭載 firmware の版・校正値・反転、ESC 設定、タイヤ・積載、電池、路面、各区間の操作内容を残す。CH1／CH2 の raw や UI の PWM 相当値は、実タイヤ角・出力パルス・ESC ニュートラルの測定値ではない。`vbec_mv` も駆動電池電圧に読み替えない。

停止操作後は UI の「保存確定」と保存先を確認してから bringup を終了する。Jetson Docker 内では実際の bag ID で確認する。

```bash
ros2 topic echo /recording/status --once
ros2 bag info '/data/record/rosbags/<bag_id>'
```

保存確定は、記録型・非ゼロ件数・保存ファイル等の確認結果である。TF の内容、画像同期、実効 FPS、元 stamp、RC の有効性まで合格した意味ではない。停止指令・ニュートラルと実際の停止も区別する。

## 3. Notebook へ受信し、姿勢を作る

1. UI「データ転送」で、保存完了した bag ディレクトリ全体を Notebook の記録先へ SCP 受信する。元 bag は保持する。
2. Notebook の ROS／CUDA Docker 内で `atd-web` を起動する。初回の `atd_mapping` build 等は [Web アプリの準備](../apps/web/README.md)に従う。
3. UI「地図とライン」→「cuVSLAM 地図を作成」で、この bag と新しい地図 ID を選ぶ。左右画像・CameraInfo・optical frame と base frame を実 bag に合わせる。
4. `/data/results/maps/<map_id>/mapping.json` と `nvidia/` 以下のログ・姿勢ファイルを確認する。解析には同じ走行の連続した `odom_poses.tum` を使い、出力が表す子フレームを導入版の設定・ログで確かめる。

現在の [offline 実装](../apps/web/backend/src/atd_api/offline.py)は rectified stereo、平面拘束なしで `edex`・`compute_poses` を実行する。IMU を記録しても自動で VIO にはならない。VGL 準備・点群取得は解析の前提ではなく、表示用 snapshot や間引いた keyframe を連続速度の代わりにしない。

## 4. 解析を実行する

Notebook Docker 内で実行する。`<...>` は実在する入力に置き換え、`--pose-frame` を推測で埋めない。

```bash
/workspaces/atd/scripts/build.sh --packages-up-to atd_vehicle_calibration
source /workspaces/isaac_ros-dev/install/setup.bash

ros2 run atd_vehicle_calibration analyze \
  --bag '/data/record/rosbags/<bag_id>' \
  --poses '/data/results/maps/<map_id>/nvidia/poses/odom_poses.tum' \
  --pose-frame '<確認したframe>'
```

`build.sh` は子プロセスなので、build 後の同じシェルにも `install/setup.bash` を読み込む。解析は VSLAM を再実行せず、上で用意した姿勢を使う。

| 任意指定 | 使う場合・省略時 |
| --- | --- |
| `--wheelbase-m '<実測m>'` | 指定すると等価舵角も算出。省略時も曲率は出るが舵角は空欄 |
| `--gravity-world-mps2 GX GY GZ` | VSLAM 親座標での重力方向を確認した場合だけ指定。未指定時も VO 加速度・IMU 比力・gyro は出るが、IMU 重力補正加速度は空欄 |
| `--output '/data/results/runs/vehicle_calibration/<run>'` | 新規保存先。省略時も同 root 以下に新しい run を作り、既存結果を上書きしない |

微分窓、時刻差・速度の閾値、TF 検査、CSV の列は [package README](../ros2_ws/src/atd_vehicle_calibration/README.md)を参照する。時計のずれやカメラ取付不足を、ゼロ・identity・自動の時間ずらしで補完しない。

## 5. 図と要約を確認する

| 生成物 | 最初に見るもの |
| --- | --- |
| `response.png` | CH1／CH2、符号付き縦速度、加速度、yaw rate の時間対応。欠落区間をまたぐ線や、実際の操作と逆の符号がないか |
| `timeseries.csv` | `vo_vx_mps` は前進正・後退負。`motion_valid` と応答集計の `valid`、除外理由、未計算の空欄を区別 |
| `summary.json` | 元 bag・姿勢との対応、入力・運動の除外理由、採用点数、各条件を満たした範囲 |

VSLAM の運動量は推定値であり真値ではない。静止・既知距離の直進・左右旋回と照合する。取付オフセットを補正した後輪軸の VO 加速度と、車体軸へ回転しただけの D455 センサ位置の IMU 加速度は、同じ物理点ではない。

応答表は CH2 と加速度・定常速度、CH1 と yaw rate・曲率を raw・速度帯・前後方向で集計する。未観測セルは未知のままで、入力の物理符号・PWM 端点や遅延・時定数を fit 済みとはしない。終了コード0も実車モデルの合格ではなく、有効なプロポ手動試料なしの場合は2になる。

## 6. 別の走行で確かめる

別の bag・姿勢を指定した解析に `--reference-summary '/data/results/runs/vehicle_calibration/<基準run>/summary.json'` を追加すると、過去の応答表との再現性を比較できる。同じ bag は参照に使えない。比較可能な点・範囲外・評価点数・誤差を分け、評価点なしを誤差0としない。

次の試行では加速・惰行・制動・後退を分け、操舵量・左右・入力の往復・速度別に条件を広げる。加速だけで走行抵抗や制動を確定せず、低速の実効舵角をサーボ角・タイヤ実測角と同一視しない。電池・路面・取付を変えた場合は条件差を残す。

raw bag・公式成果物・実測設定を保持し、推定値で上書きしない。単体テスト、ROS build/install、実 GPU の offline 処理、実 bag の解析、別走行での車両モデル検証をそれぞれの完了条件として扱う。
