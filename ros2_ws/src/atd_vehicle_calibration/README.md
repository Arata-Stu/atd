# atd_vehicle_calibration

Notebook の Docker 内で、手動走行 bag と offline VSLAM の姿勢ファイルを使って車両応答を解析する。基準は後輪車軸中心 `base_link` の縦速度とする。VSLAM の実行、走行指令の送信、記録開始、車両設定や制御器へのモデル自動反映は行わない。

本車両ではエンコーダを使わず、RPM／edge 数からの速度校正は対象外とする。VSLAM による速度・yaw rate と、プロポ入力・基板状態・IMU を対応づける。実車の校正結果と、解析処理の portable test の結果を区別する。

## 起動

事前に Jetson で `recording_profile:=calibration` の bag を記録して Notebook へ受信し、既存の offline VSLAM で姿勢ファイルを作る。記録から解析までの手順は [車両モデル作成と同定](../../../docs/vehicle_calibration.md) を参照する。

Notebook の Docker 内で実行する。`<...>` は実在する入力に置き換える。

```bash
/workspaces/atd/scripts/build.sh --packages-up-to atd_vehicle_calibration
source /workspaces/isaac_ros-dev/install/setup.bash

ros2 run atd_vehicle_calibration analyze \
  --bag '/data/record/rosbags/<bag_id>' \
  --poses '/data/results/maps/<map_id>/nvidia/poses/odom_poses.tum' \
  --pose-frame '<確認したframe>'
```

`build.sh` は子プロセスなので、build 後に使うシェルでも `install/setup.bash` を source する。

| 引数 | 内容 |
| --- | --- |
| `--bag` | metadata と MCAP を含む元の bag ディレクトリ |
| `--poses` | この bag に対応する offline VSLAM の姿勢ファイル |
| `--pose-frame` | 姿勢ファイルが表す子フレーム。出力版・設定を確認して明示する |
| `--wheelbase-m` | 実測ホイールベースを m 単位で明示する場合の任意指定 |
| `--gravity-world-mps2 GX GY GZ` | VSLAM 親座標で重力方向を確認できた場合だけ指定する重力ベクトル（m/s²） |
| `--reference-summary` | 別の bag で作った run の `summary.json`。今回の試料を過去の観測応答表と比較する |
| `--output` | 新しい解析結果ディレクトリ。省略時は `/data/results/runs/vehicle_calibration/<run>/` |

`--pose-frame` はファイル名や UI の設定から推測しない。カメラ姿勢を入力する場合は、bag 内の静的 TF を使って後輪車軸中心へ変換する。すでに `base_link` の姿勢なら取付変換を再適用しない。

ホイールベースを省略しても曲率を求めるが、等価舵角は空欄とする。重力ベクトルを省略しても VO 由来の加速度・IMU specific force・gyro を出力し、IMU の重力補正後の加速度だけを空欄にする。重力方向やホイールベースをゼロ・既定値で補完しない。

微分・対応付けの初期設定は以下のとおり。これらは解析用のソフトウェア初期値であり、実車で校正した応答時間や走行限界ではない。

| 引数 | 既定 | 用途 |
| --- | --- | --- |
| `--window-s` | 0.3 s | 中央の重み付き局所多項式微分に使う窓幅 |
| `--max-gap-s` | 0.2 s | 連続区間を分ける時間差 |
| `--max-sync-s` | 0.05 s | 入力と運動を対応づける時刻差の上限 |
| `--min-speed-mps` | 0.15 m/s | 曲率を計算する最低速度の絶対値 |

姿勢入力は ROS 秒の `stamp tx ty tz qx qy qz qw` の8列 TUM とし、連続する `odom` 姿勢を使う。map のループ補正を含む姿勢や間引いた keyframe は速度基準にしない。CLI は既知の SLAM／keyframe 出力名を拒否するため、別名へ変えて回避しない。

ATD の `mapping.json` が姿勢出力の近くにある場合は、生成元 bag の metadata hash を照合する。外部ファイルは利用者が同じ走行との対応を確認し、`user_supplied` の対応付けとして扱う。TUM 単体には VO の追跡状態がないため、数値・連続性の検査を通過しても tracking 成功や全区間の精度を保証しない。元の offline ログと実際の動きも確認する。

## 出力

| ファイル | 内容 |
| --- | --- |
| `timeseries.csv` | 元 stamp に対応する基準運動・受信入力・IMU 等の時系列 |
| `summary.json` | 入力と設定、解析結果・モデル候補の要約 |
| `response.png` | 時間軸・単位・符号を示す比較図 |

新しい run に保存し、元 bag・姿勢ファイル・以前の解析結果を上書きしない。解析結果は推定候補であり、制御器の設定を更新するものではない。

CSV の `vo_vx_mps` は前進正・後退負の後輪軸縦速度。除外された入力・運動試料と、採用条件は summary の `inputs`／`motion`／`analysis` 等へ残す。図の線が描けたことだけでモデルの有効性を判定しない。

`summary.json` は生成の最後に公開する。失敗した run に CSV や画像だけが残った場合も、完成した解析結果と混同しない。

終了コードは 0＝有効な対応試料あり、1＝入力・解析エラー、2＝時系列は保存したが有効なプロポ手動試料なし。0 は実車モデルの検証合格を意味しない。2 の場合は summary の除外理由を確認する。

## 初期の応答モデルと比較

初期実装は、受信 raw と運動の**観測応答表**を作る。入力の端点・反転・PWM 出力を仮定したアクチュエータモデルや、遅延・時定数を持つダイナミクスの係数を自動で fit するものではない。summary に `analysis.method.actuator_model_fitted=false` を残す。

| `analysis.responses` | 対応関係 |
| --- | --- |
| `throttle_acceleration` | CH2 raw・速度帯・前後方向ごとの縦加速度 |
| `throttle_steady_speed` | 入力が安定した持続区間の CH2 raw と速度 |
| `steering_response` | CH1 raw・速度帯・前後方向ごとの yaw rate・曲率・任意の等価舵角 |

各区間・表に標本数と統計量を残し、空のセルは未知として保持する。加速／減速／定常／低速、左／右／直進の区間分けと、入力が安定したかは解析閾値による分類であり、ESC の前進・制動・後退モードを測定した結果ではない。

別の bag で検証する場合は、その bag・姿勢を指定したコマンドに `--reference-summary '/data/results/runs/vehicle_calibration/<基準run>/summary.json'` を追加する。

これは同じ条件のセルに対する観測応答の再現性比較で、連続時間モデルの予測性能を保証しない。同一 bag の参照は拒否し、比較可能な点数・RMSE と、参照表でカバーできない条件を分けて読む。比較点がない場合の誤差を0と解釈しない。

### 時系列を読むときの主な列

| 列 | 意味 |
| --- | --- |
| `stamp`, `motion_valid`, `valid`, `reason` | 元の運動推定時刻、運動量の可否、応答集計への採用可否、除外理由 |
| `segment`, `motion_segment`, `pico_segment` | 対応付け・VO・Pico の連続区間 |
| `steering_raw`, `throttle_raw`, `pico_age_s` | 直前のプロポ受信値と、運動時刻との差 |
| `vo_vx_mps`, `vo_ax_mps2`, `vo_yaw_rate_rps` | 後輪軸中心の符号付き縦速度、運動学的縦加速度、body angular velocity Z |
| `curvature_inv_m`, `equivalent_steering_rad` | 十分な速度の区間の曲率、実測ホイールベース指定時の実効舵角 |
| `imu_accel_*_specific_mps2`, `imu_gyro_z_rps` | 車体軸へ回転したセンサ位置の比力、gyro Z |
| `imu_ax_at_sensor_mps2` | 明示した重力を補正したセンサ位置の加速度。未指定時は空欄 |
| `imu_accel_dt_s`, `imu_gyro_dt_s` | 採用した IMU 時刻 − 運動時刻。正なら未来側の近傍値 |

Pico は同じ連続区間の直前値を用い、未来の入力や再接続をまたいだ補間を使わない。IMU は同じ Pico 区間で指定時刻差以内の最も近い値を使う。欠落 IMU はゼロにせず空欄とし、VO と入力の解析可否とは分ける。

プロポの対応付けが無効でも、`motion_valid=true` の VO 量は保持する。`valid` は運動量の存在ではなく応答集計に採用できたかを表すため、2つを混同しない。

## 入力値の意味

bag reader は `/vehicle/pico/telemetry`、生の accel／gyro、`/tf_static` と左右の CameraInfo を読む。topic 型と CameraInfo の実 frame を照合し、元の message stamp と bag 受信時刻を別に保持する。左右 IR の画像 payload は読まず、外部の VSLAM 姿勢を使う。

bag には `base_link → camera_link` と左右 optical frame への静的な連結が必要で、姿勢ファイルが `base_link` を表す場合も取付 TF の不足を免除しない。静的 TF の多重親・循環・値の競合、static な `map`／`odom` 辺は拒否する。Pico の session・sequence・基板時計の進行、ROS stamp の逆行や重複も検査する。

プロポ手動の有効なサンプルだけを応答集計に使い、再接続・seq 欠落・長い gap の最初は基準取りとして除外する。数値の品質検査は速度20 m/s・角速度ノルム12 rad/s・加速度50 m/s²を既定の上限とし、超過を除外する。これは解析用 filter であり、実車の走行限界・許可値ではない。

Pico の CH1／CH2 は操舵・スロットルの受信 raw であり、実タイヤ角・実速度ではない。CDC1 telemetry は実際の ESC／サーボ出力パルスを含まないため、最初に求めるのはプロポ入力から車両運動までの全体応答である。S.BUS raw の PWM 相当換算を、測定した PWM や ESC ニュートラルとみなさない。

`base_link` は左右後輪の回転中心の中点、x 前・y 左・z 上。Pico の `header.stamp` は Jetson の受理時刻で、基板の `board_us` や D455 の測定時刻と無条件に同一視しない。取付 TF・姿勢フレーム・時刻が不足する入力をゼロや identity で補完しない。

VO はカメラの取付位置を含む剛体変換で `base_link` の姿勢へ変換してから運動を求める。yaw rate は body angular velocity の z 成分で、一般の3次元姿勢で Euler yaw の時間微分と同一ではない。

`T_WC` を入力姿勢、`T_BC` を bag の取付変換とすると `T_WB = T_WC · inverse(T_BC)`、`v_B = transpose(R_WB) · d p_WB / dt` とする。すでに `base_link` の姿勢なら再変換しない。中央の重み付き局所3次多項式で微分し、最低5点と前後の観測を要求する。窓が足りない端点へ外挿しない。

IMU は取付 TF の回転で車体軸へ変換するが、測定位置は D455 のセンサ位置のままである。重力補正を指定しても、取付オフセットによる接線・向心加速度の補正は行わない。後輪車軸中心の VO 加速度と IMU 加速度の単純な一致を、真値による検証としない。

VSLAM の位置・姿勢と、そこから微分した速度も推定値である。静止・既知距離の直進・左右旋回との照合を行い、実走行で使える条件を決める。学習用と検証用の試行を分け、加速だけで惰行・制動まで確定しない。

## 検証の区別

ROS 環境の package test は `colcon test --packages-select atd_vehicle_calibration` と `colcon test-result --verbose` で確認する。

解析処理の単体検証と、ROS package の build/install、実 bag の読み込み、実 GPU の offline VSLAM、実車モデルの校正は別の確認とする。現時点で実測 bag による校正値は取得していない。実データの結果を得るまで、推定値を車両の確定値として扱わない。
