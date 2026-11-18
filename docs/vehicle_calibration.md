# 車両モデル作成と校正

プロポで手動走行した bag を Notebook へ移し、ATD Studio の「車両校正」で速度・加速度・操舵応答を確認する。最初は短い1 bag で記録・姿勢・解析・図の確認まで通し、別の走行で再現性を調べる。

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

停止操作後は UI の「保存確定」と保存先を確認してから bringup を終了する。記録状態を個別に調べたい場合は Jetson Docker 内で次のコマンドを使える。

```bash
ros2 topic echo /recording/status --once
ros2 bag info '/data/record/rosbags/<bag_id>'
```

保存確定は、記録型・非ゼロ件数・保存ファイル等の確認結果である。TF の内容、画像同期、実効 FPS、元 stamp、RC の有効性まで合格した意味ではない。停止指令・ニュートラルと実際の停止も区別する。

## 3. Notebook の UI で解析する

Notebook の Docker 内で初回・自作 package 更新時に `atd-build --packages-up-to atd_mapping atd_vehicle_calibration` を実行する。通常は同じ Docker 内で `atd-web` を起動し、Notebook の Chrome で [ATD Studio の車両校正](http://127.0.0.1:8766/#calibration)を開く。[Web アプリの準備](../apps/web/README.md)

1. Jetson にある bag は「データ転送」で保存確定後のディレクトリ全体を受信し、「この bag を車両校正へ」を押す。すでに Notebook にあれば「車両校正」で直接選ぶ。同名の走行は親フォルダを含むパスで識別できる。
2. 「解析を開始」を押す。Notebook が必須 topic・記録件数・データファイルを検査し、標準 D455 のステレオ画像から軌跡を生成して解析する。対応する保存済み軌跡があれば自動で再利用する。候補が複数あるときは生成元の地図を選ぶ。
3. 実行完了後、同じ画面で速度・加速度・操舵応答のグラフを開く。等価舵角も求めたい場合は開始前に実測ホイールベースを **mm** で入力する。空欄でも速度・加速度・yaw rate・曲率を解析できる。

標準の新規生成では、カメラの取付変換を渡した車体フレームと、ROS 座標系に変換済みの odometry ファイルの SHA-256 を記録する。再利用時には元 bag のパス・metadata の SHA-256 と軌跡の SHA-256 を照合する。フレームの記録がない旧軌跡は、導入版の設定・ログで確認した TF 名を画面で指定する。[NVIDIA の地図生成処理](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mapping_and_localization/blob/v5.0-0/isaac_mapping_ros/scripts/create_map_offline.py)・[cuVSLAM の rig 姿勢出力](https://github.com/nvidia-isaac/cuVSLAM/blob/main/tools/cuvslam_api_launcher/cuvslam_api_launcher.cpp#L540-L553)

「ジョブ」で進捗・ログを確認できる。解析中にページを閉じても、Notebook の Docker が稼働していれば処理は継続する。生成した公式成果物は `/data/results/maps/<map_id>/nvidia/`、解析結果は `/data/results/runs/vehicle_calibration/<run_id>/` に保存する。解析条件を変えてやり直すときは同じ bag の軌跡を再利用できる。

標準の姿勢生成は rectified stereo で平面拘束を無効にし、`edex`・`compute_poses` を実行する。IMU は校正解析で VO と照合する。独自の地図作成設定が必要な場合は「地図とライン」で事前に軌跡を作成できる。VGL 準備や点群取得は不要。[姿勢の生成処理](../apps/web/backend/src/atd_api/offline.py)

## 4. 結果を読む

「車両校正」の結果から応答グラフを開き、時系列と要約を保存できる。有効なプロポ手動試料が 0 件の場合も診断用の結果を開き、記録と除外理由を確認できる。

| 生成物 | 最初に見るもの |
| --- | --- |
| `response.png` | CH1／CH2、符号付き縦速度、加速度、yaw rate の時間対応。欠落区間をまたぐ線や、実際の操作と逆の符号がないか |
| `timeseries.csv` | `vo_vx_mps` は前進正・後退負。`motion_valid` と応答集計の `valid`、除外理由、未計算の空欄を区別 |
| `summary.json` | 元 bag・姿勢との対応、入力・運動の除外理由、採用点数、各条件を満たした範囲 |

VSLAM の運動量は推定値であり真値ではない。静止・既知距離の直進・左右旋回と照合する。取付オフセットを補正した後輪軸の VO 加速度と、車体軸へ回転しただけの D455 センサ位置の IMU 加速度は、同じ物理点ではない。

応答表は CH2 と加速度・定常速度、CH1 と yaw rate・曲率を raw・速度帯・前後方向で集計する。未観測セルは未知のままで、入力の物理符号・PWM 端点や遅延・時定数を fit 済みとはしない。終了コード0も実車モデルの合格ではなく、有効なプロポ手動試料なしの場合は2になる。

## 5. Raceline の制限と速度フィードフォワード表を作る

「車両校正」で解析結果を選び、**「Raceline の制限候補とスロットル表を確認」**を開く。保存済みの `summary.json` と `timeseries.csv` を使うので、既存の走行でも VO・校正解析の再実行は不要。各制限には根拠になった区間の時刻、旋回方向、速度域、観測値、試料数を表示する。

| 候補 | 使用する走行区間 | Raceline 生成での用途 |
| --- | --- | --- |
| 曲率上限 `curvature_limit` [1/m] | 1.0 m/s 以下で入力を保持した左右の定常旋回。各区間の `abs(yaw_rate / vx)` | 最小曲率の最適化で許容する曲率 |
| 横加速度 `lateral_accel` [m/s²] | 前進の定常旋回。平面・小横滑りを仮定した `abs(vx × yaw_rate)` | 曲率に応じたコーナー速度の上限 |
| 加速・減速 `accel` / `decel` [m/s²] | プロポ手動で直進し、入力を保持して加速・減速した区間 | 加速側と減速側それぞれの速度変化の上限。減速は正の大きさで指定 |

初版では、保持区間内の中央値を使い、別の操作を挟んで **2 回以上到達した大きさの 80%** を候補にする。旋回は左右を個別に評価し、小さい側を採る。80% は初期候補を控えめにするための係数で、タイヤの摩擦限界や実車の安全率を測定した値ではない。観測減速度には惰行と ESC 制動の両方が含まれ得る。採用前に、根拠区間で行った減速操作を確認する。

1. 地図を選び、現在値と候補を比較して採用する項目にチェックする。観測が足りない項目は理由とともに「候補なし」と表示される。
2. **「選択した制限を地図に採用」**を押す。この地図の採用設定はチェックした組み合わせに入れ替わる。選ばなかった項目は地図画面で個別に設定できる。
3. **「Raceline 生成画面を開く」**からラインを生成する。採用設定と出典は地図に保存され、次回以降の Raceline 生成にも使われる。以前の Raceline があれば再生成が必要。手動設定に戻すときは地図画面で採用を解除する。

同じ画面の**速度フィードフォワード表**は、前進の直進定常区間から、目標速度 [m/s] とそれを維持したスロットル入力 [S.BUS CH2 raw] を対応付ける。速度域の分離が確認でき、単調な対応が得られた 2 入力帯以上で JSON を保存できる。表には走行、校正結果の識別情報、補間可能な速度範囲が付く。使い方は範囲内の線形補間。未観測の停止点や最高速は追加しない。速度制御器に使う際は、プロポ CH2 と Jetson から Pico に送る指令値との対応を別途確認する。

## 6. 別の走行で確かめる

「車両校正」で別走行の解析結果を比較元に指定すると、過去の応答表との再現性を比較できる。同じ bag は参照に使えない。比較可能な点・範囲外・評価点数・誤差を分け、評価点なしを誤差0としない。

次の試行では加速・惰行・制動・後退を分け、操舵量・左右・入力の往復・速度別に条件を広げる。加速だけで走行抵抗や制動を確定せず、低速の実効舵角をサーボ角・タイヤ実測角と同一視しない。電池・路面・取付を変えた場合は条件差を残す。

raw bag・公式成果物・実測設定を保持し、推定値で上書きしない。単体テスト、ROS build/install、実 GPU の offline 処理、実 bag の解析、別走行での車両モデル検証をそれぞれの完了条件として扱う。

時刻差・微分窓を変更する場合や、世界座標で確認した重力ベクトルを指定して IMU 加速度を補正する場合は CLI を使う。既定値と各列の定義、実行例は [解析 package の README](../ros2_ws/src/atd_vehicle_calibration/README.md) を参照する。
